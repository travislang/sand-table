import RPi.GPIO as GPIO
from utils.DRV8825 import DRV8825
import threading
import math
from time import sleep
from random import shuffle
from subprocess import call

from rpi_ws281x import PixelStrip, Color
from led_strip import *

from utils.process_files import get_files, process_new_files, read_track, get_max_disp


# Motor driver object init
M_Rot = DRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
M_Lin = DRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))

# Setting microstep size to 1/8
M_Rot.set_microstep('software','1/4step')
M_Lin.set_microstep('software','1/4step')

# Create NeoPixel object with appropriate configuration.
strip = strip_init()
strip_thread = LedStripThread()

# Setup for limit switches
outer_switch = 5
inner_switch = 6
# motor_relay = 23
# led_relay = 25
main_button = 26

GPIO.setmode(GPIO.BCM)
GPIO.setup(outer_switch, GPIO.IN)
GPIO.setup(inner_switch, GPIO.IN)
GPIO.setup(main_button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# GPIO.setup(motor_relay, GPIO.OUT)
# GPIO.setup(led_relay, GPIO.OUT)

# Slide thresholds
center_to_min = 250
outer_to_max = 250


# Run through the LED strip routine
def run_LedStrip():
    while strip_thread.running:
        print('LED Color wipe')
        strip_thread.colorWipe(strip, Color(0, 255, 0), 10)  # Blue wipe
        if not strip_thread.running: return
        print('LED Theater chase')
        strip_thread.theaterChase(strip, Color(127, 127, 127))  # White theater chase


# Functions defined for each motor thread
def run_MRot(steps, delay):
    if steps != 0 and delay >= 0:
        if steps > 0:
            M_Rot.turn_steps(Dir='forward', steps=abs(steps), stepdelay=delay)
        else:
            M_Rot.turn_steps(Dir='backward', steps=abs(steps), stepdelay=delay)

    M_Rot.stop()
    M_Rot.running = False

    # print("M_Rot done!")


def run_MRot_until(Dir, delay):
    if delay >= 0:
        while M_Rot.running:
            M_Rot.turn_steps(Dir=Dir, steps=1, stepdelay=delay)

    M_Rot.stop()

    # print("M_Rot done!")


def run_MLin(steps, delay):
    if steps != 0 and delay >= 0:
        if steps > 0:
            M_Lin.turn_steps(Dir='forward', steps=abs(steps), stepdelay=delay)
        else:
            M_Lin.turn_steps(Dir='backward', steps=abs(steps), stepdelay=delay)

    M_Lin.stop()
    M_Lin.running = False

    # print("M_Lin done!")


def run_MLin_until(steps, delay):
    run_MLin(steps, delay)
    M_Rot.running = False


# Calibrates the linear slide arm before starting the main program routine
def calibrate_slide():

    calibrated = False

    while not calibrated:
        minPos = M_Lin.turn_until_switch(Dir='backward', limit_switch=inner_switch, stepdelay=0.0002)
        maxPos = M_Lin.turn_until_switch(Dir='forward', limit_switch=outer_switch, stepdelay=0.0002) + minPos

        print((minPos, maxPos))
        totalDist = maxPos - minPos - center_to_min - outer_to_max
        print ("Travel distance: " + str(totalDist))

        sleep(.5)
        test_inner = M_Lin.turn_check_cali(Dir='backward', steps=totalDist + outer_to_max, limit_switch=inner_switch, stepdelay=0.0002)
        # sleep(.5)
        # test_outer = M_Lin.turn_check_cali(Dir='forward', steps=totalDist, limit_switch=outer_switch, stepdelay=0.0002)

        if test_inner:
            calibrated = True
            print("Calibration Passed!")
            # sleep(.5)
            # M_Lin.turn_steps(Dir='backward', steps=totalDist, stepdelay=0.0002)
        else:
            print("Calibration Failed! Trying again...")

    return totalDist


def erase_out_to_in():
    M_Rot.running = True
    M_Lin.running = True

    sleep(1)
    M_Lin.turn_until_switch(Dir='forward', limit_switch=outer_switch, stepdelay=0.0002)
    M_Lin.turn_steps(Dir='backward', steps=outer_to_max, stepdelay=0.0002)
    print("Found edge")

    sleep(.5)
    MRot = threading.Thread(target=run_MRot_until, args=('forward', 0.0005,))
    MLin = threading.Thread(target=run_MLin_until, args=(-max_disp, 0.01,))

    print("Erasing...")
    MRot.start()
    MLin.start()

    MRot.join()
    MLin.join()


def erase_in_to_out():
    sleep(1)
    M_Lin.turn_until_switch(Dir='backward', limit_switch=inner_switch, stepdelay=0.0002)
    M_Lin.turn_steps(Dir='forward', steps=center_to_min, stepdelay=0.0002)
    print("Found center")

    sleep(.5)
    MRot = threading.Thread(target=run_MRot_until, args=('forward', 0.00035,))
    MLin = threading.Thread(target=run_MLin_until, args=(max_disp, 0.01,))

    print("Erasing...")
    MRot.start()
    MLin.start()

    MRot.join()
    MLin.join()


def ask_for_erase():
    erase_timeout = 3 * 10
    interface.ask_erase = True
    yes_or_no = False

    while interface.ask_erase:
        if erase_timeout < 1:
            interface.ask_erase = False
            continue

        sleep(.1)
        erase_timeout -= 1

        if interface.paused:
            yes_or_no = True

    return yes_or_no

set_color = Color(255, 255, 255)
pause_color = Color(204, 0, 0)
accept_color = Color(0, 255, 42)

class InterfaceThread():

    def __init__(self):
        self.timeout_length = 5000

        # self.running = True
        self.button_pressed = False
        self.button_press_start_time = None

        self.paused = False
        self.pause_start_time = None

        self.stop_program = False
        self.next_drawing = False
        self.erase = False

        # add press event for push button
        GPIO.add_event_detect(main_button, GPIO.RISING,
                              callback=self.main_button_pressed, bouncetime=200)

    def main_button_pressed(self):
        if not self.button_pressed:
            self.paused = False
            self.button_pressed = True
            self.button_press_start_time = int(round(time.time() * 1000))
            #red color?
            strip_thread.colorWipe(strip, pause_color, 5)
            return

        if self.button_pressed:
            if self.paused:
                    self.next_drawing = True
                    strip_thread.colorWipe(strip, accept_color, 5)
            if not self.paused:
                self.paused = True

        

    def check_loop(self):
        while self.running:
            sleep(.1)

            if self.button_press_start_time and int(round(time.time() * 1000)) - self.button_press_start_time > self.timeout_length:
                    self.button_pressed = False
                    self.button_press_start_time = None
                    strip_thread.colorWipe(strip, set_color, 5)


# Stops the motors and LED strip, and joins the threads
def stop_program(shutdown=False):
    stop_motors()

    strip_thread.running = False
    strip_thread.colorWipe(strip, Color(0, 0, 0), 10)
    LStrip.join()

    interface.running = False
    interface_thread.join()

    if shutdown:
        sleep(2)

        GPIO.cleanup()

        call("sudo shutdown -h now", shell=True)
    else:
        GPIO.cleanup()
        print("Exiting...")
        exit()


def stop_motors():
    M_Rot.running = False
    M_Lin.running = False
    M_Rot.stop()
    M_Lin.stop()
    print("\n---------- Motors Stopped! ----------")


# button thread object init
interface = InterfaceThread()

# Create button thread
interface_thread = threading.Thread(target=interface.check_loop)

# Create LStrip thread
LStrip = threading.Thread(target=run_LedStrip)

def main():
    global max_disp

    try:
        max_disp = calibrate_slide()

        # GPIO.output(motor_relay, GPIO.LOW)
        # GPIO.output(led_relay, GPIO.LOW)

        LStrip.start()

        # process_new_files(Dir="/home/pi/code/sand_table/")
        # files = get_files(Dir="/home/pi/code/sand_table/")
        with open("/home/pi/code/sand_table/filenames.txt", "r") as f:
            content = f.readlines()
        files = [line.rstrip('\n') for line in content]
        shuffle(files)

        print("---files---")
        print(files)

        interface_thread.start()

        if len(files) == 0:
            print("---Files not found!---")

        start_first_file = False
        # first_file = not ask_for_erase()

        while not interface.stop_program:
            for f in files:
                if interface.stop_program or interface.paused:
                    break

                if not start_first_file:
                    erase_out_to_in()

                    if interface.stop_program:
                        break

                    start_first_file = True

                print("Running: {}".format(f))

                track = read_track(f, Dir="/home/pi/code/sand_table/")

                for i, step in enumerate(track):
                    print(step)
                    # lcd_display.lcd_display_string(
                    #     "{}/{}".format(i+1, track.shape[0]), 4, 10)

                    # Create motor threads
                    MRot = threading.Thread(target=run_MRot, args=(step[0], step[2],))
                    MLin = threading.Thread(target=run_MLin, args=(step[1], step[3],))

                    print("...")
                    M_Rot.running = True
                    M_Lin.running = True
                    MRot.start()
                    MLin.start()

                    MRot.join()
                    MLin.join()

                    if interface.stop_program or interface.next_drawing:
                        break
                    if interface.paused:
                        while interface.paused:
                            sleep(.1)

                    print("Motors done!")

        if interface.stop_program:
            stop_program(shutdown=True)

    except KeyboardInterrupt:
        stop_program()


if __name__ == '__main__':
    main()
