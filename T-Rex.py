#!/usr/bin/env python3

#################################################################################################
# T-Rex.py                                                                                     #
# Version 1.0                                                                                   #
#                                                                                               #
# Happily shared under the MIT License (MIT)                                                    #
#                                                                                               #
# Copyright(c) 2017 SmallRobots.it                                                              #
#                                                                                               #
# Permission is hereby granted, free of charge, to any person obtaining                         #
# a copy of this software and associated documentation files (the "Software"),                  #
# to deal in the Software without restriction, including without limitation the rights          #
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies              #
# of the Software, and to permit persons to whom the Software is furnished to do so,            #
# subject to the following conditions:                                                          #
#                                                                                               #
# The above copyright notice and this permission notice shall be included in all                #
# copies or substantial portions of the Software.                                               #
#                                                                                               #
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,           #
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR      #
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE            #
# LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,           #
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE         #
# OR OTHER DEALINGS IN THE SOFTWARE.                                                            #
#                                                                                               #
# Visit http://www.smallrobots.it for tutorials and videos                                     #
#                                                                                               #
# Credits                                                                                       #
# The T-R3X is built with Lego Mindstorms Ev3 retail set                                        #
# Building instructions can be found on                                                         #
# "The Lego Mindstorms EV3 Laboratory" written by Daniele Benedettelli                          #
#################################################################################################


from _thread import *
from time import sleep
from ev3dev.auto import *


class TRex(object):

    # Default constructor
    def __init__(self):
        # Motors init
        self.mouthMotor = MediumMotor("outC")
        self.legsMotor = LargeMotor("outA")
        self.bodyMotor = LargeMotor("outD")
        self.mouthMotor.position = 0
        self.legsMotor.position = 0
        self.bodyMotor.position = 0

        # Sensors init
        self.irSensor = InfraredSensor()
        self.irSensor.mode = "IR-SEEK"

        # Constants
        self.bodySpeed = 80
        self.bodyAngle = 65
        self.stepSpeed = 150
        self.stepAngle = 95
        self.timeOut = 1000
        self.maxMouthOpening = 60
        self.mouthOpeningSpeed = 50

        # Fields
        self.beaconFollowing = False

    # Makes the T-Rex roar
    def roar(self):
        # Roars
        Sound.play('lion214.wav')
        self.mouthMotor.run_to_rel_pos(position_sp=self.maxMouthOpening, speed_sp=self.mouthOpeningSpeed,
                                       stop_action="hold")
        self.mouthMotor.wait_while('running')
        self.mouthMotor.run_to_rel_pos(position_sp=-self.maxMouthOpening, speed_sp=self.mouthOpeningSpeed,
                                       stop_action="hold")
        self.mouthMotor.wait_while('running')

    # Reset the body motor
    def reset_body_motor(self):
        # Body motor
        print("\nResetting Body Motor")

        print("- stop Motor")
        self.bodyMotor.stop(stopaction="coast")
        print("- Stopped")

        print("- Turn motor until stalled")
        self.bodyMotor.run_forever(speed_sp=self.bodySpeed)
        self.bodyMotor.wait_until('stalled')
        self.bodyMotor.stop(stop_action="hold")
        print("- Motor stalled")

        print("- Moving motor to zero position")
        self.bodyMotor.run_to_rel_pos(position_sp=-self.bodyAngle, speed_sp=self.bodySpeed, stop_action="hold")
        self.bodyMotor.wait_while('running', timeout=self.timeOut)
        sleep(1)
        print("- Motor reached zero position")
        self.bodyMotor.position = 0

        print("- Body Motor resetted")

    # Reset the legs actuators to zero
    def reset_legs_motor(self):
        # Legs motor
        print("\nResetting Legs Motor")

        print("- stop Motor")
        self.legsMotor.stop(stopaction="coast")
        print("- Stopped")

        print("- Moving body to the left")
        self.bodyMotor.run_to_abs_pos(position_sp=self.bodyAngle, speed_sp=self.bodySpeed, stop_action="hold")
        self.bodyMotor.wait_while('running', timeout=self.timeOut)
        print("- Turn legs motor until stalled")
        self.legsMotor.run_forever(speed_sp=self.stepSpeed)
        self.legsMotor.wait_until('stalled')
        self.legsMotor.stop(stop_action="hold")
        print("- Motor stalled")

        print("- Moving motor to zero position")
        self.legsMotor.run_to_rel_pos(position_sp=-self.stepAngle, speed_sp=self.stepSpeed, stop_action="hold")
        self.legsMotor.wait_while('running', timeout=self.timeOut)
        self.legsMotor.stop(stop_action="coast")
        print("- Motor reached zero position")
        self.legsMotor.position = 0
        print("- Moving body motor to zero position")
        self.bodyMotor.run_to_abs_pos(position_sp=0, speed_sp=self.bodySpeed, stop_action="hold")
        self.bodyMotor.wait_while('running', timeout=self.timeOut)
        print("- legsMotor Motor resetted")

    # One step forward
    def one_step_forward(self):
        print("\nOne Step")
        # Move body to the left
        print("- Move body to the left")
        self.bodyMotor.run_to_abs_pos(position_sp=+self.bodyAngle, speed_sp=self.bodySpeed, stop_action="hold")
        self.bodyMotor.wait_while('running', timeout=self.timeOut)
        sleep(0.2)

        # Move right leg forward
        print("- Move right leg forward")
        self.legsMotor.run_to_abs_pos(position_sp=-self.stepAngle, speed_sp=self.stepSpeed, stop_action="hold")
        self.legsMotor.wait_while('running', timeout=self.timeOut)

        # Move body to the right
        print("- Move body to the right")
        self.bodyMotor.run_to_abs_pos(position_sp=-self.bodyAngle, speed_sp=self.bodySpeed, stop_action="hold")
        self.bodyMotor.wait_while('running', timeout=self.timeOut)
        sleep(0.2)

        # Move left leg forward
        print("- Move left leg forward")
        self.legsMotor.run_to_abs_pos(position_sp=self.stepAngle, speed_sp=self.stepSpeed, stop_action="hold")
        self.legsMotor.wait_while('running', timeout=self.timeOut)

    # Stops the robot
    def stop(self):
        self.mouthMotor.stop(stop_action='coast')
        self.bodyMotor.stop(stop_action='coast')
        self.legsMotor.stop(stop_action='coast')

    # Blink all leds Red
    def blink_all_leds_red(self):
        Leds.set(Leds.LEFT, brightness_pct=0.5, trigger='timer')
        Leds.set(Leds.RIGHT, brightness_pct=0.5, trigger='timer')
        Leds.set(Leds.LEFT, delay_on=500, delay_off=500)
        Leds.set(Leds.RIGHT, delay_on=500, delay_off=500)
        Leds.set_color(Leds.LEFT, Leds.RED)
        Leds.set_color(Leds.RIGHT, Leds.RED)

    # Turns the robot a specified angle
    def turn(self, angle):
        print("\n Turn %s degrees" % angle)
        time_to_wait = 0.1
        if angle > 0:
            print("- Angle is positive")
            print("- Move body to the right")
            self.bodyMotor.run_to_abs_pos(position_sp=-self.bodyAngle, speed_sp=self.bodySpeed, stop_action="hold")
            self.bodyMotor.wait_while('running', timeout=self.timeOut)
            sleep(time_to_wait)
            print("- Move left leg forward")
            self.legsMotor.run_to_abs_pos(position_sp=self.stepAngle, speed_sp=self.stepSpeed, stop_action="hold")
            self.legsMotor.wait_while('running', timeout=self.timeOut)
            print("- Move body to the specified angle")
            self.bodyMotor.run_to_abs_pos(position_sp=angle, speed_sp=self.bodySpeed, stop_action="hold")
            self.bodyMotor.wait_while('running', timeout=self.timeOut)
            sleep(time_to_wait)
            print("- Calling back the body")
            self.bodyMotor.run_to_abs_pos(position_sp=0, speed_sp=900, stop_action="hold")
            self.bodyMotor.wait_while('running', timeout=self.timeOut)
            sleep(time_to_wait)
        else:
            print("- Angle is negative")
            print("- Move body to the left")
            self.bodyMotor.run_to_abs_pos(position_sp=self.bodyAngle, speed_sp=self.bodySpeed, stop_action="hold")
            self.bodyMotor.wait_while('running', timeout=self.timeOut)
            sleep(time_to_wait)
            print("- Move right leg forward")
            self.legsMotor.run_to_abs_pos(position_sp=-self.stepAngle, speed_sp=self.stepSpeed, stop_action="hold")
            self.legsMotor.wait_while('running', timeout=self.timeOut)
            print("- Move body to the specified angle")
            self.bodyMotor.run_to_abs_pos(position_sp=angle, speed_sp=self.bodySpeed, stop_action="hold")
            self.bodyMotor.wait_while('running', timeout=self.timeOut)
            sleep(time_to_wait)
            print("- Calling back the body")
            self.bodyMotor.run_to_abs_pos(position_sp=0, speed_sp=900, stop_action="hold")
            self.bodyMotor.wait_while('running', timeout=self.timeOut)
            sleep(time_to_wait)

    # Turns right
    def turn_right(self):
        self.turn(-40)
        sleep(0.5)

    # Turns left
    def turn_left(self):
        self.turn(75)
        sleep(0.5)

    # Starts the beacon following thread
    def start_beacon_following(self):
        self.beaconFollowing = True
        start_new_thread(self.follow_beacon, ())

    # Stops the beacon following thread
    def stop_beacon_following(self):
        self.beaconFollowing = False

    # Follows the beacon
    def follow_beacon(self):
        while self.beaconFollowing:
            direction = self.irSensor.value()
            if abs(direction) < 10:
                self.one_step_forward()
            elif direction > 0:
                self.turn_right()
            else:
                self.turn_left()


# TRex initialization
robot = TRex()
robot.reset_body_motor()
robot.reset_legs_motor()

# A good roar
robot.roar()

robot.start_beacon_following()
fine = False
while not fine:
    userInput = input("Press 'S' to stop >")
    if userInput == "S" or userInput == "s":
        fine = True
robot.stop_beacon_following()
