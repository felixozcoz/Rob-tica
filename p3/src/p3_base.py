#!/usr/bin/python
# -*- coding: UTF-8 -*-
from Robot import Robot

def main():

    red_min  = [(160, 100,  20),(  0, 100, 250)]
    red_max  = [(179, 255, 255),( 10, 255, 255)]
    blue_min = [(110,  50,  50),(120, 213,  60)]
    blue_max = [(130, 255, 255),(120, 155, 255)]

    try:
        # if args.radioD < 0:
        #     print('d must be a positive value')
        #     exit(1)

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot() 

        # 1. launch updateOdometry thread()
        robot.startOdometry()

        # 2. Loop running the tracking until ??, then catch the ball
        # TO-DO: ADD to the Robot class a method to track an object, given certain parameters
        # for example the different target properties we want (size, position, color, ..)
        # or a boolean to indicate if we want the robot to catch the object or not
        # At least COLOR, the rest are up to you, but always put a default value.
    	# res = robot.trackObject(colorRangeMin=[0,0,0], colorRangeMax=[255,255,255], 
        #                   targetSize=??, target??=??, ...)
        print("Vamos a seguir la pelota")
        if robot.trackObject(colorRangeMin=red_min, colorRangeMax=red_max, showFrame=False):
            robot.catch()
        elif robot.trackObject(colorRangeMin=blue_min, colorRangeMax=blue_max, showFrame=False):
            robot.catch()

        # 3. Wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors, 
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()
        print('*** Ctrl-C detected - Finishing ...')

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-c", "--color", help="color of the ball to track",
    #                     type=float, default=40.0)
    # args = parser.parse_args()
    main()


