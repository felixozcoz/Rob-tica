#!/usr/bin/python
# -*- coding: UTF-8 -*-
from Robot import Robot

def main():

    #red_min  = [(160, 100,  20),(  0, 100, 250)]
    #red_max  = [(179, 255, 255),( 10, 255, 255)]
    # red_min  = [(71, 30, 100)]
    # red_max  = [(71, 31, 133)]
    #blue_min = [(110,  50,  50),(120, 213,  60)]
    #blue_max = [(130, 255, 255),(120, 155, 255)]

    # Negative red mask (blue mask)
    colorRangeMin = (80, 70, 50)
    colorRangeMax = (100, 255, 255)
    
    try:
        # if args.radioD < 0:
        #     print('d must be a positive value')
        #     exit(1)

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot() 

        # 1. launch updateOdometry thread()
        #robot.startOdometry()

        # 2. Loop running the tracking until ??, then catch the ball
        print("Vamos a jugah")
        robot.trackObject(colorRangeMin=colorRangeMin, colorRangeMax=colorRangeMax)

        # 3. Wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors, 
        # and restore the LED to the control of the BrickPi3 firmware.
        #robot.stopOdometry()

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


