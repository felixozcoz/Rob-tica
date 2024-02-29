import brickpi3
import time
BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder B
BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder C
while True:
    sA = BP.get_motor_encoder(BP.PORT_A)
    sD = BP.get_motor_encoder(BP.PORT_D)
    print(sA)
    print(sD)
    print("\n")
    time.sleep(2.00)

