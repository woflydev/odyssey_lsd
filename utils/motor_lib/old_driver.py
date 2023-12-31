# from board import SCL, SDA
# import busio
# from adafruit_pca9685 import PCA9685
import signal
from time import sleep
import math
from pyfirmata import Arduino, util, pyfirmata
import sys 

PWM_FREQ = 3906  # (Hz) max is 1.5 kHz
MAP_CONST = 1 / 120   # 1 / 120 to limit speed below 100% duty cycle
HALF_WIDTH = 0.1          # Half of the width of droid, in metres
MAX_CENT_ACC = 30000   # Maximum "centripetal acceleration" the robot is allowed to undergo. UNITS ARE DODGY, MUST BE DETERMIEND BY EXPERIMENTATION
MAX_SPEED = MAP_CONST * 100  # (percent) max speed of motors
SERIAL_PORT = '/dev/ttyACM0' # Serial port for Arduino

# Left
#     A -> 13  
#     B -> 12
#   PWM -> 11

# Right
#     A -> 8
#     B -> 9
#   PWM -> 10

# I_Sense -> 7

def exit_handler(signal, frame):
    print("\n...\nStopping motors...")
    off()
    print("Cleaning up...")  
    print("Done.")  
    board.exit()
    sys.exit(0)

signal.signal(signal.SIGINT, exit_handler)

board = Arduino(SERIAL_PORT)
print("Communication Successfully started")

motorLA = board.get_pin('d:13:o')
motorLB = board.get_pin('d:12:o')
motorRA = board.get_pin('d:8:o')
motorRB = board.get_pin('d:9:o')

motorPWMLA = board.get_pin('d:11:p')
motorPWMLB = board.get_pin('d:6:p')
motorPWMRA = board.get_pin('d:10:p')
motorPWMRB = board.get_pin('d:5:p')

Isense = board.get_pin('d:7:i')

print("Motor driver initialized. \n path: utils\motor_lib\driver.py \n PWM frequency: " + str(PWM_FREQ) + "Hz \n Max speed: " + str(MAX_SPEED) + "%")

# Path: utils\motor_lib\driver.py
# off/coast/stop are the same
def off():
    # Enable pins are low during off() to coast
    motorPWMLA.write(0)
    motorPWMLB.write(0)
    motorPWMRA.write(0)
    motorPWMRB.write(0)

    motorLA.write(0)
    motorLB.write(0)
    motorRA.write(0)
    motorRB.write(0)

def stop():
    off()
    
def coast():
    off()

def brake():
    off()
    motorLA.write(1)
    motorLB.write(1)
    motorRA.write(1)
    motorRB.write(1)
    # Enable pins are high during brake() to brake
    motorPWMLA.write(0)
    motorPWMRA.write(0)

# brakes after 1.5s of coasting
def ebrake():
    off()
    sleep(1.5) # sleep(1.5)
    motorLA.write(1)
    motorLB.write(1)
    motorRA.write(1)
    motorRB.write(1)

# forward function
def fwd(speed, timeout=0):
    motorPWMLA.write(speed * MAP_CONST)
    motorPWMRA.write(speed * MAP_CONST)

    motorLA.write(1)
    motorLB.write(0)
    motorRA.write(1)
    motorRB.write(0)

    if timeout > 0:
        sleep(timeout / 1000)
        off()

# reverse function
def rev(speed, timeout=0):
    motorPWMLB.write(speed * MAP_CONST)
    motorPWMRB.write(speed * MAP_CONST)

    motorLA.write(0)
    motorLB.write(1)
    motorRA.write(0)
    motorRB.write(1)

    if timeout > 0:
        sleep(timeout / 1000)
        off()

# Write motor values for a turn, where a positive radius denotes a right turn (think +x), and negatvie radius defines left turn
def turn(speed: float, radius: float, timeout=0):
    r = abs(radius)
    if(speed < 0 or speed > 100):
        raise Exception(f"[MOTOR]: Invalid turn speed {speed}")
    if( r == 0 or speed * speed / r > MAX_CENT_ACC):
        print("[MOTOR]: Ignored attempt to turn at speed {speed} and radius {r} due to potential slipping.")
        return # Should I raise an exception instead?
    omega = speed / r
    if(radius > 0):
        move(omega * (r + HALF_WIDTH), omega * (r - HALF_WIDTH), timeout)
    elif(radius == 0):
        move(omega * (r - HALF_WIDTH), omega * (r + HALF_WIDTH), timeout)

# # input -100 to 100 left and right sides
def move(RIN, LIN, timeout=0):
    LIN = round(LIN / 5) * 5
    RIN = round(RIN / 5) * 5
    L = LIN * MAP_CONST  # map values to 0-1
    R = RIN * MAP_CONST
    #print(L, R)
    if L == 0 and R == 0:
        off()
        brake()
    else:
        #print(L, R)
        if L > 0:
            motorPWMLA.write(abs(L))
            motorPWMLB.write(0)
        else:
            motorPWMLA.write(0)
            motorPWMLB.write(abs(L))
        if R > 0:
            motorPWMRA.write(abs(R))
            motorPWMRB.write(0)
        else:
            motorPWMRA.write(0)
            motorPWMRB.write(abs(R))
            
        motorLA.write(1)
        motorLB.write(1)
        motorRA.write(1)
        motorRB.write(1)
    if timeout > 0:
        sleep(timeout / 1000)
        off()

# # input -100 to 100 left and right sides
# def move(LIN, RIN, timeout=0):
#     LIN = round(LIN / 5) * 5
#     RIN = round(RIN / 5) * 5
#     L = LIN * MAP_CONST  # map values to 0-1
#     R = RIN * MAP_CONST
#     #print(L, R)
#     if L == 0 and R == 0:
#         off()
#         brake()
#     else:
#         #print(L, R)
#         if L > 0:
#             motorPWML.write(abs(L))
#         else:
#             motorPWML.write(0)
#         if R > 0:
#             motorPWMR.write(abs(R))
#         else:
#             motorPWMR.write(0)

#     if timeout > 0:
#         sleep(timeout / 1000)
#         off()

def readCurrent():
    return Isense.read()

# Drive pins other than motor pins
# def drivePin(pin, val):
#     if pin == 0 or pin == 1 or pin == 2 or pin == 3:
#         raise Exception(f"Pin {pin} is used for motors.")
#     else:
#         pca.channels[pin].duty_cycle = int(val / 100 * 65535)
#         print(f"Pin {pin} set to {val}%")

