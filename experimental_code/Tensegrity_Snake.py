#This code was the basic sample of using an Xbox controller to control 
# movement of a robot. 
# 
from __future__ import print_function
import xbox
import RPi.GPIO as GPIO
import time

movement_state = None  # None, "compress", or "decompress"
movement_end_time = 0
# GPIO setup
GPIO.setmode(GPIO.BCM)

# Use GPIO to setup PWM output for motors
motors_pwm = {
    "motor1": 12,  # PWM pin for motor 1 Left
    "motor2": 13,  # PWM pin for motor 2 Left
    "motor3": 25,  # PWM pin for motor 3 Left
    "motor4": 10,  # PWM pin for motor 4 Right
    "motor5": 17,  # PWM pin for motor 5 Right
    "motor6": 14   # PWM pin for motor 6 Right
}

for pwm_pin in motors_pwm.values():
    GPIO.setup(pwm_pin, GPIO.OUT)  # Set pin as output
motors_pwm_objs = {name: GPIO.PWM(pin, 1000) for name, pin in motors_pwm.items()}  # Initialize PWM
for pwm in motors_pwm_objs.values():
    pwm.start(0)  # Start PWM with value of 0

# Motor direction pins setup using RPi.GPIO
motors_digital = {
    "ain1": 20, "ain2": 21,  # Motor 1 direction control
    "bin1": 26, "bin2": 19,  # Motor 2 direction control
    "cin1": 7, "cin2": 8,    # Motor 3 direction control
    "din1": 11, "din2": 9,   # Motor 4 direction control
    "ein1": 27, "ein2": 22,  # Motor 5 direction control
    "fin1": 15, "fin2": 18   # Motor 6 direction control
}

for motor_pin in motors_digital.values():
    GPIO.setup(motor_pin, GPIO.OUT)  # Set pin as output
def stop_all_motors():
    # stop motor 1
    set_motor_direction("ain","stop")
    set_pwm(motors_pwm_objs, "motor1",0)
    # stop motor 2
    set_motor_direction("bin","stop")
    set_pwm(motors_pwm_objs, "motor2",0)
    # stop motor 3
    set_motor_direction("cin","stop")
    set_pwm(motors_pwm_objs, "motor3",0)
    # stop motor 4
    set_motor_direction("din","stop")
    set_pwm(motors_pwm_objs, "motor4",0)
    # stop motor 5
    set_motor_direction("ein","stop")
    set_pwm(motors_pwm_objs, "motor5",0)
    # stop motor 6
    set_motor_direction("fin","stop")
    set_pwm(motors_pwm_objs, "motor6",0)
   
# Helper function to set motor direction
def set_motor_direction(motor_name, direction):
    if direction == "forward":
        GPIO.output(motors_digital[f"{motor_name}1"], True)
        GPIO.output(motors_digital[f"{motor_name}2"], False)
    elif direction == "backward":
        GPIO.output(motors_digital[f"{motor_name}1"], False)
        GPIO.output(motors_digital[f"{motor_name}2"], True)
    else:  # Stop
        GPIO.output(motors_digital[f"{motor_name}1"], False)
        GPIO.output(motors_digital[f"{motor_name}2"], False)

# Format floating point number to string format -x.xxx
def fmtFloat(n):
    return '{:6.3f}'.format(n)

# Print one or more values without a line feed
def show(*args):
    for arg in args:
        print(arg, end="")

# Print true or false value based on a boolean, without linefeed
def showIf(boolean, ifTrue, ifFalse=" "):
    if boolean:
        show(ifTrue)
    else:
        show(ifFalse)

##### Instantiate the controller, Important
joy = xbox.Joystick()

# Helper function to set PWM duty cycle
def set_pwm(pwm_objs, motor_name, duty_cycle):
    pwm_objs[motor_name].ChangeDutyCycle(max(0, min(100, duty_cycle)))  # Ensure 0 <= duty_cycle <= 100

# The basic idea for this is to have the front be controlled by the left joy stick 
# and the rear to be controlled by the right joy stick

# Control logic for front and rear ends
def control_front(joy_x, joy_y):
    # Control motor1 (side to side, left-right motion via x-axis)
    if joy_x > 0:  # Move right
        set_motor_direction("ain", "forward")
        set_pwm(motors_pwm_objs, "motor1", joy_x * 100)
    elif joy_x < 0:  # Move left
        set_motor_direction("ain", "backward")
        set_pwm(motors_pwm_objs, "motor1", -joy_x * 100)
    else:  # Stop motor1
        set_motor_direction("ain", "stop")
        set_pwm(motors_pwm_objs, "motor1", 0)

    # Control motor2 (diagonal motion, combine x and y axes)
    if joy_x != 0 or joy_y != 0:
        direction = "forward" if (joy_x + joy_y) > 0 else "backward"
        duty_cycle = abs(joy_x + joy_y) * 50  # Scale combined value to duty cycle
        set_motor_direction("bin", direction)
        set_pwm(motors_pwm_objs, "motor2", min(100, duty_cycle))  # Limit max to 100%
    else:  # Stop motor2
        set_motor_direction("bin", "stop")
        set_pwm(motors_pwm_objs, "motor2", 0)

    # Control motor3 (up and down motion via y-axis)
    if joy_y > 0:  # Move up
        set_motor_direction("cin", "forward")
        set_pwm(motors_pwm_objs, "motor3", joy_y * 100)
    elif joy_y < 0:  # Move down
        set_motor_direction("cin", "backward")
        set_pwm(motors_pwm_objs, "motor3", -joy_y * 100)
    else:  # Stop motor3
        set_motor_direction("cin", "stop")
        set_pwm(motors_pwm_objs, "motor3", 0)

def control_rear(joy_x, joy_y):
    # Control motor4 (up and down motion via y-axis)
    if joy_y > 0:  # Move forward
        set_motor_direction("din", "forward")
        set_pwm(motors_pwm_objs, "motor4", joy_y * 100)
    elif joy_y < 0:  # Move backward
        set_motor_direction("din", "backward")
        set_pwm(motors_pwm_objs, "motor4", -joy_y * 100)
    else:  # Stop motor4
        set_motor_direction("din", "stop")
        set_pwm(motors_pwm_objs, "motor4", 0)

    # Control motor5 (side-to-side motion via x-axis)
    if joy_x > 0:  # Move right
        set_motor_direction("ein", "forward")
        set_pwm(motors_pwm_objs, "motor5", joy_x * 100)
    elif joy_x < 0:  # Move left
        set_motor_direction("ein", "backward")
        set_pwm(motors_pwm_objs, "motor5", -joy_x * 100)
    else:  # Stop motor5
        set_motor_direction("ein", "stop")
        set_pwm(motors_pwm_objs, "motor5", 0)

    # Control motor6 (diagonal motion, combine x and y axes)
    if joy_x != 0 or joy_y != 0:
        direction = "forward" if (joy_x + joy_y) > 0 else "backward"
        duty_cycle = abs(joy_x + joy_y) * 50  # Scale combined value to duty cycle
        set_motor_direction("fin", direction)
        set_pwm(motors_pwm_objs, "motor6", min(100, duty_cycle))  # Limit max to 100%
    else:  # Stop motor6
        set_motor_direction("fin", "stop")
        set_pwm(motors_pwm_objs, "motor6", 0)

# This is a preset motion for complete compression and deployment
def move_all_motors(joy_x, joy_y):
    if joy.dpadLeft():  # Compression
        compress_all_motors()
    elif joy.dpadDown():  # Decompression
        decompress_all_motors()
    else:
            # Front end control with left joystick
        control_front(joy.leftX(), joy.leftY())
            # Rear end control with right joystick
        control_rear(joy.rightX(), joy.rightY())
def compress_all_motors():
    # Motor 1: Compress (move forward at 100% speed)
    set_motor_direction("ain", "forward")
    set_pwm(motors_pwm_objs, "motor1", 100)

    # Motor 2: Compress (move backward at 100% speed)
    set_motor_direction("bin", "backward")
    set_pwm(motors_pwm_objs, "motor2", 100)

    # Motor 3: Compress (move forward at 100% speed)
    set_motor_direction("cin", "forward")
    set_pwm(motors_pwm_objs, "motor3", 100)

    # Compress motor 4
    set_motor_direction("din", "forward")
    set_pwm(motors_pwm_objs, "motor4", 100)

    # Compress motor 5
    set_motor_direction("ein", "forward")
    set_pwm(motors_pwm_objs, "motor5", 100)

    # Compress motor 6
    set_motor_direction("fin", "forward")
    set_pwm(motors_pwm_objs, "motor6", 100)

def decompress_all_motors():
    # Motor 1: Decompress (move backward at 100% speed)
    set_motor_direction("ain", "backward")
    set_pwm(motors_pwm_objs, "motor1", 100)

    # Motor 2: Decompress (move forward at 100% speed)
    set_motor_direction("bin", "forward")
    set_pwm(motors_pwm_objs, "motor2", 100)

    # Motor 3: Decompress (move backward at 100% speed)
    set_motor_direction("cin", "backward")
    set_pwm(motors_pwm_objs, "motor3", 100)

    # Decompress motor 4
    set_motor_direction("din", "backward")
    set_pwm(motors_pwm_objs, "motor4", 100)

    # Decompress motor 5
    set_motor_direction("ein", "backward")
    set_pwm(motors_pwm_objs, "motor5", 100)

    # Decompress motor 6
    set_motor_direction("fin", "backward")
    set_pwm(motors_pwm_objs, "motor6", 100)

# This section is from the sample, and i just never removed it since it shows what is being pressed on the controller

try:
    # Show various axis and button states until Back button is pressed
    while not joy.Back():
        # Show connection status
        show("Connected:")
        showIf(joy.connected(), "Y", "N")
        # Left analog stick
        show("  Left X/Y:", fmtFloat(joy.leftX()), "/", fmtFloat(joy.leftY()))
        show("  Right X/Y:", fmtFloat(joy.rightX()), "/", fmtFloat(joy.rightY()))
        # Right trigger
        show("  RightTrg:", fmtFloat(joy.rightTrigger()))
        # A/B/X/Y buttons
        show("  Buttons:")
        showIf(joy.A(), "A")
        showIf(joy.B(), "B")
        showIf(joy.X(), "X")
        showIf(joy.Y(), "Y")
        # Dpad U/D/L/R
        show("  Dpad:")
        showIf(joy.dpadUp(),    "U")
        showIf(joy.dpadDown(),  "D")
        showIf(joy.dpadLeft(),  "L")
        showIf(joy.dpadRight(), "R")
        # Move cursor back to start of line
        show(chr(13))
        if joy.dpadLeft():
            compress_all_motors()
        elif joy.dpadDown() and movement_state is None:
            decompress_all_motors()
            movement_end_time = time.time() + 2  # Run motors for 2 seconds
            decompress_all_motors()
        else:
            # Front end control with left joystick
            control_front(joy.leftX(), joy.leftY())

            # Rear end control with right joystick
            control_rear(joy.rightX(), joy.rightY())
       

        ####### This does not work, I fixed it with the d pad movemenr above
        # Button A: Spin all motors forward
        if joy.A():
            for motor in ["ain", "bin", "cin", "din", "ein", "fin"]:
                set_motor_direction(motor, "forward")
                set_pwm(motors_pwm_objs, f"motor{int(motor[-1])}", 100)

        # Button B: Spin all motors backward
        if joy.B():
            for motor in ["ain", "bin", "cin", "din", "ein", "fin"]:
                set_motor_direction(motor, "backward")
                set_pwm(motors_pwm_objs, f"motor{int(motor[-1])}", 100)
    # Close out when done
finally:
    joy.close()
    for pwm in motors_pwm_objs.values():
        pwm.stop()
    GPIO.cleanup()
