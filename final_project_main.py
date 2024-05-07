# *****************************************************************************
# ***************************  Python Source Code  ****************************
# *****************************************************************************
#
#   DESIGNER NAME:  Dakota Maxwell & Isaac Cassady
#
#       FILE NAME:  final_project_main.py
#
# DESCRIPTION
#   INSERT HERE
#
# *****************************************************************************
#*****************************************************************************
# Imports to be used in program
#*****************************************************************************
import RPi.GPIO as GPIO
import time
import LCD1602
from bottle import route, run, template
import Tkinter as TK


#*****************************************************************************
# Constants to be used in program
#*****************************************************************************

# cm
OBST_MAX_DIST           = 11.43
FORWARD                 = 1
BACKWARDS               = -1
STOP                    = 0
LEFT_WHEEL_ENABLE_GPIO  = 22
RIGHT_WHEEL_ENABLE_GPIO = 25
LEFT_WHEEL_1_GPIO       = 27
LEFT_WHEEL_2_GPIO       = 17
RIGHT_WHEEL_1_GPIO      = 24
RIGHT_WHEEL_2_GPIO      = 23
LEFT_OBST_GPIO          = ?
RIGHT_OBST_GPIO         = ?
LED_GPIO                = ?

TRIG_GPIO    = ?
ECHO_GPIO    = ?
MAX_DISTANCE = 220
TIME_OUT     = MAX_DISTANCE * 60
NANO_SEC_10    = 0.000001
REVERT_DIVIDE  = 1000000
SPEED_OF_SOUND = 340.00
REFLECT_NEGATE = 2.0
DIVIDE_BY_TIME = 10000.00

# Constants for PWM functions
START_DUTY_CYCLE = 0
STOP_DUTY_CYCLE  = 100
DUTY_CYCLE_DELAY = 0.01
TURNAROUND_TIME  = 0.5
PWM_FREQUENCY    = 1000

# Constants for GUI Functions
WINDOW_SIZE     = "400x175"
X_OFFSET        = "+100" 
Y_OFFSET        = "+100"
WINDOW_OFFSET   = X_OFFSET + Y_OFFSET
WINDOW_GEOMETRY = WINDOW_SIZE + WINDOW_OFFSET

# Delays with varying lengths
READ_DELAY_10NS  = 0.00001
READ_DELAY_10MS  = 0.01
READ_DELAY_200MS = 0.2
READ_DELAY_500MS = 0.5
READ_DELAY_1SEC  = 1

# Increments/decrement
INCREMENT = 1
DECREMENT = -1

# sec
MIN_PULSE_WIDTH = 0.0005
MAX_PULSE_WIDTH = 0.0025

# sec
PIG_MIN_PULSE_WIDTH = 500
PIG_MAX_PULSE_WIDTH = 2500

# angle
MIN_ANGLE = -90
MAX_ANGLE = 90
ANGLE_OFFSET = 90

# angle
MIN_ROTATION = 0
MAX_ROTATION = 180

PERIOD_LENGTH = 0.02

CONVERT_TO_PERCENT = 100

SECONDS_PER_DEGREE = (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / MAX_ROTATION

PERIOD = 1 / PWM_FREQUENCY

ON_TIME_BUFFER = 1E6

SERVER_HOST = '0.0.0.0'
SERVER_PORT = 80

START_INDEX = 0

#---------------------------------------------------------------------
# Global variables to be used
#---------------------------------------------------------------------
# Debugging features flags
g_adaptive_obst_driving = False
g_adaptive_path_driving = False
g_degree_turn_test      = False


#---------------------------------------------------------------------
# This is the main function for the program
#---------------------------------------------------------------------
def main ():
  #---------------------------------------------------
  # Variables to be used in main
  #---------------------------------------------------
  # N/A

  try:
    left_enable_pwm, right_enable_pwm = setup_gpio()
    create_gui(left_enable_pwm, right_enable_pwm)
    
    while (input to shut car down is entered):
       loop(left_enable_pwm, right_enable_pwm)

  except Exception as Error:
    print(f"Unexpected error detected: {Error}")

  finally:
    destroy(left_enable_pwm, right_enable_pwm)
    


    
# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function holds the reiterative code that accomplishes the main tasks 
#   of the program. Specifically, 
#
# INPUT PARAMETERS:
#   N/A
#
# OUTPUT PARAMETERS:
#   N/A
#
# RETURN:
#   N/A
# -----------------------------------------------------------------------------
def loop():
    direction_state = FORWARD
    
    if(g_adaptive_obst_driving):
        direction_state = determine_distance(gpio_pin, on_time, off_time, repeat_alarm)

    left_enable_pwm, right_enable_pwm = forward_drive_direction(direction_state, left_enable_pwm, right_enable_pwm)
    
    if(g_adaptive_path_driving):
        turn_left_flag = determine_turn_direction(gpio_pin, GPIO.LOW)
        turn_right_flag = determine_turn_direction(gpio_pin, GPIO.LOW)
        if(turn_left_flag != turn_right_flag)
            if(turn_left_flag):
                turn_left()

            if(turn_right_flag):
                turn_right()
    
    if(g_degree_turn_test):
        # Input that polls for input without stoping program and stores it in "degree_input"
        if(degree_input != 0):
            left_enable_pwm, right_enable_pwm = forward_drive_direction(STOP, left_enable_pwm, right_enable_pwm)
            turn_by_degree(degree_input, left_enable_pwm, right_enable_pwm)
    
       
       
   #time.sleep(amount of time needed to let the car drive)



# -----------------------------------------------------------------------------
# DESCRIPTION
#   Sets up GPIO functionality for the program, speciffically for pins 4, 23, & 
#   24.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def setup_gpio():   
     # use BCM GPIO numbering scheme
    GPIO.setmode(GPIO.BCM)
    
    # Set Pin ? to OUTPUT mode
    GPIO.setup(TRIG_GPIO, GPIO.OUT)
    # Set Pin ? to INPUT mode
    GPIO.setup(ECHO_GPIO, GPIO.IN)

RIGHT_WHEEL_2_GPIO      = 23

    # Set Pin 22 to OUTPUT mode
    GPIO.setup(LEFT_WHEEL_ENABLE_GPIO, GPIO.OUT)
    # Set Pin 27 to OUTPUT mode
    GPIO.setup(LEFT_WHEEL_1_GPIO, GPIO.OUT)
    # Set Pin 17 to OUTPUT mode
    GPIO.setup(LEFT_WHEEL_2_GPIO, GPIO.OUT)

    # Set Pin 25 to OUTPUT mode
    GPIO.setup(RIGHT_WHEEL_ENABLE_GPIO, GPIO.OUT)
    # Set Pin 24 to OUTPUT mode
    GPIO.setup(RIGHT_WHEEL_1_GPIO, GPIO.OUT) 
    # Set Pin 23 to OUTPUT mode
    GPIO.setup(RIGHT_WHEEL_2_GPIO, GPIO.OUT) 

    # Set Pin ? to INPUT mode
    GPIO.setup(LEFT_OBST_GPIO, GPIO.IN)
    # Set Pin ? to INPUT mode
    GPIO.setup(RIGHT_OBST_GPIO, GPIO.IN)

    # Set Pin ? to OUTPUT mode
    GPIO.setup(LED_GPIO, GPIO.OUT) 

    # Create PWM Objects
    left_enable_pwm = GPIO.PWM (LEFT_WHEEL_ENABLE_GPIO, PWM_FREQUENCY)
    right_enable_pwm = GPIO.PWM (RIGHT_WHEEL_ENABLE_GPIO, PWM_FREQUENCY)

   return left_enable_pwm, right_enable_pwm
     


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function sends a trigger signal to the HS-SR04. This signal will be 
#   emitted from the sensor and reflected off a surface. This reflection will 
#   be used to calculate the distance between the recieving sensor and the  
#   aforementioned surface
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def send_trigger_pulse():
    GPIO.output(TRIG_GPIO,GPIO.HIGH) #make trigPin send 10us high level
    time.sleep(READ_DELAY_10NS) #10us
    GPIO.output(TRIG_GPIO,GPIO.LOW)



# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function obtains the reflected signal and uses it to calculate the 
#   distance between the recieving sensor and a surface. This distance is
#   returned to the calling function. The program will read if a specified GPIO
#   has a signal at a specified logic level. If it doesn't receive a signal 
#   within the alloted time, it will return '0.'
#
# INPUT PARAMETERS:
#   gpio_pin - An input signal that will be reading the reflected signal
#   logic_level - A logic level that indicates that a signal is recieved
#   time_out - The estimated amount of time to wait for a signal.
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   distance - The distance between the sensor and a surface
# -----------------------------------------------------------------------------
def measure_return_echo(gpio_pin, logic_level, time_out):
    t0 = time.time()
    while(GPIO.input(gpio_pin) != logic_level):
        if((time.time() - t0) > time_out * DIVIDE_BY_TIME):
            return 0;
    t0 = time.time()
    while(GPIO.input(gpio_pin) == logic_level):
        if((time.time() - t0) > time_out * DIVIDE_BY_TIME):
            return 0;
    pingTime = (time.time() - t0) * REVERT_DIVIDE
    distance = pingTime * SPEED_OF_SOUND / REFLECT_NEGATE / DIVIDE_BY_TIME
    return distance



# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function will send a trigger signal and receive the reflected signal
#   to determine the calculated distance to determine if the external alarm
#   should be sounded at a certain rate in a certain amount of cycles
#
# INPUT PARAMETERS:
#   gpio_pin - An output pin that will turn on/off the alarm
#   time_on - The amount of time the alarm will be left on
#   time_off - The amount of time the alarm will be left off
#   repeat_alarm - The amount of times the alarm cycle will repeat
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#    distance - The distance between the sensor and a surface
# -----------------------------------------------------------------------------
def determine_distance(gpio_pin):
    direction_state = FORWARD

    send_trigger_pulse()
    distance = measure_return_echo(ECHO_GPIO, GPIO.HIGH, TIME_OUT)
    
    if(distance < OBST_MAX_DIST):
        direction_state = BACKWARDS

    if(distance == OBST_MAX_DIST):
        direction_state = STOP

    return direction_state



# -----------------------------------------------------------------------------
# DESCRIPTION
#   Ensures that certain functions, specifically the GPIO pins, are turned off.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def turn_left():
   


# -----------------------------------------------------------------------------
# DESCRIPTION
#   Ensures that certain functions, specifically the GPIO pins, are turned off.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def turn_right():
   


# -----------------------------------------------------------------------------
# DESCRIPTION
#   Ensures that certain functions, specifically the GPIO pins, are turned off.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def forward_drive_direction():
   


# -----------------------------------------------------------------------------
# DESCRIPTION
#   
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def determine_turn_direction(gpio_pin, logic_level):
   turn_status = False

   if(GPIO.input(gpio_pin) == logic_level):
      turn_status = True

    return turn_status



# -----------------------------------------------------------------------------
# DESCRIPTION
#   
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def turn_by_degree(degree_input, left_enable_pwm, right_enable_pwm):
    while(#checks if the car has turned to the specified angle):
        if(degree_input > 0):
            # Turns the car to the left to the specified angle
        else:
            # Turns the car to the right to the specified angle

        # Insert appropriate delay for car to turn

    return left_enable_pwm, right_enable_pwm



# -----------------------------------------------------------------------------
# DESCRIPTION
#   Ensures that certain functions, specifically the GPIO pins, are turned off.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def destroy(left_enable_pwm, right_enable_pwm):
    left_enable_pwm.stop()
    right_enable_pwm.stop()
    GPIO.cleanup()
    # Other functions are shut down



# Call the main function.
main()
