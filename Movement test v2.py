#*****************************************************************************
#*****************************  Python  Source Code  ******************************
#*****************************************************************************
#
#   DESIGNER NAME:  Dakota Maxwell
#
#       FILE NAME:  Motor_Test.py
# 
#            DATE:  3/21/2024
#
# DESCRIPTION
#   This program will control the speed of a motor by adjusting the resistance of a potentiometer
#
#   This program will use a breadboard in addition to an analog to digital converter,
#   potentiometer, and an l293 chip to control the speed of a dc motor. The l293 chip
#   will serve as an H-bridge which will be able to reverse direction of the motor
#   by changing the flow of electricity between 2 outputs.
#   The potentiometer will control the speed of the motor by adjusting the resistance
#   of the current flowing through it.
#*****************************************************************************
import RPi.GPIO as GPIO
import time

#-----------------------------------
#Constants to be used in programming
#-----------------------------------
#GPIO based on BCM numbering scheme
LEFT_MOTOR_PIN1       = 27
LEFT_MOTOR_PIN2       = 17
LEFT_ENABLE_PIN       = 22

RIGHT_MOTOR_PIN2      = 24
RIGHT_MOTOR_PIN1      = 23
RIGHT_ENABLE_PIN      = 25

FREQUENCY        = 1000

SHORT_WAIT       = 0.01

# -----------------------------------------------------------------------------
# DESCRIPTION
#   Will configure the GPIO on ports 27, 17, and 22 for the motor output pins and the
#   enable pin which will have the pwm enabled
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
#
# -----------------------------------------------------------------------------
def setup_gpio():
    
    
    #use BCM GPIO numbering scheme
    GPIO.setmode(GPIO.BCM)
    
    #set LED pin to OUTPUT mode
    GPIO.setup(LEFT_MOTOR_PIN1, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_PIN2, GPIO.OUT)
    GPIO.setup(LEFT_ENABLE_PIN, GPIO.OUT)
    
    GPIO.setup(RIGHT_MOTOR_PIN1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_PIN2, GPIO.OUT)
    GPIO.setup(RIGHT_ENABLE_PIN, GPIO.OUT)
    
    #enables pwm
    left_motor_pwm  = GPIO.PWM(LEFT_ENABLE_PIN, FREQUENCY)
    right_motor_pwm = GPIO.PWM(RIGHT_ENABLE_PIN, FREQUENCY)
    
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)

    return left_motor_pwm, right_motor_pwm
    

 
 


# -----------------------------------------------------------------------------
# DESCRIPTION
#   Will use update the speed of the motor based on the analog value detected from the
#   ADC converter
#
# INPUT PARAMETERS:
#   ADC - the value of the current flowing to the analog to digital convrter
#   motor_pwm - the pwm that will control the speed of the motor
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
#
# -----------------------------------------------------------------------------
def motor(left_speed, right_speed, left_motor_pwm, right_motor_pwm):
#    if (value > 0):
#        GPIO.output(MOTOR_PIN1, GPIO.HIGH)
#        GPIO.output(MOTOR_PIN2, GPIO.LOW)
#        print ('Turn Forward...')
#    elif (value < 0):
#        GPIO.output(MOTOR_PIN1, GPIO.LOW)
#        GPIO.output(MOTOR_PIN2, GPIO.HIGH)
#        print ('Turn Backwards...')
#    else :
#        GPIO.output(MOTOR_PIN1, GPIO.LOW)
#        GPIO.output(MOTOR_PIN2, GPIO.LOW)
#        print ('Motor Stop...')
     
     
    #left motor forward?     
    GPIO.output(LEFT_MOTOR_PIN1, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_PIN2, GPIO.LOW)
    
    #right motor forward?
    GPIO.output(RIGHT_MOTOR_PIN1, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_PIN2, GPIO.LOW)
    
        
    left_motor_pwm.start(left_speed)
    right_motor_pwm.start(right_speed)
    print ('The PWM duty cycle is left:(left_speed) right:(right_speed)')

# -----------------------------------------------------------------------------
# DESCRIPTION
#   Will clean up the GPIO ports so there is no damage done to the board due to
#   incorrectly configured states
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
#
# -----------------------------------------------------------------------------
def destroy():
    GPIO.cleanup()


# -----------------------------------------------------------------------------
# DESCRIPTION
#   Will create a loop that will always make the program run
#
# INPUT PARAMETERS:
#   motor_pwm - the pwm for the motor
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
#
# -----------------------------------------------------------------------------
def loop(left_motor_pwm, right_motor_pwm):
    left_speed  = 50
    right_speed = 50
    while True:
        print ('speed is (speed)')
        motor(left_speed, right_speed, left_motor_pwm, right_motor_pwm)
        time.sleep(SHORT_WAIT)



# -----------------------------------------------------------------------------
# DESCRIPTION
#   Main function
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
#
# -----------------------------------------------------------------------------    
def main():
    
    print("The program is running")
    
    try:
        left_motor_pwm, right_motor_pwm = setup_gpio()
        loop(left_motor_pwm, right_motor_pwm)
                    
    except KeyboardInterrupt:
        destroy()
        print()
        print("CTRL-C detected")
    
    finally:
        destroy()
        print("GPIO ports have been cleaned up")
        print()
        print("*******PROGRAM TERMINATED*******")
        
main()
    
    
    
    
    
    
    
    
    
    
    
    