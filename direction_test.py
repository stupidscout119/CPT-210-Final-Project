# *****************************************************************************
# ***************************  Python Source Code  ****************************
# *****************************************************************************
#
#   DESIGNER NAME:  Dakota Maxwell & Isaac Cassady
#
#       FILE NAME:  direction_test.py
#
# DESCRIPTION
#   INSERT HERE
#
# *****************************************************************************
#*****************************************************************************
# Imports to be used in program
#*****************************************************************************
import RPi.GPIO as GPIO



#---------------------------------------------------------------------
# This is the main function for the program
#---------------------------------------------------------------------
def main ():
  #---------------------------------------------------
  # Variables to be used in main
  #---------------------------------------------------
  # N/A

  try:
    setup_gpio()
    
    while (True):
       loop()

   except KeyboardInterrupt:
        print()
        print("CTRL-c detected.")

  finally:
    GPIO.cleanup()



# -----------------------------------------------------------------------------
# DESCRIPTION
#   Sets up GPIO functionality for the program, speciffically for pins 
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

    # Set Pin 5 to INPUT mode
    GPIO.setup(LEFT_OBST_GPIO, GPIO.IN)
    # Set Pin 6 to INPUT mode
    GPIO.setup(RIGHT_OBST_GPIO, GPIO.IN)



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
def loop ():
    #---------------------------------------------------
    #Variables to be used in main
    #---------------------------------------------------
    # N/A
    turn_left_flag = determine_turn_direction(LEFT_OBST_GPIO, GPIO.LOW)
    turn_right_flag = determine_turn_direction(RIGHT_OBST_GPIO, GPIO.LOW)

    if(turn_left_flag != turn_right_flag):
        if(turn_left_flag):
            print("Hello\n")

        if(turn_right_flag):
            print("Goodbye\n")



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
