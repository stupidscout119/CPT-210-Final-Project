# *****************************************************************************
# ***************************  Python Source Code  ****************************
# *****************************************************************************
#                                                                                  
#   DESIGNER NAME:  Dakota Maxwell & Isaac Cassady                                 
#                                                                                  
#       FILE NAME:  final_project_main.py                                                    
#                                                                                      
# DESCRIPTION                                                                   
#   The main goal of the project is to create an RC car with two main modes,      
#   automatous and manual. In automatous mode, when a car is placed on a flat    
#   path marked in black with various turns and obstacles, it will turn to stay
#   on the paths and back away / stop in-front of obstacles until they are 
#   removed. The speed of the car’s motors can be changed by a user via a GUI
#   created by Python’s Tkinter module. In manual mode, the car’s individual 
#   motor speeds and rotation directions are directly controlled by 
#   the GUI.
#
# *****************************************************************************
#*****************************************************************************
# Imports to be used in program
#*****************************************************************************
import RPi.GPIO as GPIO
import time
import tkinter as TK
import threading


#*****************************************************************************
# Constants to be used in program
#*****************************************************************************

# cm
OBST_MIN_DIST           = 11.43
OBST_MAX_DIST           = 20

# Program specific constants
FORWARD                 = 1
BACKWARDS               = -1
STOP                    = 0
LEFT_WHEEL_ENABLE_GPIO  = 22
RIGHT_WHEEL_ENABLE_GPIO = 25
LEFT_WHEEL_1_GPIO       = 27
LEFT_WHEEL_2_GPIO       = 17
RIGHT_WHEEL_1_GPIO      = 24
RIGHT_WHEEL_2_GPIO      = 23
LEFT_OBST_GPIO          = 6
RIGHT_OBST_GPIO         = 5
NO_SPEED                = 0
MAN_MIN_SPEED           = -100
AUTO_MIN_SPEED          = 0
MAX_SPEED               = 100

# Ultrasonic Constants
TRIG_GPIO      = 26
ECHO_GPIO      = 16
MAX_DISTANCE   = 220
TIME_OUT       = MAX_DISTANCE * 60
NANO_SEC_10    = 0.000001
REVERT_DIVIDE  = 1000000
SPEED_OF_SOUND = 340.00
REFLECT_NEGATE = 2.0
DIVIDE_BY_TIME = 10000.00

# Constants for PWM functions
NO_DUTY_CYCLE    = 0
PWM_FREQUENCY    = 1000

# Constants for GUI Functions
WINDOW_SIZE     = "800x250"
X_OFFSET        = "+100" 
Y_OFFSET        = "+100"
WINDOW_OFFSET   = X_OFFSET + Y_OFFSET
WINDOW_GEOMETRY = WINDOW_SIZE + WINDOW_OFFSET

# Delays with varying lengths
READ_DELAY_10NS  = 0.00001
READ_DELAY_50MS  = 0.05
READ_DELAY_100MS = 0.1

#---------------------------------------------------------------------
# Global variables to be used
#---------------------------------------------------------------------
g_autominous_mode       = False
g_adaptive_obst_driving = False
g_adaptive_path_driving = False
g_program_quit          = False
g_direction_state       = FORWARD


#---------------------------------------------------------------------
# This is the main function for the program
#---------------------------------------------------------------------
def main ():
  #---------------------------------------------------
  # Variables to be used in main
  #---------------------------------------------------
  global g_program_quit

  left_enable_pwm, right_enable_pwm = setup_gpio()

  # Starts the PWM of both h-bridge enables
  left_enable_pwm.start(NO_DUTY_CYCLE)
  right_enable_pwm.start(NO_DUTY_CYCLE)

  path_detection = threading.Thread(target=determine_turn, args=())
  path_detection.start()
  obst_detection = threading.Thread(target=determine_direction, args=())
  obst_detection.start()

  while (not g_program_quit):
      create_gui(left_enable_pwm, right_enable_pwm)
      gui_main_window.mainloop()
   
  destroy(left_enable_pwm, right_enable_pwm, path_detection, obst_detection)
    


# -----------------------------------------------------------------------------
# DESCRIPTION
#   Sets up GPIO functionality for the program, specifically for pins 5, 6, 16, 
#   17, 22, 23, 24, 25, 26, & 27
#
# INPUT PARAMETERS:
#   N/A
#
# OUTPUT PARAMETERS:
#   N/A
#
# RETURN:
#   left_enable_pwm & right_enable_pwm- the PWM of both h-bridge enables
# -----------------------------------------------------------------------------
def setup_gpio():   
     # use BCM GPIO numbering scheme
    GPIO.setmode(GPIO.BCM)
    
    # Set Pin 26 to OUTPUT mode
    GPIO.setup(TRIG_GPIO, GPIO.OUT)
    # Set Pin 16 to INPUT mode
    GPIO.setup(ECHO_GPIO, GPIO.IN)

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

    # Set Pin 5 to INPUT mode
    GPIO.setup(LEFT_OBST_GPIO, GPIO.IN)
    # Set Pin 6 to INPUT mode
    GPIO.setup(RIGHT_OBST_GPIO, GPIO.IN)

    # Create PWM Objects
    left_enable_pwm = GPIO.PWM (LEFT_WHEEL_ENABLE_GPIO, PWM_FREQUENCY)
    right_enable_pwm = GPIO.PWM (RIGHT_WHEEL_ENABLE_GPIO, PWM_FREQUENCY)

    return left_enable_pwm, right_enable_pwm



# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function, if called while autonomus mode is activated, will use the
#   distance obtained by the ultrasonic sensor to determine which direction to
#   go on the path
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
def determine_direction():
    global g_program_quit
    
    while(not g_program_quit):
        if(g_autominous_mode):
            g_direction_state = FORWARD
            if(g_adaptive_obst_driving):
                g_direction_state = determine_distance(ECHO_GPIO)
                print(g_direction_state)

            forward_drive_direction(g_direction_state)
            
            time.sleep(READ_DELAY_100MS)



# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function, if called while autonomus mode is activated, will use the
#   values returned from the IR sensors to determine whether to turn and what
#   direction to turn
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
def determine_turn():
    global g_program_quit
    global g_direction_state
    
    while(not g_program_quit):
        if(g_autominous_mode):
            if(g_adaptive_path_driving):
                turn_left_flag = determine_turn_direction(LEFT_OBST_GPIO, GPIO.LOW)
                turn_right_flag = determine_turn_direction(RIGHT_OBST_GPIO, GPIO.LOW)
                if(turn_left_flag != turn_right_flag):
                    if(turn_left_flag):
                        turn_left()

                    if(turn_right_flag):
                        turn_right()
            
            time.sleep(READ_DELAY_50MS)
            



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
#    This function will determine if an object is within stopping / backing up 
#    range of the car.
#
# INPUT PARAMETERS:
#    N/A
#
# OUTPUT PARAMETERS:
#    N/A
#
# RETURN:
#    N/A
# -----------------------------------------------------------------------------
def determine_distance(gpio_pin):
    g_direction_state = FORWARD

    send_trigger_pulse()
    distance = measure_return_echo(ECHO_GPIO, GPIO.HIGH, TIME_OUT)
    
    if(distance < OBST_MIN_DIST):
        g_direction_state = BACKWARDS

    if(distance < OBST_MAX_DIST and distance > OBST_MIN_DIST):
        g_direction_state = STOP
    
    return g_direction_state

# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function configures the GPIO motor pins to turn the car to the left  
#   regardless of direction
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
    global g_direction_state
    
    if(g_direction_state == FORWARD):
        GPIO.output(LEFT_WHEEL_1_GPIO,GPIO.LOW) 
        GPIO.output(LEFT_WHEEL_2_GPIO,GPIO.HIGH)
    
    else:
        GPIO.output(RIGHT_WHEEL_1_GPIO,GPIO.LOW) 
        GPIO.output(RIGHT_WHEEL_2_GPIO,GPIO.HIGH)
    
    
        
# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function configures the GPIO motor pins to turn the car to the right  
#   regardless of direction
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
    global g_direction_state
    
    if(g_direction_state == FORWARD):
        GPIO.output(RIGHT_WHEEL_1_GPIO,GPIO.HIGH) 
        GPIO.output(RIGHT_WHEEL_2_GPIO,GPIO.LOW)
    
    else:
        GPIO.output(LEFT_WHEEL_1_GPIO,GPIO.HIGH) 
        GPIO.output(LEFT_WHEEL_2_GPIO,GPIO.LOW)



# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function configures the GPIO motor pins to go along the path in a 
#   specified direction 
#
# INPUT PARAMETERS:
#   g_direction_state - Integer value that tells the program what direction the
#                       car must go on the path.
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def forward_drive_direction(g_direction_state):
    if(g_direction_state == FORWARD):
        GPIO.output(LEFT_WHEEL_1_GPIO,GPIO.HIGH) 
        GPIO.output(LEFT_WHEEL_2_GPIO,GPIO.LOW) 

        GPIO.output(RIGHT_WHEEL_1_GPIO,GPIO.LOW)
        GPIO.output(RIGHT_WHEEL_2_GPIO,GPIO.HIGH)
    
    elif(g_direction_state == BACKWARDS):
        GPIO.output(LEFT_WHEEL_1_GPIO,GPIO.LOW) 
        GPIO.output(LEFT_WHEEL_2_GPIO,GPIO.HIGH) 

        GPIO.output(RIGHT_WHEEL_1_GPIO,GPIO.HIGH)
        GPIO.output(RIGHT_WHEEL_2_GPIO,GPIO.LOW)

    else:
        GPIO.output(LEFT_WHEEL_1_GPIO,GPIO.LOW) 
        GPIO.output(LEFT_WHEEL_2_GPIO,GPIO.LOW) 

        GPIO.output(RIGHT_WHEEL_1_GPIO,GPIO.LOW)
        GPIO.output(RIGHT_WHEEL_2_GPIO,GPIO.LOW)
   


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function reads the value of a specified IR sensor and determines if 
#   the path takes a turn.
#
# INPUT PARAMETERS:
#   gpio_pin    - The GPIO pin of a specified IR sensor.
#   logic_level - The IR value that indicates that a path turn is detected.
#
# OUTPUT PARAMETERS:
#   N/A
#
# RETURN:
#   turn_status - A flag that indicates that a turn has been detected.
# -----------------------------------------------------------------------------
def determine_turn_direction(gpio_pin, logic_level):
   turn_status = False

   if(GPIO.input(gpio_pin) == logic_level):
      turn_status = True

   return turn_status



# -----------------------------------------------------------------------------
# DESCRIPTION
#   Depending on the selected modes, this program will create and display a GUI
#   for the car. The GUI will have in-depth inmstructions on how to operate the 
#   car.
#
# INPUT PARAMETERS:
#   left_enable_pwm & right_enable_pwm- the PWM of both h-bridge enables
#
# OUTPUT PARAMETERS:
#   A GUI with in-depth operational instructions that adapts to the selected 
#   modes
#
# RETURN:
#   N/A
# -----------------------------------------------------------------------------
def create_gui(left_enable_pwm, right_enable_pwm):
    global gui_main_window
    global g_autominous_mode
    global g_adaptive_path_driving
    global g_adaptive_obst_driving

    # Creates the main GUI window
    gui_main_window = TK.Tk()
    
    #Creates the main window's title
    if(g_autominous_mode):
        gui_main_window.title("CPT-210: Final Project V3 (Automatous Mode)")
    
    else:
        gui_main_window.title("CPT-210: Final Project V3 (Manual Mode)")
    
    # Creates a frame that holds the instructions on how to use the GUI
    desc_frame = TK.Frame(gui_main_window)
    
    # Packs the frame inside the main window
    desc_frame.pack(side=TK.TOP)
    # The directions are written into the frame
    TK.Label(desc_frame, text="DIRECTIONS:").grid(row=0, column=0)
    if(g_autominous_mode):
        TK.Label(desc_frame, text="Move the slider around to adjust the speed of the car").grid(row=0, column=1)
        TK.Label(desc_frame, text="0 = full stop; 100 = full speed").grid(row=2, column=1)
    
    else:
        TK.Label(desc_frame, text="Move sliders around to adjust the respective motor speeds of the car").grid(row=0, column=1)
        TK.Label(desc_frame, text="-100 = full speed backwards; 100 = full speed forward").grid(row=2, column=1)

    TK.Label(desc_frame, text="Press the \"STOP\" button to completely stop the car").grid(row=3, column=1)
    TK.Label(desc_frame, text="Press the \"SWITCH MODE\" button to switch between the car's modes").grid(row=4, column=1)
    
    if(g_autominous_mode):
        TK.Label(desc_frame, text="Press the \"FOLLOW PATH\" button to enable the car to follow a physical 2D floor path").grid(row=5, column=1)
        TK.Label(desc_frame, text="Press the \"AVOID OBSTACLES\" button to enable the car to backup and stop in front of obstacles").grid(row=6, column=1)  
        TK.Label(desc_frame, text="Press the \"QUIT\" button to turn off the program safely").grid(row=7, column=1) 
        TK.Label(desc_frame, text="Follow Path: " + mode_status(g_adaptive_path_driving)).grid(row=9, column=0)
        TK.Label(desc_frame, text="Avoid Obstacles: " + mode_status(g_adaptive_obst_driving)).grid(row=9, column=1)   
        
    else:
        TK.Label(desc_frame, text="Press the \"QUIT\" button to turn off the program safely").grid(row=5, column=1)    
    
    # Creates a frame that will hold the controls in the main window
    ctrl_frame = TK.Frame(gui_main_window)
    ctrl_frame.pack(side=TK.BOTTOM)

    # Creates labels for the slider controls
    # Add labels for the frame
    if(g_autominous_mode):
        TK.Label(ctrl_frame, text="Car Speed").grid(row=0, column=0)
    
    else:
        TK.Label(ctrl_frame, text="Left Motor Speed").grid(row=0, column=0)
        TK.Label(ctrl_frame, text="Right Motor Speed").grid(row=1, column=0)
    
    # Pack the new frame into the window at the bottom
    ctrl_frame.pack(side=TK.BOTTOM)
    
    #Creates handles to PWMs as part of the main window object
    gui_main_window.left_enable_pwm = left_enable_pwm
    gui_main_window.right_enable_pwm = right_enable_pwm
    
    # A slider object for pi_pwm1 (BLU LED segment PWM) is created and properly placed
    if(g_autominous_mode):
        car_speed = TK.Scale(ctrl_frame, from_=AUTO_MIN_SPEED, to=MAX_SPEED, orient=TK.HORIZONTAL, command=update_car_pwm)
        car_speed.grid(row=0, column=1)
        stop_button = TK.Button(ctrl_frame, text='STOP', command=stop_auto_car_pwm)
        stop_button.grid(row=1, column=0)
        mode_button = TK.Button(ctrl_frame, text='SWITCH MODE', command=toggle_mode)
        mode_button.grid(row=1, column=1)
        path_button = TK.Button(ctrl_frame, text='FOLLOW PATH', command=toggle_follow_path)
        path_button.grid(row=1, column=2)
        obst_button = TK.Button(ctrl_frame, text='AVOID OBSTACLES', command=toggle_avoid_obstacles)
        obst_button.grid(row=1, column=3)
        quit_button = TK.Button(ctrl_frame, text='QUIT', command=quit_program)
        quit_button.grid(row=1, column=4)
    
    else:
        left_motor_speed = TK.Scale(ctrl_frame, from_=MAN_MIN_SPEED, to=MAX_SPEED, orient=TK.HORIZONTAL, command=update_left_enable_pwm)
        left_motor_speed.grid(row=0, column=1)
        right_motor_speed = TK.Scale(ctrl_frame, from_=MAN_MIN_SPEED, to=MAX_SPEED, orient=TK.HORIZONTAL, command=update_right_enable_pwm)
        right_motor_speed.grid(row=1, column=1)
        stop_button = TK.Button(ctrl_frame, text='STOP', command=stop_man_car_pwm)
        stop_button.grid(row=2, column=0)
        mode_button = TK.Button(ctrl_frame, text='SWITCH MODE', command=toggle_mode)
        mode_button.grid(row=2, column=1)
        quit_button = TK.Button(ctrl_frame, text='QUIT', command=quit_program)
        quit_button.grid(row=2, column=2)


    # Sets the window to the proper position.
    gui_main_window.geometry(WINDOW_GEOMETRY)
    


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function simultaniously updates the enable PWMs to change the speed of
#   the car while maintaining a forward motion
#
# INPUT PARAMETERS:
#   car_speed - The string of the updated motor speeds from the GUI
#
# OUTPUT PARAMETERS:
#   N/A
#
# RETURN:
#   N/A
# -----------------------------------------------------------------------------
def update_car_pwm(car_speed):
    global g_car_speed
    
    g_car_speed = (int)(car_speed)
    gui_main_window.left_enable_pwm.ChangeDutyCycle(g_car_speed)
    gui_main_window.right_enable_pwm.ChangeDutyCycle(g_car_speed)



# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function exclusively updates the enable PWM of the left motor.
#
# INPUT PARAMETERS:
#   left_motor_speed - The string of the updated left motor speed from the GUI
#
# OUTPUT PARAMETERS:
#   N/A
#
# RETURN:
#   N/A 
# -----------------------------------------------------------------------------
def update_left_enable_pwm(left_motor_speed):
    conv_left_motor_speed = int(left_motor_speed)
    
    if(conv_left_motor_speed > NO_SPEED):
        GPIO.output(LEFT_WHEEL_1_GPIO,GPIO.HIGH) 
        GPIO.output(LEFT_WHEEL_2_GPIO,GPIO.LOW)
        
    elif(conv_left_motor_speed < NO_SPEED):
        GPIO.output(LEFT_WHEEL_1_GPIO,GPIO.LOW) 
        GPIO.output(LEFT_WHEEL_2_GPIO,GPIO.HIGH)
        
    else:
        GPIO.output(LEFT_WHEEL_1_GPIO,GPIO.LOW) 
        GPIO.output(LEFT_WHEEL_2_GPIO,GPIO.LOW)

    gui_main_window.left_enable_pwm.ChangeDutyCycle(abs(conv_left_motor_speed))



# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function exclusively updates the enable PWM of the right motor.
#
# INPUT PARAMETERS:
#   right_motor_speed - The string of the updated left motor speed from the GUI
#
# OUTPUT PARAMETERS:
#   N/A
#
# RETURN:
#   N/A
# -----------------------------------------------------------------------------
def update_right_enable_pwm(right_motor_speed):
    conv_right_motor_speed = int(right_motor_speed)
    
    if(conv_right_motor_speed > NO_SPEED):
        GPIO.output(RIGHT_WHEEL_1_GPIO,GPIO.LOW) 
        GPIO.output(RIGHT_WHEEL_2_GPIO,GPIO.HIGH)
        
    elif(conv_right_motor_speed < NO_SPEED):
        GPIO.output(RIGHT_WHEEL_1_GPIO,GPIO.HIGH) 
        GPIO.output(RIGHT_WHEEL_2_GPIO,GPIO.LOW)
        
    else:
        GPIO.output(RIGHT_WHEEL_1_GPIO,GPIO.LOW) 
        GPIO.output(RIGHT_WHEEL_2_GPIO,GPIO.LOW)

    gui_main_window.right_enable_pwm.ChangeDutyCycle(abs(conv_right_motor_speed))
    
    

# -----------------------------------------------------------------------------
# DESCRIPTION
#   This functions simultaniously sets both the enable PWMs to zero while the 
#   car is in autonomous mode.
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
def stop_auto_car_pwm():
    gui_main_window.left_enable_pwm.ChangeDutyCycle(NO_SPEED)
    gui_main_window.right_enable_pwm.ChangeDutyCycle(NO_SPEED) 
    


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This functions simultaniously sets both the enable PWMs to zero and turn 
#   off both h-bride output GPIO pins of both motors while the car is in manual 
#   mode.
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
def stop_man_car_pwm():
    GPIO.output(LEFT_WHEEL_1_GPIO,GPIO.LOW) 
    GPIO.output(LEFT_WHEEL_2_GPIO,GPIO.LOW)
    
    GPIO.output(RIGHT_WHEEL_1_GPIO,GPIO.LOW) 
    GPIO.output(RIGHT_WHEEL_2_GPIO,GPIO.LOW)
        
    gui_main_window.left_enable_pwm.ChangeDutyCycle(NO_SPEED)
    gui_main_window.right_enable_pwm.ChangeDutyCycle(NO_SPEED)
    
    
    
# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function will allow the user to change between the main modes and 
#   update the GUI.
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
def toggle_mode():
    global g_autominous_mode
    global gui_main_window
    
    g_autominous_mode = not g_autominous_mode
    
    if(g_autominous_mode):
      gui_main_window.left_enable_pwm.start(NO_DUTY_CYCLE)
      gui_main_window.right_enable_pwm.start(NO_DUTY_CYCLE)

    else:
      gui_main_window.left_enable_pwm.start(NO_DUTY_CYCLE)
      gui_main_window.right_enable_pwm.start(NO_DUTY_CYCLE)
      
    gui_main_window.destroy()



# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function toggles a submode where the car follows along a 2D path. The
#   GUI will be updated after this.
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
def toggle_follow_path():
    global g_adaptive_path_driving
    
    stop_auto_car_pwm()
    gui_main_window.destroy()
    g_adaptive_path_driving = not g_adaptive_path_driving    
    
    
    
# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function toggles a submode where the car will back up / stop in front 
#   of obstacles. The GUI will be updated after this.
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
def toggle_avoid_obstacles():
    global g_adaptive_obst_driving
    
    stop_auto_car_pwm()
    gui_main_window.destroy()
    g_adaptive_obst_driving = not g_adaptive_obst_driving  



# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function will allow the program to stop in a safe manner.
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
def quit_program():
    global g_program_quit
    global g_autominous_mode
    
    gui_main_window.destroy()
    
    g_program_quit = not g_program_quit     



# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function will read in the state of the boolean that has been passed as
#   an argument. A string that reflects that state is returned
#
# INPUT PARAMETERS:
#   flag_var - A boolean with an initialized state.
#
# OUTPUT PARAMETERS:
#   N/A
#
# RETURN:
#   flag_string - A string that reflects the state of the flag_var.
# -----------------------------------------------------------------------------
def mode_status(flag_var):
   flag_string = "ON"
   
   if(flag_var):
       flag_string = "ON"
   
   else:
       flag_string = "OFF"  
         
   return flag_string



# -----------------------------------------------------------------------------
# DESCRIPTION
#   Ensures that certain functions, specifically the threads, PWM objects, & 
#   GPIO pins, are turned off.
#
# INPUT PARAMETERS:
#   left_enable_pwm & right_enable_pwm - The enable PWMs of the h-bridge.
#   path_detection & obst_detection - The threads of the program.
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def destroy(left_enable_pwm, right_enable_pwm, path_detection, obst_detection):
    global gui_main_window
    
    path_detection.join()
    obst_detection.join()
    left_enable_pwm.stop()
    right_enable_pwm.stop()
    GPIO.cleanup()



# Call the main function.
main()
