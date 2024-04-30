# *****************************************************************************
# ***************************  Python Source Code  ****************************
# *****************************************************************************
#
#   DESIGNER NAME:  Bruce Link
#
#       FILE NAME:  LCD1602.py
#
# DESCRIPTION
#   This file is the source is intended to run on Raspberry Pi board
#   interfacing to the 1602A LCD module. The driver assumes that the LCD
#   module is 2 line, 16 character display configured with an IIC interface. 
#   This code assumes that the IIC address is 0x27. 
#
#   This code uses the Python SMBus library module for the I2C communication. 
#   To satisfy the timing for the LCD module, the Python time module is used.
#
#   The source code was leveraged from the existing LCD16 driver used in the
#   the course CPT-210. The changes mainly related to code cleanup to
#   improve readability for the student. The source of this original driver
#   is unknown.
#
# *****************************************************************************

import time
import smbus


#---------------------------------------------------
# Constants to be used in program
#---------------------------------------------------
LCD_IIC_ADDRESS  = 0x27

IIC_g_SMBus_obj_PORT_NUM = 1
TIME_DELAY_5MS   = 0.005
TIME_DELAY_2MS   = 0.002


NIBBLE_SHIFT      = 4
UPPER_NIBBLE_MASK = 0xF0
LOWER_NIBBLE_MASK = 0x0F

# control mode for RS (Instruction/Data), RW, and E (Latch)
RS_BIT_MASK       = 0x1
RW_BIT_MASK       = 0x2
EN_BIT_MASK       = 0x4
LATCH_ENABLE      = 0x4
LATCH_DISABLE     = 0x0
READ_ENABLE       = 0x2
WRITE_ENABLE      = 0x0
LATCH_DISABLE     = 0x0
INSTRUCTION_SIG   = 0x0
DATA_SIG          = 0x1

LINE_OFFSET       = 0x40

# Define LCD size (16 characters x 2 lines)
MIN_CHAR_POSITION = 0
MAX_CHAR_POSITION = 15
MIN_LINE_NUM      = 0
MAX_LINE_NUM      = 1

# Define LCD line numbers
LCD_LINE_NUM_1      = 0
LCD_LINE_NUM_2      = 1

# Define LCD character positions
LCD_CHAR_POSITION_1      = 0
LCD_CHAR_POSITION_2      = 1
LCD_CHAR_POSITION_3      = 2
LCD_CHAR_POSITION_4      = 3
LCD_CHAR_POSITION_5      = 4
LCD_CHAR_POSITION_6      = 5
LCD_CHAR_POSITION_7      = 6
LCD_CHAR_POSITION_8      = 7
LCD_CHAR_POSITION_9      = 8
LCD_CHAR_POSITION_10     = 9
LCD_CHAR_POSITION_11     = 10
LCD_CHAR_POSITION_12     = 11
LCD_CHAR_POSITION_13     = 12
LCD_CHAR_POSITION_14     = 13
LCD_CHAR_POSITION_15     = 14
LCD_CHAR_POSITION_16     = 15

# 1602 LCD command
LCD_CLEAR_DISPLAY_CMD    = 0x01
LCD_RETURN_HOME_CMD      = 0x02
LCD_ENTRY_MODE_SET_CMD   = 0x04
LCD_DISPLAY_CNTRL_CMD    = 0x08
LCD_CURSOR_SHIFT_CMD     = 0x10
LCD_FUNCTION_SET_CMD     = 0x20
LCD_SET_CGRAM_ADDR_CMD   = 0x40
LCD_SET_DDRAM_ADDR_CMD   = 0x80

# flags for display entry mode
LCD_ENTRY_RIGHT         = 0x00
LCD_ENTRY_LEFT          = 0x02
LCD_ENTRY_SHIFT_INC     = 0x01
LCD_ENTRY_SHIFT_DEC     = 0x00

# flags for display on/off control
LCD_DISPLAY_ON          = 0x04
LCD_DISPLAY_OFF         = 0x00
LCD_CURSOR_ON           = 0x02
LCD_CURSOR_OFF          = 0x00
LCD_BLINK_ON            = 0x01
LCD_BLINK_OFF           = 0x00

# flags for display/cursor shift
LCD_DISPLAY_MOVE        = 0x08
LCD_CURSOR_MOVE         = 0x00
LCD_MOVE_RIGHT          = 0x04
LCD_MOVE_LEFT           = 0x00

# flags for function set
LCD_8BIT_MODE           = 0x10
LCD_4BIT_MODE           = 0x00
LCD_2_LINE_DISPLAY      = 0x08
LCD_1_LINE_DISPLAY      = 0x00
LCD_5x10_DOTS           = 0x04
LCD_5x8_DOTS            = 0x00


#---------------------------------------------------
# Variables to be used in program
#---------------------------------------------------
g_SMBus_obj = 0


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function writes a byte of data to a specified address on the IIC bus.
#
# INPUT PARAMETERS:
#   addr   - The IIC address to write the data to
#   data   - The data to write to the specified address
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def write_word(addr, data):
  global g_iic_port_number
  temp = data

  if (g_iic_port_number == 1):
    temp |= 0x08
  else:
    temp &= 0xF7

  g_SMBus_obj.write_byte(addr ,temp)


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function will write an 8-bit command (instruction) to the 1602 LCD.
#   The 1602 LCD is configured for an 4-bit interface, so we have to send the
#   upper nibble first followed by the lower nibble. For each nibble we send,
#   we need pulse the latch enable signal.
#
# INPUT PARAMETERS:
#   command   - 8 bit command to send to the 1602 LCD
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def send_command(command):

  # Send bit7-4 of command first
  buffer = command & UPPER_NIBBLE_MASK
  buffer |= (LATCH_ENABLE | WRITE_ENABLE | INSTRUCTION_SIG)
  write_word(g_lcd_iic_addr, buffer)

  time.sleep(TIME_DELAY_2MS)

  # clear enable
  buffer &= (UPPER_NIBBLE_MASK | LOWER_NIBBLE_MASK) & ~EN_BIT_MASK
  write_word(g_lcd_iic_addr, buffer)

  # Send bit3-0 secondly
  buffer = (command & LOWER_NIBBLE_MASK) << NIBBLE_SHIFT
  buffer |= (LATCH_ENABLE | WRITE_ENABLE | INSTRUCTION_SIG)
  write_word(g_lcd_iic_addr, buffer)

  time.sleep(TIME_DELAY_2MS)

  # clear enable
  buffer &= (UPPER_NIBBLE_MASK | LOWER_NIBBLE_MASK) & ~EN_BIT_MASK
  write_word(g_lcd_iic_addr, buffer)

# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function will write an 8-bit data to the 1602 LCD.
#   The 1602 LCD is configured for an 4-bit interface, so we have to send the
#   upper nibble first followed by the lower nibble. For each nibble we send,
#   we need pulse the latch enable signal.
#
# INPUT PARAMETERS:
#   data   - 8 bit data to send to the 1602 LCD
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def send_data(data):
  # Send bit7-4 of data first
  buffer = data & UPPER_NIBBLE_MASK
  buffer |= (LATCH_ENABLE | WRITE_ENABLE | DATA_SIG)
  write_word(g_lcd_iic_addr, buffer)

  time.sleep(TIME_DELAY_2MS)

  # clear enable
  buffer &= (UPPER_NIBBLE_MASK | LOWER_NIBBLE_MASK) & ~EN_BIT_MASK
  write_word(g_lcd_iic_addr, buffer)

  # Now send bit3-0 of data
  buffer = (data & LOWER_NIBBLE_MASK) << NIBBLE_SHIFT
  buffer |= (LATCH_ENABLE | WRITE_ENABLE | DATA_SIG)
  write_word(g_lcd_iic_addr, buffer)

  time.sleep(TIME_DELAY_2MS)

  # clear enable
  buffer &= (UPPER_NIBBLE_MASK | LOWER_NIBBLE_MASK) & ~EN_BIT_MASK
  write_word(g_lcd_iic_addr, buffer)


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function creates a handle for SMBus object for the LCD module on the 
#   IIC bus and creates global variables of the essential information. It then 
#   performs the LCD module initialization sequence for 4-bit interface mode. 
#   Since the 1602 LCD is configured for an 4-bit interface, so we have to 
#   send the upper nibble first followed by the lower nibble. For each nibble 
#   we need pulse the latch enable signal.
#
# INPUT PARAMETERS:
#   iic_addr    - IIC address for the device to communicate with 
#   iic_bus_num - I2C port number device is located on 
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def init(iic_addr, iic_bus_num):
  global g_SMBus_obj
  global g_lcd_iic_addr
  global g_iic_port_number

  # create object of SMBus  to access I2C based Python function
  # capture IIC information into global variables
  g_SMBus_obj       = smbus.SMBus(iic_bus_num)
  g_lcd_iic_addr    = iic_addr
  g_iic_port_number = iic_bus_num

  try:
    # Must initialize to 8-bit mode at first
    send_command(LCD_FUNCTION_SET_CMD | LCD_8BIT_MODE | 0x03)
    time.sleep(TIME_DELAY_5MS)

    # Now we can initialize to 4-bit mode
    send_command(LCD_FUNCTION_SET_CMD | LCD_8BIT_MODE | 0x02)
    time.sleep(TIME_DELAY_5MS)

    # set LCD display configuration
    send_command(LCD_FUNCTION_SET_CMD | LCD_4BIT_MODE | LCD_2_LINE_DISPLAY | 
                 LCD_5x8_DOTS)
    time.sleep(TIME_DELAY_5MS)

    
    send_command(LCD_DISPLAY_CNTRL_CMD | LCD_DISPLAY_ON | LCD_CURSOR_OFF)
    time.sleep(TIME_DELAY_5MS)

    clear()

  except:
    # if any exceptions are detected then give up and return FALSE
    return False
  else:
    return True

# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function sends the command to turn off the display on the module.
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
def display_off():
  send_command(LCD_DISPLAY_CNTRL_CMD | LCD_DISPLAY_OFF | LCD_CURSOR_OFF)
  time.sleep(TIME_DELAY_5MS)


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function sends the clear display command to the module to clears 
#   the LCD display
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
def clear():
  send_command(LCD_CLEAR_DISPLAY_CMD)
  time.sleep(TIME_DELAY_5MS)


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function enables the back-light
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
def openlight():
  g_SMBus_obj.write_byte(LCD_IIC_ADDRESS, LCD_DISPLAY_CNTRL_CMD)
  g_SMBus_obj.close()


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function writes a string to the DDRAM on the LCD module so it can be 
#   displayed at the specified line and character location. This function 
#   ensures that the line number and character position are valid. If either 
#   is invalid, this function clips the line and character positions to the 
#   limits as needed.
#
# INPUT PARAMETERS:
#   char_position - an integer that specifies the starting position to write to 
#   line_num      - an integer that specifies the line number to write to 
#   string        - a string of bytes to write to the DDRAM
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def write(char_position, line_num, string):
  # verify the character position is between MIN and MAX position
  if (char_position < MIN_CHAR_POSITION):
    char_position = MIN_CHAR_POSITION
  elif (char_position > MAX_CHAR_POSITION):
    char_position = MAX_CHAR_POSITION

  # verify the line number is between MIN and MAX line
  if (line_num < MIN_LINE_NUM):
    line_num = MIN_LINE_NUM
  elif (line_num > MAX_LINE_NUM):
    line_num = MAX_LINE_NUM

  # Set the DDRAM address to the correct location
  addr = LCD_SET_DDRAM_ADDR_CMD + (LINE_OFFSET * line_num) + char_position
  send_command(addr)

  # for each character in string, convert to Unicode and write it to the module
  for chr in string:
      send_data(ord(chr))


if __name__ == '__main__':
  print()
  print("****************  LCD1602 IS RUNNING  ****************")
  print()

  try:
    init(LCD_IIC_ADDRESS, IIC_g_SMBus_obj_PORT_NUM)

    clear()

    write(LCD_CHAR_POSITION_1, LCD_LINE_NUM_1, "*** LCD 1602 ***")
    write(LCD_CHAR_POSITION_2, LCD_LINE_NUM_2, "HELLO CPT-210!")

  except:
     print ('I2C Address Error !')

  finally:
    print()
    print("**************** LCD1602 TERMINATED ****************")
    print()
