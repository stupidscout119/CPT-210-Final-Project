


import RPi.GPIO as GPIO

LEFT_GPIO_IR  = 5
RIGHT_GPIO_IR = 6

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LEFT_GPIO_IR, GPIO.IN)
    GPIO.setup(RIGHT_GPIO_IR, GPIO.IN)

def main():
    if GPIO.input(LEFT_GPIO_IR) == GPIO.HIGH:
        print('left pin is high')
    elif GPIO.input(RIGHT_GPIO_IR) == GPIO.HIGH:
        print('Right pin is high')

main()