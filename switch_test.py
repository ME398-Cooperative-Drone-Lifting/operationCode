import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
button_pin = 16 # GPIO pin we set the switch 
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
    button_state = GPIO.input(button_pin)
    if button_state == False:
        print("FUCK FUCK FUCK FUCK FUCK FUCK FUCK FUCK FUCK FUCK FUCK FUCK FUCK FUCK FUCK")
    else:
        continue