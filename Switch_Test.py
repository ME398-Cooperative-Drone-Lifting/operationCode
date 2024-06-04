import time
import lgpio as GPIO

button_pin = 16 # GPIO pin we set the switch 
GPIO.gpiochip_open(0)
GPIO.gpio_claim_input(0, button_pin)

while True:
    button_state = GPIO.gpio_read(0, button_pin)
    if button_state == 0:
        print("Button is pressed")
    else:
        continue