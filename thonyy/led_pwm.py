from machine import Pin, PWM
from time import sleep

# Set up PWM on GPIO pin 25 (built-in LED)
led = PWM(Pin(25))  # Initialize PWM on pin 25
led.freq(1000)      # Set frequency to 1000 Hz

# Function to change brightness in a loop
try:
    while True:
        # Increase brightness
        for duty in range(0, 65535, 1000):  # Range from 0 to max (65535)
            led.duty_u16(duty)               # Set duty cycle (16-bit resolution)
            sleep(0.01)
        
        # Decrease brightness
        for duty in range(65535, 0, -1000):
            led.duty_u16(duty)
            sleep(0.01)

except KeyboardInterrupt:
    # Turn off LED when exiting
    led.duty_u16(0)
    led.deinit()  # Clean up the PWM settings
