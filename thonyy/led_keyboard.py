from machine import Pin
import time

# Setup the built-in LED
led = Pin('LED', Pin.OUT)  # Built-in LED on Pico
led1 = Pin(2, Pin.OUT)
led2 = Pin(3, Pin.OUT)
led3 = Pin(4, Pin.OUT)
led4 = Pin(5, Pin.OUT)

while True:
    # Read a character from the serial input
    command = input("Enter command (w=up, s=down, a=left, d=right): ")
    
    if command == 'w':  # Up
        print("Up command received")
        led1.value(1)
        time.sleep(1)
        led1.value(0)
        
    elif command == 's':  # Down
        print("Down command received")
        for _ in range(2):  # Blink twice
            led2.value(1)
            time.sleep(0.2)
            led2.value(0)
            time.sleep(0.2)
            
    elif command == 'a':  # Left
        print("Left command received")
        for _ in range(3):  # Blink three times quickly
            led3.value(1)
            time.sleep(0.1)
            led3.value(0)
            time.sleep(0.1)
            
    elif command == 'd':  # Right
        print("Right command received")
        led4.value(1)
        time.sleep(1)  # Stay on for 1 second
        led4.value(0)
    else:
        print('Wrong key pressed')
        
    time.sleep(0.1)  # Small delay between commands