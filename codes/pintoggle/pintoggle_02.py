from gpiozero import PWMLED
from time import sleep

pwmMax = 1

led = PWMLED(4) 
 
i = 0.1
while True:
    while i < pwmMax:
        led.value = i
        print(i)
        sleep(0.2)
        i += 0.1

    i = 0.9

    while i > 0.1:
        led.value = i
        print(i)
        sleep(0.2)
        i -= 0.1

    i = 0.1
