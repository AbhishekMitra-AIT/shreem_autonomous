from gpiozero import LED
from time import sleep

led = LED(4) 

while True:
    led.on()
    sleep(0.2)
    led.off()
    sleep(0.2)

led.off()
