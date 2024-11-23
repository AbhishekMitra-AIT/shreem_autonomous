from machine import Pin, PWM
from time import sleep

# Define motor control pins
DIR_R = 13    # PWM for Motor 1
PWM_R = 12    # Direction for Motor 1
DIR_L = 11    # PWM for Motor 2
PWM_L = 10    # Direction for Motor 2

# Set up PWM for each motor
pwm_r = PWM(Pin(PWM_R))
pwm_l = PWM(Pin(PWM_L))

# Set PWM frequency for the motors
pwm_r.freq(1000)  # 1 kHz frequency for Motor 1
pwm_l.freq(1000)  # 1 kHz frequency for Motor 2

# Set up direction control pins
dir_r = Pin(DIR_R, Pin.OUT)
dir_l = Pin(DIR_L, Pin.OUT)

# Function to control Motor 1
def control_motor_right(speed, direction):
    # Set direction: 1 for forward, 0 for reverse
    dir_r.value(direction)
    
    # Set speed (duty cycle 0 to 65535)
    duty_cycle = int((speed / 100) * 65535)  # Convert percentage to duty cycle
    pwm_r.duty_u16(duty_cycle)

# Function to control Motor 2
def control_motor_left(speed, direction):
    # Set direction: 1 for forward, 0 for reverse
    dir_l.value(direction)
    
    # Set speed (duty cycle 0 to 65535)
    duty_cycle = int((speed / 100) * 65535)  # Convert percentage to duty cycle
    pwm_l.duty_u16(duty_cycle)

# Example usage
try:
    while True:
        # Forward at 50% speed
        print("Right & Left Motor: Forward at 50% speed")
        control_motor_right(50, 1)  # Motor 1 at 50% speed, forward direction
        control_motor_left(50, 1)  # Motor 2 at 50% speed, forward direction
        sleep(10)
        
        # Reverse at 50% speed
        print("Right & Left Motor: Reverse at 50% speed")
        control_motor_right(55, 0)  # Motor 1 at 50% speed, reverse direction
        control_motor_left(50, 0)  # Motor 2 at 50% speed, reverse direction
        sleep(10)
        
        # Stop both motors
        print("Stopping both motors")
        control_motor_right(0, 1)  # Stop Motor 1
        control_motor_left(0, 1)  # Stop Motor 2
        sleep(5)

except KeyboardInterrupt:
    # Stop the motors if the script is interrupted
    print("Stopping motors")
    control_motor1(0, 1)  # Stop Motor 1
    control_motor2(0, 1)  # Stop Motor 2
