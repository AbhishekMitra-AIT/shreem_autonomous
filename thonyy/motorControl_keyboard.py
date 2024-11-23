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

def stop_motors():
    control_motor_right(0, 1)
    control_motor_left(0, 1)
    print("Motors stopped")

# Main control loop
MOTOR_SPEED = 50  # Set a constant motor speed (50%)

print("Motor Control Instructions:")
print("'w' - Move both motors forward")
print("'s' - Move both motors backward")
print("'a' - Move both motors left")
print("'d' - Move both motors right")
print("'q' - Quit the program")
print("Any other key - Stop motors")

try:
    while True:
        command = input().strip().lower()  # Read input and convert to lowercase
        
        if command == 'w':
            # Move both motors forward
            control_motor_right(MOTOR_SPEED, 1)
            control_motor_left(MOTOR_SPEED, 0)
            print("Moving forward")
            
        elif command == 's':
            # Move both motors backward
            control_motor_right(MOTOR_SPEED, 0)
            control_motor_left(MOTOR_SPEED, 1)
            print("Moving backward")
            
        elif command == 'a':
        # Move left
            control_motor_right(MOTOR_SPEED, 1)
            control_motor_left(MOTOR_SPEED, 1)
            print("Moving left")
            
        elif command == 'd':
        # Move right
            control_motor_right(MOTOR_SPEED, 0)
            control_motor_left(MOTOR_SPEED, 0)
            print("Moving right")
            
        elif command == 'q':
            # Quit the program
            stop_motors()
            print("Exiting program")
            break
            
        else:
            # Stop motors for any other key
            stop_motors()
        
        sleep(0.1)  # Small delay to prevent CPU overload

except KeyboardInterrupt:
    # Stop the motors if the script is interrupted
    stop_motors()
    print("\nProgram terminated")

finally:
    # Clean up
    stop_motors()
    print("Cleanup complete")