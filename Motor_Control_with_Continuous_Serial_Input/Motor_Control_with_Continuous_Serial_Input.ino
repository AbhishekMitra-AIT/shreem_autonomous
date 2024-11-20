// Define motor control pins
const int DIR_R = 13;    // Direction for Motor 1
const int PWM_R = 12;    // PWM for Motor 1
const int DIR_L = 11;    // Direction for Motor 2
const int PWM_L = 10;    // PWM for Motor 2

// Motor speed constant (0-255)
const int MOTOR_SPEED = 150;  // About 50% speed

// Variables for continuous input
char lastCommand = 0;
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT = 100; // Timeout in milliseconds

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set pin modes
  pinMode(DIR_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  
  // Initially stop motors
  stopMotors();
  
  // Print instructions
  Serial.println("Motor Control Instructions:");
  Serial.println("Hold 'w' - forward");
  Serial.println("Hold 's' - backward");
  Serial.println("Hold 'a' - left");
  Serial.println("Hold 'd' - right");
  Serial.println("Release key to stop motors");
}

void loop() {
  if (Serial.available() > 0) {
    lastCommand = Serial.read();
    lastCommandTime = millis();
    
    switch(lastCommand) {
      case 'w':
        moveForward();
        Serial.println("Moving forward");
        break;
        
      case 's':
        moveBackward();
        Serial.println("Moving backward");
        break;

      case 'a':
        moveLeft();
        Serial.println("Moving left");
        break;
        
      case 'd':
        moveRight();
        Serial.println("Moving right");
        break;
        
      default:
        stopMotors();
        Serial.println("Motors stopped");
        break;
    }
  }
  
  // Stop motors if no command received for TIMEOUT duration
  if (millis() - lastCommandTime > TIMEOUT) {
    if (lastCommand != 0) {  // Only print stop message if we were moving
      stopMotors();
      Serial.println("Motors stopped - timeout");
      lastCommand = 0;
    }
  }
}

void moveForward() {
  // Right motor forward
  digitalWrite(DIR_R, HIGH);  // Forward direction
  analogWrite(PWM_R, MOTOR_SPEED);
  
  // Left motor forward
  digitalWrite(DIR_L, LOW);  // Forward direction
  analogWrite(PWM_L, MOTOR_SPEED);
}

void moveBackward() {
  // Right motor backward
  digitalWrite(DIR_R, LOW);   // Reverse direction
  analogWrite(PWM_R, MOTOR_SPEED);
  
  // Left motor backward
  digitalWrite(DIR_L, HIGH);   // Reverse direction
  analogWrite(PWM_L, MOTOR_SPEED);
}

void moveLeft() {
  // Right motor reverse
  digitalWrite(DIR_R, HIGH);  // Reverse direction
  analogWrite(PWM_R, MOTOR_SPEED);
  
  // Left motor forward
  digitalWrite(DIR_L, HIGH);  // Forward direction
  analogWrite(PWM_L, MOTOR_SPEED);
}

void moveRight() {
  // Right motor backward
  digitalWrite(DIR_R, LOW);   // Reverse direction
  analogWrite(PWM_R, MOTOR_SPEED);
  
  // Left motor backward
  digitalWrite(DIR_L, LOW);   // Reverse direction
  analogWrite(PWM_L, MOTOR_SPEED);
}

void stopMotors() {
  // Stop both motors
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);
}
