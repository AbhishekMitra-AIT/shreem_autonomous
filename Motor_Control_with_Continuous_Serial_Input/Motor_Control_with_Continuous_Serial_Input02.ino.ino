// Define motor control pins
const int DIR_R = 13;    // Direction for Motor 1
const int PWM_R = 12;    // PWM for Motor 1
const int DIR_L = 11;    // Direction for Motor 2
const int PWM_L = 10;    // PWM for Motor 2

// Motor speed constant (0-255)
const int MOTOR_SPEED = 150;  // About 60% speed

// Variables for continuous input
char lastCommand = 0;
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT = 50; // Reduced timeout for quicker response
bool isRampingUp = false;

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
    char currentCommand = Serial.read();
    lastCommandTime = millis();
    
    if (currentCommand != lastCommand) {
      isRampingUp = true;
      lastCommand = currentCommand;
      
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
  }
  
  // Stop motors if no command received for TIMEOUT duration
  if (millis() - lastCommandTime > TIMEOUT) {
    if (lastCommand != 0) {  // Only stop if we were moving
      stopMotors();
      lastCommand = 0;
      isRampingUp = false;
      Serial.println("Motors stopped - key released");
    }
  }
}

void moveForward() {
  // Set directions
  digitalWrite(DIR_R, HIGH);  // Right motor forward
  digitalWrite(DIR_L, LOW);   // Left motor forward
  
  // Ramp up both motors simultaneously
  for(int i = 0; i < MOTOR_SPEED; i++) {
    if (Serial.available() > 0 || (millis() - lastCommandTime > TIMEOUT)) {
      // If new command available or timeout, stop ramping
      return;
    }
    analogWrite(PWM_R, i);
    analogWrite(PWM_L, i);
    Serial.println(i);
    delay(2);
  }
}

void moveBackward() {
  // Set directions
  digitalWrite(DIR_R, LOW);    // Right motor backward
  digitalWrite(DIR_L, HIGH);   // Left motor backward
  
  // Ramp up both motors simultaneously
  for(int i = 0; i < MOTOR_SPEED; i++) {
    if (Serial.available() > 0 || (millis() - lastCommandTime > TIMEOUT)) {
      // If new command available or timeout, stop ramping
      return;
    }
    analogWrite(PWM_R, i);
    analogWrite(PWM_L, i);
    Serial.println(i);
    delay(2);
  }
}

void moveLeft() {
  // Set directions
  digitalWrite(DIR_R, HIGH);  // Right motor forward
  digitalWrite(DIR_L, HIGH);  // Left motor backward
  
  // Ramp up both motors simultaneously
  for(int i = 0; i < MOTOR_SPEED; i++) {
    if (Serial.available() > 0 || (millis() - lastCommandTime > TIMEOUT)) {
      // If new command available or timeout, stop ramping
      return;
    }
    analogWrite(PWM_R, i);
    analogWrite(PWM_L, i);
    delay(2);
  }
}

void moveRight() {
  // Set directions
  digitalWrite(DIR_R, LOW);   // Right motor backward
  digitalWrite(DIR_L, LOW);   // Left motor forward
  
  // Ramp up both motors simultaneously
  for(int i = 0; i < MOTOR_SPEED; i++) {
    if (Serial.available() > 0 || (millis() - lastCommandTime > TIMEOUT)) {
      // If new command available or timeout, stop ramping
      return;
    }
    analogWrite(PWM_R, i);
    analogWrite(PWM_L, i);
    delay(2);
  }
}

void stopMotors() {
  // Immediately stop both motors
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);
}
