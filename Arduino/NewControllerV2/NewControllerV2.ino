#include <Wire.h>
#include <PololuMaestro.h>
#include <CytronMotorDriver.h>

#define SLAVE_ADDRESS 8

const int LATERAL_POT = A0;

// #######################################
// Motor Control
// #######################################

// Cytron motor driver
CytronMD drive_axel_motor(PWM_DIR, 9, 10);
CytronMD drive_flywheel_motor(PWM_DIR, 3, 4);
// Maestro
MicroMaestro maestro(Serial3);
#define head_rotation_servo 0
#define head_tilt_servo_x 1
#define head_tilt_servo_y 2
#define drive_tilt_servo 4

// Variables to hold the received float values
float receivedTiltX, receivedTiltY, receivedTiltZ;
float receivedAngularVelX, receivedAngularVelY, receivedAngularVelZ;
float receivedCartX, receivedCartY, receivedCartZ;

float current_HeadX = 6000; // starting at middle position
float current_HeadY = 6000; // starting at middle position
float targetServoX = 6000;
float targetServoY = 6000;
float lastTiltXForUpdate = 0;  
float lastTiltYForUpdate = 0;

String receivedMessage;
String command;
char receivedChar;

int right_stick_x = 128;
int right_stick_y = 128;

// #######################################
// States
// #######################################

#define AUTONOMOUS 0
#define CONTROLLER_INPUT 1
int control_state;

#define HEAD_SPIN_STATIONARY 1
#define HEAD_SPIN_CLOCKWISE 2
#define HEAD_SPIN_ANTICLOCKWISE 3
int state_head_spin;

#define DRIVE_AXEL_STATIONARY 1
#define DRIVE_AXEL_CLOCKWISE 2
#define DRIVE_AXEL_ANTICLOCKWISE 3
int state_drive_axel;

#define DRIVE_AXEL_STATIONARY_NO_PID 1
#define DRIVE_AXEL_CLOCKWISE_NO_PID 2
#define DRIVE_AXEL_ANTICLOCKWISE_NO_PID 3
int state_drive_axel_no_pid;

#define DRIVE_FLYWHEEL_STATIONARY 1
#define DRIVE_FLYWHEEL_CLOCKWISE 2
#define DRIVE_FLYWHEEL_ANTICLOCKWISE 3
int state_drive_flywheel;

#define DRIVE_TILT_MIDDLE 1
#define DRIVE_TILT_CLOCKWISE 2
#define DRIVE_TILT_ANTICLOCKWISE 3
int state_drive_tilt;

#define HEAD_CONTROL_UPRIGHT 0
#define HEAD_CONTROL_MANUAL 1
int state_head_control;

boolean first_entry = true;
boolean first_entry_flywheel = true;

// #######################################
// BEGIN
// #######################################

void setup() {
  // Start serial communication
  Serial.begin(9600);
  // Uno
  Serial1.begin(115200);
  // Com with Nano
  Wire.begin();
  // Initialise Serial3 for communication with Maestro
  Serial3.begin(9600);

  receivedTiltX = 0;
  receivedTiltY = 0;
  control_state = CONTROLLER_INPUT;
  state_head_spin = HEAD_SPIN_STATIONARY;
  state_drive_axel = DRIVE_AXEL_STATIONARY;
  state_drive_axel = DRIVE_AXEL_STATIONARY_NO_PID;
  state_drive_flywheel = DRIVE_FLYWHEEL_STATIONARY;
  state_drive_tilt = DRIVE_TILT_MIDDLE;
  state_head_control = HEAD_CONTROL_UPRIGHT;

  //maestro.setTarget(head_tilt_servo_y, current_HeadX);
  //maestro.setTarget(head_tilt_servo_x, current_HeadY);
}

void loop() {
 
  parseInputs();
//  Serial.print(receivedTiltX);
//  Serial.print(", ");
//  Serial.println(receivedTiltY);
//  Serial.print(current_HeadX);
//  Serial.print(", ");
//  Serial.println(current_HeadY);
//  Serial.println("=======================");

  if(control_state == CONTROLLER_INPUT) {
    parseStatesController();
    parseMovements();
    parseControlStickToHeadControlState();
  } else {
   
  }

  delay(50);
}

void parseInputs() {
  // Read from Controller
  while (Serial1.available() > 0) {
    // Read 1 character
    receivedChar = Serial1.read();
    if (receivedChar == '\n') {
      Serial.println(receivedMessage);
      command = receivedMessage;
      command.trim();
      // Reset the received message
      receivedMessage = "";
    }
    else {
      // Append characters to the received message
      receivedMessage += receivedChar;  
    }
  }

  // Recieve data from Tracking Nano
  Wire.requestFrom(SLAVE_ADDRESS, 36);
  if (Wire.available() >= 36) {
    // Read the bytes and reconstruct the floats
    receivedTiltX = readFloat();
    receivedTiltY = readFloat();
    receivedTiltZ = readFloat();
    receivedAngularVelX = readFloat();
    receivedAngularVelY = readFloat();
    receivedAngularVelZ = readFloat();
    receivedCartX = readFloat();
    receivedCartY = readFloat();
    receivedCartZ = readFloat();

    // Print the received data
//    Serial.print("Tilt X: "); Serial.println(receivedTiltX);
//    Serial.print("Tilt Y: "); Serial.println(receivedTiltY);
//    Serial.print("Tilt Z: "); Serial.println(receivedTiltZ);
//    Serial.print("Angular Velocity X: "); Serial.println(receivedAngularVelX);
//    Serial.print("Angular Velocity Y: "); Serial.println(receivedAngularVelY);
//    Serial.print("Angular Velocity Z: "); Serial.println(receivedAngularVelZ);
//    Serial.print("Cartesian X: "); Serial.println(receivedCartX);
//    Serial.print("Cartesian Y: "); Serial.println(receivedCartY);
//    Serial.print("Cartesian Z: "); Serial.println(receivedCartZ);
  }
 
}

float readFloat() {
  byte data[4];
  for (int i = 0; i < 4; i++) {
    data[i] = Wire.read();  // Read each byte of the float
  }
  float *value = (float*)data;  // Cast byte array to float
  return *value;
}

void parseStatesController(){
  parseAnalogStickInput();
  parseInputsToHeadSpinState();
  parseInputsToDriveAxelStateNoPID();
  parseInputsToDriveAxelState();
  parseInputsToDriveFlywheelState();
  parseInputsToDriveTiltState();
}

void parseMovements(){
  handle_head_spin();
  handle_drive_axel_NO_PID();
  handle_drive_axel_PID();
  handle_drive_flywheel();
  handle_drive_tilt();
}

void parseAnalogStickInput() {
  if(command.startsWith("rx")) {
    int firstEqual = command.indexOf('=');
    int commaPos = command.indexOf(',');

    String rxStr = command.substring(firstEqual+1, commaPos);
    int rxValue = rxStr.toInt();
    right_stick_x = rxValue;

    int secondEqual = command.indexOf('=', commaPos);
    String ryStr = command.substring(secondEqual+1);
    int ryValue = ryStr.toInt();
    right_stick_y = ryValue;

    command = "";
  }
}

void parseInputsToHeadSpinState() {
  if(command.equals("drp")) {
    command = "";
    Serial.print("HEAD_SPIN_CLOCKWISE");
    state_head_spin = HEAD_SPIN_CLOCKWISE;
  } else if(command.equals("dlp")) {
    command = "";
    Serial.print("HEAD_SPIN_ANTICLOCKWISE");
    state_head_spin = HEAD_SPIN_ANTICLOCKWISE;
  } else if(command.equals("drr") || command.equals("dlr")) {
    command = "";
    Serial.print("HEAD_SPIN_STATIONARY");
    state_head_spin = HEAD_SPIN_STATIONARY;
    first_entry = true;
  }
}

void parseInputsToDriveAxelState() {
  if(command.equals("xp")) {
    command = "";
    Serial.print("DRIVE_AXEL_CLOCKWISE");
    state_drive_axel = DRIVE_AXEL_CLOCKWISE;
  } else if(command.equals("L2p")) {
    command = "";
    Serial.print("DRIVE_AXEL_ANTICLOCKWISE");
    state_drive_axel = DRIVE_AXEL_ANTICLOCKWISE;
  } else if(command.equals("xr") || command.equals("L2r")) {
    command = "";
    Serial.print("DRIVE_AXEL_STATIONARY");
    state_drive_axel = DRIVE_AXEL_STATIONARY;
    first_entry = true;
  }
}

void parseInputsToDriveAxelStateNoPID() {
  if(command.equals("tp")) {
    command = "";
    Serial.print("DRIVE_AXEL_CLOCKWISE_NO_PID");
    state_drive_axel_no_pid = DRIVE_AXEL_CLOCKWISE_NO_PID;
  } else if(command.equals("sp")) {
    command = "";
    Serial.print("DRIVE_AXEL_ANTICLOCKWISE_NO_PID");
    state_drive_axel_no_pid = DRIVE_AXEL_ANTICLOCKWISE_NO_PID;
  } else if(command.equals("tr") || command.equals("sr")) {
    command = "";
    Serial.print("DRIVE_AXEL_STATIONARY_NO_PID");
    state_drive_axel_no_pid = DRIVE_AXEL_STATIONARY_NO_PID;
    first_entry = true;
  }
}


void parseInputsToDriveFlywheelState() {
  if(command.equals("dup")) {
    command = "";
    Serial.print("DRIVE_FLYWHEEL_CLOCKWISE");
    state_drive_flywheel = DRIVE_FLYWHEEL_CLOCKWISE;
    first_entry_flywheel = true;
  } else if(command.equals("ddp")) {
    command = "";
    Serial.print("DRIVE_FLYWHEEL_ANTICLOCKWISE");
    state_drive_flywheel = DRIVE_FLYWHEEL_ANTICLOCKWISE;
    first_entry_flywheel = true;
  } else if(command.equals("dur") || command.equals("ddr")) {
    command = "";
    Serial.print("DRIVE_FLYWHEEL_STATIONARY");
    state_drive_flywheel = DRIVE_FLYWHEEL_STATIONARY;
    first_entry_flywheel = true;
  }
}

void parseInputsToDriveTiltState() {
  if(command.equals("R1p")) {
    command = "";
    Serial.print("DRIVE_TILT_CLOCKWISE");
    state_drive_tilt = DRIVE_TILT_CLOCKWISE;
  } else if(command.equals("L1p")) {
    command = "";
    Serial.print("DRIVE_TILT_ANTICLOCKWISE");
    state_drive_tilt = DRIVE_TILT_ANTICLOCKWISE;
  } else if(command.equals("R1r") || command.equals("L1r")) {
    command = "";
    Serial.print("DRIVE_TILT_MIDDLE");
    state_drive_tilt = DRIVE_TILT_MIDDLE;
    first_entry = true;
  }
}

void parseControlStickToHeadControlState() {
  if(right_stick_x != 128 || right_stick_y !=128) {
    int mapped_x = map(right_stick_x, 0, 255, 4000, 8000);
    int mapped_y = map(right_stick_y, 0, 255, 4000, 8000);

    maestro.setSpeed(head_tilt_servo_x, 25);
    maestro.setSpeed(head_tilt_servo_y, 25);
    maestro.setTarget(head_tilt_servo_x, mapped_x);
    maestro.setTarget(head_tilt_servo_y, mapped_y);
  } else {
    head_Upright();
  }
}

void handle_head_spin() {
  if(state_head_spin == HEAD_SPIN_CLOCKWISE) {
    maestro.setTarget(head_rotation_servo, 8000);
  } else if(state_head_spin == HEAD_SPIN_ANTICLOCKWISE) {
    maestro.setTarget(head_rotation_servo, 4000);
  } else if(state_head_spin == HEAD_SPIN_STATIONARY) {
    maestro.setTarget(head_rotation_servo, 6000);
  }
}

void handle_drive_axel_NO_PID() {
  if(state_drive_axel_no_pid == DRIVE_AXEL_CLOCKWISE_NO_PID) {
    drive_axel_motor.setSpeed(100);
  } else if(state_drive_axel_no_pid == DRIVE_AXEL_ANTICLOCKWISE_NO_PID) {
    drive_axel_motor.setSpeed(-100);
  } else if(state_drive_axel_no_pid == DRIVE_AXEL_STATIONARY_NO_PID && state_drive_axel != DRIVE_AXEL_CLOCKWISE && state_drive_axel != DRIVE_AXEL_ANTICLOCKWISE) {
    drive_axel_motor.setSpeed(0); // FIX THIS
  }
}

int flywheel_speed = 15;
int flywheel_increase_interval = 3;
void handle_drive_flywheel() {
  if(state_drive_flywheel == DRIVE_FLYWHEEL_CLOCKWISE) {
    if(first_entry_flywheel) {
      flywheel_speed = 15;
      first_entry_flywheel = false;
    }
    if(flywheel_speed <= 100) {
      flywheel_speed = flywheel_speed+3;
    }
    drive_flywheel_motor.setSpeed(flywheel_speed);
  } else if(state_drive_flywheel == DRIVE_FLYWHEEL_ANTICLOCKWISE) {
    if(first_entry) {
      flywheel_speed = 15;
      first_entry_flywheel = false;
    }
    if(flywheel_speed >= -100) {
      flywheel_speed = flywheel_speed-3;
    }
    drive_flywheel_motor.setSpeed(flywheel_speed);
  } else if(state_drive_flywheel == DRIVE_FLYWHEEL_STATIONARY) {
    drive_flywheel_motor.setSpeed(0);
  }
}

void handle_drive_tilt() {
  if(state_drive_tilt == DRIVE_TILT_CLOCKWISE) {
    maestro.setSpeed(drive_tilt_servo, 50);
    maestro.setTarget(drive_tilt_servo, 8000);
  } else if(state_drive_tilt == HEAD_SPIN_ANTICLOCKWISE) {
    maestro.setSpeed(drive_tilt_servo, 50);
    maestro.setTarget(drive_tilt_servo, 4000);
  } else if(state_drive_tilt == DRIVE_TILT_MIDDLE) {
    maestro.setTarget(drive_tilt_servo, 6000);
  }
}

// PID Parameters
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.05;

float desired_axle_tilt_forward = -25.0; // Y is -30
float desired_axle_tilt_backward = 25.0; // Y is 30
float prev_error = 0.0;
float integral = 0.0;

unsigned long lastTime_axle = 0;

float currentSpeed = 0;
float ramp_interval = 1.0;
float min_speed = 40; // to avoid stall

void handle_drive_axel_PID() {
  if(state_drive_axel == DRIVE_AXEL_CLOCKWISE) {
    if(first_entry) {
      prev_error = 0.0;
      integral = 0.0;
      currentSpeed = 0.0;
      first_entry = false;
    }
    unsigned long currentTime = millis();
    float dt = (currentTime-lastTime_axle)/1000.0;
    float current_axle_tilt = receivedTiltY;

    // 30 - 3 = 27 ------- 30-(-6) = 36
    float error = desired_axle_tilt_forward - current_axle_tilt;
    integral += error * dt;
    float derivative = (error - prev_error) / dt;

    float output = Kp*error + Ki*integral + Kd * derivative;
    output = -output;
    output = constrain(output, -60, 60);
    if(output > 0) {
      currentSpeed = constrain(currentSpeed, min_speed, 60);
    }

    if(output > 0 && output < min_speed) {
      output = min_speed;
      currentSpeed = output;
    }

    if(output > currentSpeed) {
      currentSpeed = currentSpeed + ramp_interval;
      if(currentSpeed > output) {
        currentSpeed = output;
      }
    } else if(output < currentSpeed) {
      currentSpeed = currentSpeed - ramp_interval;
      if(currentSpeed < output) {
        currentSpeed = output;
      }
    }

    if(current_axle_tilt < desired_axle_tilt_forward+3 && current_axle_tilt > desired_axle_tilt_forward-3) {
      currentSpeed = 20;
    }

    drive_axel_motor.setSpeed(currentSpeed);

    Serial.print("Output: ");
    Serial.print(output);
    Serial.print(", Motor Speed: ");
    Serial.print(currentSpeed);
    Serial.print(", Current Tilt: ");
    Serial.print(current_axle_tilt);
    Serial.print(", Desired: ");
    Serial.println(desired_axle_tilt_forward);

    prev_error = error;
    lastTime_axle=currentTime;
  } else if(state_drive_axel == DRIVE_AXEL_ANTICLOCKWISE) {
    if(first_entry) {
      prev_error = 0.0;
      integral = 0.0;
      currentSpeed = 0.0;
      first_entry = false;
    }
    unsigned long currentTime = millis();
    float dt = (currentTime-lastTime_axle)/1000.0;
    float current_axle_tilt = receivedTiltY;

    float error = desired_axle_tilt_backward - current_axle_tilt;
    integral += error * dt;
    float derivative = (error - prev_error) / dt;

    float output = Kp*error + Ki*integral + Kd * derivative;
    output = constrain(output, -60, 60);
    if(output > 0) {
      currentSpeed = constrain(currentSpeed, min_speed, 60);
    }

    if(output > 0 && output < min_speed+10) {
      output = min_speed+10;
      currentSpeed = output;
    }

    if(output > currentSpeed) {
      currentSpeed = currentSpeed + ramp_interval;
      if(currentSpeed > output) {
        currentSpeed = output;
      }
    } else if(output < currentSpeed) {
      currentSpeed = currentSpeed - ramp_interval;
      if(currentSpeed < output) {
        currentSpeed = output;
      }
    }

    // if tilt is less than 20+4 and more than 20-3
    if(current_axle_tilt < desired_axle_tilt_backward+3 && current_axle_tilt > desired_axle_tilt_backward-3) {
      currentSpeed = 20;
    }

    drive_axel_motor.setSpeed(-currentSpeed);

    Serial.print("Output: ");
    Serial.print(output);
    Serial.print(", Motor Speed: ");
    Serial.print(currentSpeed);
    Serial.print(", Current Tilt: ");
    Serial.print(current_axle_tilt);
    Serial.print(", Desired: ");
    Serial.println(desired_axle_tilt_backward);

    prev_error = error;
    lastTime_axle=currentTime;
  } else if(state_drive_axel == DRIVE_AXEL_STATIONARY && state_drive_axel_no_pid != DRIVE_AXEL_CLOCKWISE_NO_PID && state_drive_axel_no_pid != DRIVE_AXEL_ANTICLOCKWISE_NO_PID) {
    drive_axel_motor.setSpeed(0);
  }
}

// Tuning parameters for hjjead_Upright
const float smoothingFactor = 0.2;
const float tiltDeadband = 7.0;

void head_Upright() {
  //int potentialTargetX = (int)map(receivedTiltX, -20, 23, 8000, 4000);
  int potentialTargetX = (int)map(receivedTiltX, -16, 23, 8000, 4000);
  int potentialTargetY = (int)map(receivedTiltY, -24, 20, 4000, 8000);
 
  if (abs(receivedTiltX - lastTiltXForUpdate) >= tiltDeadband) {
    targetServoX = potentialTargetX;
    lastTiltXForUpdate = receivedTiltX;
  }
 
  if (abs(receivedTiltY - lastTiltYForUpdate) >= tiltDeadband) {
    targetServoY = potentialTargetY;
    lastTiltYForUpdate = receivedTiltY;
  }
 
  // Linear smoothing: always gradually move the current servo position toward the target
  current_HeadX = current_HeadX + smoothingFactor * (targetServoX - current_HeadX);
  current_HeadY = current_HeadY + smoothingFactor * (targetServoY - current_HeadY);
 
  // Send the smoothed positions to the servos
  maestro.setTarget(head_tilt_servo_x, (int)current_HeadX);
  maestro.setTarget(head_tilt_servo_y, (int)current_HeadY);
}
