#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
#define DEVICE_NAME "3" //Dynamixel on Serial3(USART3) <-OpenCM 485EXP
#elif defined(__OPENCR__)
#define NanoPiSerial Serial2
#define DEVICE_NAME ""
#endif

#define BAUDRATE 1000000
#define STIR_SERVO 3
#define MOTOR_ID 5
#define CAM_ID 1
#define IR_RIGHT 11     // Line sensor ID
#define IR_MIDDLE 12    // Line sensor ID
#define IR_LEFT 13      // Line sensor ID
#define BTN_1 14
#define BTN_2 15

DynamixelWorkbench dxl_wb;

// Define Pin Numbers for Front Sensor
const int FRONT_TRIG_PIN = 2; // OpenCR Pin 2
const int FRONT_ECHO_PIN = 3; // OpenCR Pin 3

// Define Pin Numbers for First Side Sensor
const int LEFT_TRIG_PIN = 4; // OpenCR Pin 4
const int LEFT_ECHO_PIN = 5; // OpenCR Pin 5

// Define Pin Numbers for Second Side Sensor
const int RIGHT_TRIG_PIN = 6; // OpenCR Pin 6
const int RIGHT_ECHO_PIN = 7; // OpenCR Pin 7

// Define variables for duration and distance
long front_duration, LEFT_duration, RIGHT_duration;
int front_distance, LEFT_distance, RIGHT_distance;

// Define servo positions in radians
const float START_POSITION_RADIAN = 20.0 * PI / 180.0;
const float LEFT_TURN_RADIAN = -20.0 * PI / 180.0;
const float RIGHT_TURN_RADIAN = 60.0 * PI / 180.0; // User set to 60.0 * PI / 180.0
const int OBSTACLE_DISTANCE_CM = 12; // Distance threshold in cm (Used for front stop/avoidance)
const int SIDE_AVOID_DISTANCE_CM = 8; // Distance threshold in cm for side avoidance

// Conversion of radians to servo values
int32_t start_position_value;
int32_t left_turn_value;
int32_t right_turn_value;

// Motor control constants
const uint16_t MAX_POWER_ADDR = 24;
const uint16_t ENCODER_MODE_ADDR = 27;
const uint16_t POWER_ADDR = 28;
const uint16_t ENCODER_ADDR = 32;
const uint16_t GOAL_SPEED_ADDR = 40;
const int16_t MOTOR_POWER_FORWARD = 300; // Power value to move forward
const int16_t MOTOR_POWER_SEARCH = 230; // Power value for slow search/junction
const uint8_t LINE_DATA = 27; // Register address for line sensor data
const uint8_t BTN_DATA = 27; // register address for the button data (Assuming the same register)

// Distance calculation variables
long previous_encoder_value = 0;  
const float ENCODER_TO_CM = 0.019; // Calibrate this value (cm per encoder tick)
float total_distance_cm = 0;

// Global variables for line sensor data
uint8_t ir_right_data = 0;
uint8_t ir_middle_data = 0;
uint8_t ir_left_data = 0;

// ========== STATE MANAGEMENT ==========
enum RobotMode {
  MODE_OFF = 0,
  MODE_LIDAR = 1,
  MODE_ULTRASONIC_LINE = 2
};

volatile RobotMode current_mode = MODE_OFF;
bool mode_initialized = false;

// Button timing constants
const unsigned long CONSECUTIVE_PRESS_TIMEOUT = 1500; 
const unsigned long BUTTON_DEBOUNCE_TIME = 50; 

// Button state variables for Button 1 (LIDAR)
int btn1_press_count = 0;
unsigned long btn1_last_press_time = 0;
bool btn1_is_down = false;

// Button state variables for Button 2 (Ultrasonic+Line)
int btn2_press_count = 0;
unsigned long btn2_last_press_time = 0;
bool btn2_is_down = false;

// LIDAR data structure and variables
struct LidarData {
  float front;
  float right;
  float back;
  float left;
  bool dataReceived;
  bool lidarConnected;
};

LidarData lidar = {0, 0, 0, 0, false, false};
String inputString = "";
bool stringComplete = false;

// LIDAR obstacle thresholds (in meters)
const float LIDAR_OBSTACLE_THRESHOLD = 0.35;
const float LIDAR_CRITICAL_THRESHOLD = 0.20;

// Function prototypes
void readLineSensorData();
uint8_t readButtonState(uint8_t button_id);
void readLidarData();
void implementLidarObstacleAvoidance();
void implementUltrasonicObstacleAvoidance();
void implementLineFollowing();
bool checkLidarConnection();
void handleButtonPresses();
void setMotorPower(int16_t power);
void turnLeft();
void returnToCenter();
void turnRight();
void moveStop();
void moveBackward();
void moveForward();
void dynamicTurn(int direction);

void setMotorPower(int16_t power) {
  const char *log;
  dxl_wb.writeRegister(MOTOR_ID, POWER_ADDR, (uint16_t)2, (uint8_t*)&power, &log);
}

void turnLeft() {
  const char *log;
  dxl_wb.goalPosition(STIR_SERVO, left_turn_value, &log);
}

void returnToCenter() {
  const char *log;
  dxl_wb.goalPosition(STIR_SERVO, start_position_value, &log);
}

void turnRight() {
  const char *log;
  dxl_wb.goalPosition(STIR_SERVO, right_turn_value, &log);
}

uint8_t readButtonState(uint8_t button_id) {
  const char *log;
  uint32_t temp_data;
  uint16_t data_length = 1;
  uint8_t button_state = 1;

  if (dxl_wb.readRegister(button_id, BTN_DATA, data_length, &temp_data, &log)) {
    button_state = (uint8_t)temp_data;
  }
  return button_state;
}

// ========== TWO BUTTON HANDLER ==========
void handleButtonPresses() {
    unsigned long current_time = millis();
    
    // Handle Button 1 (LIDAR Mode)
    uint8_t btn1_state = readButtonState(BTN_1);
    
    // Reset press count if timeout occurs
    if (btn1_press_count > 0 && (current_time - btn1_last_press_time) > CONSECUTIVE_PRESS_TIMEOUT) {
        btn1_press_count = 0;
    }

    // Button 1 press detection (falling edge)
    if (btn1_state == 0 && !btn1_is_down) {
        btn1_is_down = true;
        
        // Check for debounce
        if (current_time - btn1_last_press_time > BUTTON_DEBOUNCE_TIME || btn1_press_count == 0) {
            btn1_press_count++;
            btn1_last_press_time = current_time;
            
            // 3 presses: Toggle LIDAR Mode
            if (btn1_press_count == 3) {
                if (current_mode == MODE_LIDAR) {
                    // Turn OFF if already in LIDAR mode
                    current_mode = MODE_OFF;
                    mode_initialized = false;
                    setMotorPower(0);
                    returnToCenter();
                    Serial.println(">>> ROBOT STOPPED (LIDAR Mode) <<<");
                } else {
                    // Turn ON LIDAR mode
                    current_mode = MODE_LIDAR;
                    mode_initialized = false;
                    Serial.println(">>> LIDAR MODE ACTIVATED <<<");
                }
                btn1_press_count = 0;
            }
        }
    } 
    // Button 1 release detection
    else if (btn1_state == 1 && btn1_is_down) {
        btn1_is_down = false;
    }

    // Handle Button 2 (Ultrasonic+Line Mode)
    uint8_t btn2_state = readButtonState(BTN_2);
    
    // Reset press count if timeout occurs
    if (btn2_press_count > 0 && (current_time - btn2_last_press_time) > CONSECUTIVE_PRESS_TIMEOUT) {
        btn2_press_count = 0;
    }

    // Button 2 press detection (falling edge)
    if (btn2_state == 0 && !btn2_is_down) {
        btn2_is_down = true;
        
        // Check for debounce
        if (current_time - btn2_last_press_time > BUTTON_DEBOUNCE_TIME || btn2_press_count == 0) {
            btn2_press_count++;
            btn2_last_press_time = current_time;
            
            // 3 presses: Toggle Ultrasonic+Line Mode
            if (btn2_press_count == 3) {
                if (current_mode == MODE_ULTRASONIC_LINE) {
                    // Turn OFF if already in Ultrasonic+Line mode
                    current_mode = MODE_OFF;
                    mode_initialized = false;
                    setMotorPower(0);
                    returnToCenter();
                    Serial.println(">>> ROBOT STOPPED (Ultrasonic+Line Mode) <<<");
                } else {
                    // Turn ON Ultrasonic+Line mode
                    current_mode = MODE_ULTRASONIC_LINE;
                    mode_initialized = false;
                    Serial.println(">>> ULTRASONIC + LINE FOLLOWING MODE ACTIVATED <<<");
                }
                btn2_press_count = 0;
            }
        }
    } 
    // Button 2 release detection
    else if (btn2_state == 1 && btn2_is_down) {
        btn2_is_down = false;
    }
}

void readLineSensorData() {
  const char *log;
  uint32_t temp_data;
  uint16_t data_length = 1;

  if (dxl_wb.readRegister(IR_RIGHT, LINE_DATA, data_length, &temp_data, &log)) {
    ir_right_data = (uint8_t)temp_data;
  }
  if (dxl_wb.readRegister(IR_MIDDLE, LINE_DATA, data_length, &temp_data, &log)) {
    ir_middle_data = (uint8_t)temp_data;
  }
  if (dxl_wb.readRegister(IR_LEFT, LINE_DATA, data_length, &temp_data, &log)) {
    ir_left_data = (uint8_t)temp_data;
  }
}

void dynamicTurn(int direction) {
  if (direction == 1) {
    turnRight();
  } else {
    turnLeft();
  }
  setMotorPower(MOTOR_POWER_SEARCH);

  while (true) {
    handleButtonPresses();
    if (current_mode == MODE_OFF) {
      setMotorPower(0);
      returnToCenter();
      return;
    }

    readLineSensorData();
    
    if (ir_middle_data == 0 && ir_left_data == 1 && ir_right_data == 1) {
      returnToCenter();
      setMotorPower(MOTOR_POWER_FORWARD);
      break;
    }
    delay(50);
  }
}



void readLidarData() {
  while (NanoPiSerial.available()) {
    char inChar = (char)NanoPiSerial.read();
    inputString += inChar;
    
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  
  if (stringComplete) {
    if (inputString.startsWith("LIDAR:")) {
      lidar.lidarConnected = true;
      String data = inputString.substring(6);
      
      int firstComma = data.indexOf(',');
      int secondComma = data.indexOf(',', firstComma + 1);
      int thirdComma = data.indexOf(',', secondComma + 1);
      
      if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
        lidar.front = data.substring(0, firstComma).toFloat();
        lidar.right = data.substring(firstComma + 1, secondComma).toFloat();
        lidar.back = data.substring(secondComma + 1, thirdComma).toFloat();
        lidar.left = data.substring(thirdComma + 1).toFloat();
        lidar.dataReceived = true;
      }
    }
    
    inputString = "";
    stringComplete = false;
  }
}






// Move these constants to global scope so all functions can access them
const int SIDE_SAFE_DISTANCE = 20;
const int SIDE_CRITICAL_DISTANCE = 15;
const int FRONT_OBSTACLE_DISTANCE = 35;
const int FRONT_CLEAR_DISTANCE = 100;

// Keep these as static variables in the main function
void implementLidarObstacleAvoidance() {
  static bool isAvoiding = false;
  static int avoidanceDirection = 0;
  static unsigned long avoidanceStartTime = 0;
  const unsigned long MAX_AVOIDANCE_TIME = 8000;
  
  int front_cm = lidar.front * 100;
  int left_cm = lidar.left * 100;
  int right_cm = lidar.right * 100;
  
  // ===== ALWAYS ACTIVE SIDE PROTECTION =====
  bool sideCorrectionApplied = handleSideCorrection(left_cm, right_cm);
  if (sideCorrectionApplied) {
    return; // Side correction takes priority
  }
  
  if (!isAvoiding) {
    // ===== FRONT OBSTACLE DETECTION =====
    if (front_cm <= FRONT_OBSTACLE_DISTANCE) {
      isAvoiding = true;
      avoidanceStartTime = millis();
      
      moveStop();
      delay(100);
      moveBackward();
      delay(1000);
      moveStop();
      delay(200);
      
      // Choose avoidance direction with safety margin
      if (left_cm > right_cm && left_cm > SIDE_SAFE_DISTANCE + 5) {
        avoidanceDirection = 1;
        turnLeft();
        setMotorPower(500);
      } else if (right_cm > left_cm && right_cm > SIDE_SAFE_DISTANCE + 5) {
        avoidanceDirection = 2;
        turnRight();
        setMotorPower(550);
      } else {
        // Both sides too close, reverse and try again
        avoidanceDirection = 0;
        moveBackward();
        setMotorPower(-MOTOR_POWER_FORWARD);
        delay(2000);
        moveStop();
        isAvoiding = false; // Reset to try again
      }
    } else {
      moveForward();
    }
  } else {
    // ===== CURRENTLY AVOIDING - CHECK MULTIPLE CONDITIONS =====
    
    // 1. Check for critical side obstacles during turning
    if (avoidanceDirection == 1) { 
      // Currently turning LEFT - check RIGHT side (the side we're turning away from)
      if (right_cm > 0 && right_cm < SIDE_CRITICAL_DISTANCE) {
        // Critical obstacle on right while turning left - EMERGENCY STOP & CORRECT
        handleTurningCollisionAvoidance(1, right_cm, avoidanceStartTime);
        return;
      }
    } 
    else if (avoidanceDirection == 2) { 
      // Currently turning RIGHT - check LEFT side (the side we're turning away from)
      if (left_cm > 0 && left_cm < SIDE_CRITICAL_DISTANCE) {
        // Critical obstacle on left while turning right - EMERGENCY STOP & CORRECT
        handleTurningCollisionAvoidance(2, left_cm, avoidanceStartTime);
        return;
      }
    }
    
    // 2. Check for safe side distances during turning
    if (avoidanceDirection == 1 && right_cm > 0 && right_cm < SIDE_SAFE_DISTANCE) {
      // Getting too close to right side while turning left - adjust
      turnSlightlyRight(); // Reduce left turn angle
      setMotorPower(400);
      delay(300);
      turnLeft(); // Resume left turn
      setMotorPower(500);
    }
    else if (avoidanceDirection == 2 && left_cm > 0 && left_cm < SIDE_SAFE_DISTANCE) {
      // Getting too close to left side while turning right - adjust
      turnSlightlyLeft(); // Reduce right turn angle
      setMotorPower(400);
      delay(300);
      turnRight(); // Resume right turn
      setMotorPower(550);
    }
    
    // 3. Primary condition: Front path is clear
    if (front_cm > FRONT_CLEAR_DISTANCE) {
      isAvoiding = false;
      avoidanceDirection = 0;
      returnToCenter();
      setMotorPower(MOTOR_POWER_FORWARD);
    } 
    // 4. Safety timeout
    else if (millis() - avoidanceStartTime > MAX_AVOIDANCE_TIME) {
      isAvoiding = false;
      avoidanceDirection = 0;
      returnToCenter();
      setMotorPower(MOTOR_POWER_FORWARD);
    }
  }
}




// Updated function to accept avoidanceStartTime as a parameter
void handleTurningCollisionAvoidance(int turningDirection, int obstacleDistance, unsigned long &avoidanceStartTime) {
  // Emergency procedure when about to hit something while turning
  
  // 1. Immediate stop
  moveStop();
  delay(100);
  
  // 2. Reverse slightly to create space
  moveBackward();
  setMotorPower(-MOTOR_POWER_FORWARD);
  delay(800);
  moveStop();
  delay(200);
  
  // 3. Re-evaluate the situation
  int front_cm = lidar.front * 100;
  int left_cm = lidar.left * 100;
  int right_cm = lidar.right * 100;
  
  // 4. Choose new strategy based on current distances
  if (turningDirection == 1) { 
    // Was turning LEFT, but right side was blocked
    if (front_cm > 40 && left_cm > SIDE_SAFE_DISTANCE + 10) {
      // Front is clear and left has more space - continue left turn but slower
      turnLeft();
      setMotorPower(400); // Slower power for more control
    } else {
      // Try opposite direction
      turnRight();
      setMotorPower(450);
    }
  } 
  else { 
    // Was turning RIGHT, but left side was blocked
    if (front_cm > 40 && right_cm > SIDE_SAFE_DISTANCE + 10) {
      // Front is clear and right has more space - continue right turn but slower
      turnRight();
      setMotorPower(450); // Slower power for more control
    } else {
      // Try opposite direction
      turnLeft();
      setMotorPower(400);
    }
  }
  
  // Reset avoidance timer to give new strategy time to work
  avoidanceStartTime = millis();
}

bool handleSideCorrection(int left_cm, int right_cm) {
  // Emergency side correction - too close to either side
  if (left_cm > 0 && left_cm < SIDE_CRITICAL_DISTANCE) {
    turnRight();
    setMotorPower(MOTOR_POWER_FORWARD);
    delay(200);
    returnToCenter();
    return true;
  }
  else if (right_cm > 0 && right_cm < SIDE_CRITICAL_DISTANCE) {
    turnLeft();
    setMotorPower(MOTOR_POWER_FORWARD);
    delay(200);
    returnToCenter();
    return true;
  }
  // Gentle side correction - maintaining safe distance
  else if (left_cm > 0 && left_cm < SIDE_SAFE_DISTANCE) {
    turnRight();
    setMotorPower(MOTOR_POWER_FORWARD);
    delay(150);
    returnToCenter();
    return true;
  }
  else if (right_cm > 0 && right_cm < SIDE_SAFE_DISTANCE) {
    turnLeft();
    setMotorPower(MOTOR_POWER_FORWARD);
    delay(150);
    returnToCenter();
    return true;
  }
  
  return false;
}

// Add these helper functions for fine-tuned steering control
void turnSlightlyLeft() {
  const char *log;
  // Less aggressive left turn (half of full left turn)
  int32_t slight_left_value = (start_position_value + left_turn_value) / 2;
  dxl_wb.goalPosition(STIR_SERVO, slight_left_value, &log);
}

void turnSlightlyRight() {
  const char *log;
  // Less aggressive right turn (half of full right turn)
  int32_t slight_right_value = (start_position_value + right_turn_value) / 2;
  dxl_wb.goalPosition(STIR_SERVO, slight_right_value, &log);
}




void moveStop() {
  setMotorPower(0);
  returnToCenter();
}

void moveBackward() {
  setMotorPower(-MOTOR_POWER_FORWARD);
  returnToCenter();
}

void moveForward() {
  returnToCenter();
  setMotorPower(MOTOR_POWER_FORWARD);
}










void implementUltrasonicObstacleAvoidance() {
  if (front_distance > 0 && front_distance < OBSTACLE_DISTANCE_CM) {
    if (front_distance < SIDE_AVOID_DISTANCE_CM) {
      returnToCenter();
      setMotorPower(-MOTOR_POWER_FORWARD);
    } else {
      returnToCenter();
      setMotorPower(0);
    }
  } else if ((LEFT_distance > 0 && LEFT_distance < SIDE_AVOID_DISTANCE_CM) || 
             (RIGHT_distance > 0 && RIGHT_distance < SIDE_AVOID_DISTANCE_CM)) {
    if (LEFT_distance > 0 && LEFT_distance < SIDE_AVOID_DISTANCE_CM) {
      turnRight();
      setMotorPower(MOTOR_POWER_FORWARD);
    } else if (RIGHT_distance > 0 && RIGHT_distance < SIDE_AVOID_DISTANCE_CM) {
      turnLeft();
      setMotorPower(MOTOR_POWER_FORWARD);
    }
  } else {
    implementLineFollowing();
  }
}

void implementLineFollowing() {
  if (ir_middle_data == 0 && ir_left_data == 1 && ir_right_data == 1) {
    returnToCenter();
    setMotorPower(MOTOR_POWER_FORWARD);
  } else if (ir_right_data == 0 && ir_middle_data == 1 && ir_left_data == 1) {
    dynamicTurn(1);
  } else if (ir_left_data == 0 && ir_middle_data == 1 && ir_right_data == 1) {
    dynamicTurn(-1);
  } else if (ir_middle_data == 1 && ir_left_data == 1 && ir_right_data == 1) {
    returnToCenter();
    setMotorPower(MOTOR_POWER_SEARCH);
  } else {
    returnToCenter();
    setMotorPower(MOTOR_POWER_SEARCH);
  }
}

bool checkLidarConnection() {
  static unsigned long lastLidarTime = 0;
  if (lidar.dataReceived) {
    lastLidarTime = millis();
    lidar.dataReceived = false;
    return true;
  }
  if (millis() - lastLidarTime > 2000) {
    return false;
  }
  return lidar.lidarConnected;
}

void setup() {
  const char *log;
  Serial.begin(115200);

  // Initialize Ultrasonic Pins
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT); 
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  // Initialize NanoPi Serial for LIDAR data
  NanoPiSerial.begin(115200);

  // Initialize Dynamixel Workbench
  dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);

  // Ping all required Dynamixels (including new button)
  if (!(dxl_wb.ping(STIR_SERVO, &log)) || !(dxl_wb.ping(MOTOR_ID, &log)) ||
      !(dxl_wb.ping(IR_RIGHT, &log)) || !(dxl_wb.ping(IR_MIDDLE, &log)) ||
      !(dxl_wb.ping(IR_LEFT, &log)) || !(dxl_wb.ping(BTN_1, &log)) ||
      !(dxl_wb.ping(BTN_2, &log))) {
      Serial.println("One or more Dynamixels not found. Halting.");
      while(1);
  }

  // Set the steering servo to Joint Mode and set position limits
  dxl_wb.jointMode(STIR_SERVO, 0, 0, &log);

  start_position_value = dxl_wb.convertRadian2Value(STIR_SERVO, START_POSITION_RADIAN);
  left_turn_value = dxl_wb.convertRadian2Value(STIR_SERVO, LEFT_TURN_RADIAN);
  right_turn_value = dxl_wb.convertRadian2Value(STIR_SERVO, RIGHT_TURN_RADIAN);

  dxl_wb.itemWrite(STIR_SERVO, "Position_Limit_Min", left_turn_value, &log);
  dxl_wb.itemWrite(STIR_SERVO, "Position_Limit_Max", right_turn_value, &log);
  dxl_wb.goalPosition(STIR_SERVO, start_position_value);

  // Configure the DC motor driver
  uint16_t max_power = 1023;
  uint8_t encoder_mode = 1;
  dxl_wb.writeRegister(MOTOR_ID, MAX_POWER_ADDR, (uint16_t)2, (uint8_t*)&max_power, &log);
  dxl_wb.writeRegister(MOTOR_ID, ENCODER_MODE_ADDR, (uint16_t)1, (uint8_t*)&encoder_mode, &log);
  setMotorPower(0);

  // Read the initial encoder value
  int32_t temp_encoder_value;
  dxl_wb.readRegister(MOTOR_ID, ENCODER_ADDR, (uint16_t)2, (uint32_t*)&temp_encoder_value, &log);
  previous_encoder_value = (int16_t)temp_encoder_value;

  Serial.println("=== ROBOT CONTROL SYSTEM READY ===");
  Serial.println("Button Controls:");
  Serial.println("- Button 1 (ID 14): 3 presses for LIDAR Mode (On/Off)");
  Serial.println("- Button 2 (ID 15): 3 presses for Ultrasonic+Line Mode (On/Off)");
  Serial.println("Current mode: OFF");
}

void loop() {
  const long TIMEOUT_MICROSECONDS = 100000;

  // Always handle button presses for mode changes
  handleButtonPresses();

  // Execute based on current mode
  switch(current_mode) {
    case MODE_OFF:
      // Robot is off - do nothing except check buttons
      if (!mode_initialized) {
        setMotorPower(0);
        returnToCenter();
        mode_initialized = true;
      }
      delay(50);
      break;

    case MODE_LIDAR:
      // LIDAR Mode
      if (!mode_initialized) {
        setMotorPower(0);
        returnToCenter();
        mode_initialized = true;
      }
      
      readLidarData();
      if (checkLidarConnection()) {
        implementLidarObstacleAvoidance();
      } else {
        moveStop();
      }
      delay(50);
      break;

    case MODE_ULTRASONIC_LINE:
      // Ultrasonic + Line Following Mode
      if (!mode_initialized) {
        setMotorPower(0);
        returnToCenter();
        mode_initialized = true;
      }
      
      // Read ultrasonic sensors
      digitalWrite(FRONT_TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(FRONT_TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(FRONT_TRIG_PIN, LOW);
      front_duration = pulseIn(FRONT_ECHO_PIN, HIGH, TIMEOUT_MICROSECONDS);
      front_distance = (front_duration > 0) ? (front_duration * 0.034 / 2) : -1;

      digitalWrite(LEFT_TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(LEFT_TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(LEFT_TRIG_PIN, LOW);
      LEFT_duration = pulseIn(LEFT_ECHO_PIN, HIGH, TIMEOUT_MICROSECONDS);
      LEFT_distance = (LEFT_duration > 0) ? (LEFT_duration * 0.034 / 2) : -1;

      digitalWrite(RIGHT_TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(RIGHT_TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(RIGHT_TRIG_PIN, LOW);
      RIGHT_duration = pulseIn(RIGHT_ECHO_PIN, HIGH, TIMEOUT_MICROSECONDS);
      RIGHT_distance = (RIGHT_duration > 0) ? (RIGHT_duration * 0.034 / 2) : -1;
      
      // Read line sensors
      readLineSensorData();

      implementUltrasonicObstacleAvoidance();
      delay(50);
      break;
  }
}