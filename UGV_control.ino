// #include <SBUS.h>
#include <sbus.h> //ch0: throttle, ch1: steer, ch4: mode, ch5: arm. ch6: kill, ch7: check sbus signal
#include <Wire.h>


// Define channels to control the car
#define THROTTLE_CHANNEL_INDEX 0
#define STEERING_CHANNEL_INDEX 1
#define ARM_CHANNEL_INDEX 5
#define MODE_CHANNEL_INDEX 4
#define KILL_CHANNEL_INDEX 6
#define CHECK_CHANNEL_INDEX 7
// Define the maximum and minimum values for the SBUS channels (throttle and steering)
//trim ch1: 1481
//trim ch2: 1503
#define SBUS_ARM_MIN 172
#define SBUS_ARM_MAX 1811
#define SBUS_ARM_TRIM 992
#define SBUS_THROTTLE_MIN  172
#define SBUS_THROTTLE_MAX  1750
#define SBUS_THROTTLE_TRIM 1001
#define SBUS_STEERING_MIN  182
#define SBUS_STEERING_MAX  1811
#define SBUS_MODE_MANUAL 172
#define SBUS_MODE_MISSION 992
#define SBUS_MODE_OFFBOARD 1811
#define SBUS_KILL_DISABLE 172
#define SBUS_KILL_TRIM 992
#define SBUS_KILL_ENABLE 1811
#define SBUS_CHECK_DISABLE 172
#define SBUS_CHECK_TRIM 992
#define SBUS_CHECK_ENABLE 1811

// Define the maximum PWM duty cycle for motor control
#define MAX_PWM_DUTY_CYCLE 225
#define MIN_PWM_DUTY_CYCLE 0
#define THROTTLE_TRIM 5 //apprxmly 1/10 duty cycle
#define STEERING_TRIM 2 //apprxmly 1/2 throttle trim

// define pins to connect to the motor drivers
#define FRONT_RIGHT_PWM_FW 9
#define FRONT_RIGHT_PWM_BW 10
#define FRONT_LEFT_PWM_FW 11
#define FRONT_LEFT_PWM_BW 12
#define FRONT_RIGHT_EN_FW 42
#define FRONT_RIGHT_EN_BW 44
#define FRONT_LEFT_EN_FW 46
#define FRONT_LEFT_EN_BW 48

#define REAR_RIGHT_PWM_FW 5
#define REAR_RIGHT_PWM_BW 6
#define REAR_LEFT_PWM_FW 7
#define REAR_LEFT_PWM_BW 8
#define REAR_RIGHT_EN_FW 43
#define REAR_RIGHT_EN_BW 45
#define REAR_LEFT_EN_FW 47
#define REAR_LEFT_EN_BW 49
#define FORWARD 1
#define BACKWARD -1
#define STOP 0
#define ALARM 4

#define SLAVE_ADDRESS 0x08

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial2);
/* SBUS data */
bfs::SbusData data;

// constants
const int delay_time = 1000;
/*********************************************************scale speed**************************************************************/
const float D_front_to_rear = 46.0; //distance from front wheel to rear wheel
const float r_rear_rotation = 16.0; //the radius of the circle that rear wheels go around when rear wheels making a 0 radius turn 
const float scale_speed = (D_front_to_rear/r_rear_rotation + 1.0) + 0.0; // scale_speed = theoretical_scale_speed + practical_error
/**********************************************************************************************************************************/

// variables
bool arm_flag = false;
bool throttle_flag = false;
bool steering_flag = false;
uint32_t current_speed = 0;
int32_t left_speed = 0;
int32_t right_speed = 0;

bool forward_flag = false;
bool backward_flag = false;
bool hold_flag = false;
bool left_flag = false;
bool right_flag = false;
bool rc_flag = false;

void forward_drive(uint8_t fw_pwm,int32_t  turn_pwm);
void backward_drive(uint8_t bw_pwm,int32_t  turn_pwm);
void left_drive(uint8_t pwm);
void right_drive(uint8_t pwm);
void stop();
void test_drive();
void throttle();
void steering();
void failsafe_handle();
void left_motors(int8_t mode, uint8_t pwm);
void right_motors(int8_t mode, uint8_t pwm);
void Drive();
void RCDrive();
void sbusCheck();

void indicatorRC();
void indicatorI2C();

/*--------------------------------------------I2C Comm-------------------------------*/
void receiveData(int byteCount);
byte receivedData[3];
void sendData();
void I2CDrive();
byte data_to_echo = 0;
uint8_t I2C_throttle_pwm;
uint8_t I2C_steering_pwm;
bool I2C_flag = false;

void setup() {
  Serial.begin(9600);
  // sbus.begin();
  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();

  pinMode(ALARM, OUTPUT);
  pinMode(FRONT_RIGHT_EN_FW, OUTPUT);
  pinMode(FRONT_RIGHT_EN_BW, OUTPUT);
  pinMode(FRONT_LEFT_EN_FW, OUTPUT);
  pinMode(FRONT_LEFT_EN_BW, OUTPUT);
  pinMode(REAR_RIGHT_EN_FW, OUTPUT);
  pinMode(REAR_RIGHT_EN_BW, OUTPUT);
  pinMode(REAR_LEFT_EN_FW, OUTPUT);
  pinMode(REAR_LEFT_EN_BW, OUTPUT);
  digitalWrite(FRONT_LEFT_EN_FW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_FW, HIGH);
  digitalWrite(FRONT_LEFT_EN_BW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_BW, HIGH);
  digitalWrite(REAR_LEFT_EN_FW, HIGH);
  digitalWrite(REAR_RIGHT_EN_FW, HIGH);
  digitalWrite(REAR_LEFT_EN_BW, HIGH);
  digitalWrite(REAR_RIGHT_EN_BW, HIGH);

  //I2C Setup
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.setClock(100000);
  // Wire.onRequest(sendData);
}


void loop() {
  Drive();
}

void backward_drive(uint8_t bw_pwm,int32_t  turn_pwm) {
  // Serial.print("Moving backward, speed is ");
  // Serial.print(pwm);
  // Serial.print("\n");
  left_speed = constrain(bw_pwm + turn_pwm, -MAX_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
  right_speed = constrain(bw_pwm - turn_pwm, -MAX_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
  if(turn_pwm < 0) {
    Serial.print("Turning right: ");
    Serial.print("\t");
    if(right_speed >= 0) {
      left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
      right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
      left_motors(BACKWARD, left_speed);
      right_motors(BACKWARD, right_speed);
    } else {
      left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
      right_speed = constrain(right_speed, -MAX_PWM_DUTY_CYCLE, 0);
      right_speed = abs(right_speed);
      left_motors(BACKWARD, left_speed);
      right_motors(FORWARD, right_speed);
    }
  }
  if(turn_pwm > 0) {
    Serial.print("Turning left: ");
    Serial.print("\t");
    if(left_speed >= 0) {
      left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
      right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
      left_motors(BACKWARD, left_speed);
      right_motors(BACKWARD, right_speed);
    } else {
      left_speed = constrain(left_speed, -MAX_PWM_DUTY_CYCLE, 0);
      right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
      left_speed = abs(left_speed);
      left_motors(FORWARD, left_speed);
      right_motors(BACKWARD, right_speed);
    }
  }
  if(turn_pwm == 0) {
    Serial.print("No turning: ");
    Serial.print("\t");
    left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
    right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
    left_motors(BACKWARD, left_speed);
    right_motors(BACKWARD, right_speed);
  }
  Serial.print("left speed: ");
  Serial.print(left_speed);
  Serial.print("\t");
  Serial.print("right speed: ");
  Serial.print(right_speed);
  Serial.print("\t");
  Serial.print("\n");
}

void forward_drive(uint8_t fw_pwm,int32_t  turn_pwm) {
  // Serial.print("Moving forward, speed is ");
  // Serial.print(fw_pwm);
  // Serial.print("\n");
  left_speed = constrain(fw_pwm + turn_pwm, -MAX_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
  right_speed = constrain(fw_pwm - turn_pwm, -MAX_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
  Serial.print("fw and turn pwm: ");
  Serial.print(fw_pwm);
  Serial.print("\t");
  Serial.print(turn_pwm);
  Serial.print("\t");
  Serial.print("Current speed: ");
  Serial.print(left_speed);
  Serial.print("\t");
  Serial.print(right_speed);
  Serial.print("okokok");
  if(turn_pwm > 0) {
    Serial.print("Turning right: ");
    Serial.print("\t");
    if(right_speed >= 0) {
      left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
      right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
      left_motors(FORWARD, left_speed);
      right_motors(FORWARD, right_speed);
    } else {
      left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
      right_speed = constrain(right_speed, -MAX_PWM_DUTY_CYCLE, 0);
      right_speed = abs(right_speed);
      left_motors(FORWARD, left_speed);
      right_motors(BACKWARD, right_speed);
    }
  }
  if(turn_pwm < 0) {
    Serial.print("Turning left: ");
    Serial.print("\t");
    if(left_speed >= 0) {
      left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
      right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
      left_motors(FORWARD, left_speed);
      right_motors(FORWARD, right_speed);
    } else {
      left_speed = constrain(left_speed, -MAX_PWM_DUTY_CYCLE, 0);
      right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
      left_speed = abs(left_speed);
      left_motors(BACKWARD, left_speed);
      right_motors(FORWARD, right_speed);
    }
  }
  if(turn_pwm == 0) {
    Serial.print("No turning: ");
    Serial.print("\t");
    left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
    right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
    left_motors(FORWARD, left_speed);
    right_motors(FORWARD, right_speed);
  }
  Serial.print("left speed: ");
  Serial.print(left_speed);
  Serial.print("\t");
  Serial.print("right speed: ");
  Serial.print(right_speed);
  Serial.print("\t");
  Serial.print("\n");
}

void hold_drive(int32_t  turn_pwm) {
  // Serial.print("Moving forward, speed is ");
  // Serial.print(fw_pwm);
  // Serial.print("\n");
  Serial.print("Turn pwm is: ");
  Serial.print(turn_pwm);
  left_speed = constrain(abs(turn_pwm), 0, MAX_PWM_DUTY_CYCLE);
  right_speed = constrain(abs(turn_pwm), 0, MAX_PWM_DUTY_CYCLE);
  if(turn_pwm > 0) {
    left_motors(FORWARD, left_speed);
    right_motors(BACKWARD, right_speed);
  } 
  if(turn_pwm < 0) {
    left_motors(BACKWARD, left_speed);
    right_motors(FORWARD, right_speed);
  }
  if(turn_pwm == 0) {
    stop();
  }
  Serial.print("left speed: ");
  Serial.print(left_speed);
  Serial.print("\t");
  Serial.print("right speed: ");
  Serial.print(right_speed);
  Serial.print("\t");
  Serial.print("\n");
}

// void test_drive() {
//   Serial.println("Implementing test drive....");
//   for(int i = 0; i < 255; i+=10) {
//     forward_drive(i);
//     delay(delay_time);
//   }
//   for(int i = 255; i > 0; i-=10) {
//     forward_drive(i);
//     delay(delay_time);
//   }
//   for(int i = 0; i < 255; i+=10) {
//     backward_drive(i);
//     delay(delay_time);
//   }
//   for(int i = 255; i > 0; i-=10) {
//     backward_drive(i);
//     delay(delay_time);
//   }
// }

void stop() {
  // Serial.print("Stop the car. ");
  // Serial.print("\n");

  left_motors(STOP, 0);
  right_motors(STOP, 0);
}

// Function to control the car's speed proportional to the throttle value received from channel
void throttle_steering() {
  int32_t throttleValue = 0;
  int32_t steeringValue = 0;
  if(rc_flag == true && sbus_rx.data().ch[MODE_CHANNEL_INDEX] == SBUS_MODE_MANUAL) {
    throttleValue = map(sbus_rx.data().ch[THROTTLE_CHANNEL_INDEX], SBUS_THROTTLE_MIN, SBUS_THROTTLE_MAX, -MAX_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
    steeringValue = map(sbus_rx.data().ch[STEERING_CHANNEL_INDEX], SBUS_STEERING_MIN, SBUS_STEERING_MAX, -MAX_PWM_DUTY_CYCLE,  MAX_PWM_DUTY_CYCLE);
  }
  if(I2C_flag == true && sbus_rx.data().ch[MODE_CHANNEL_INDEX] == SBUS_MODE_MISSION) {
    throttleValue = map(I2C_throttle_pwm, 0, 255, -MAX_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
    steeringValue = map(I2C_steering_pwm, 0, 255, -MAX_PWM_DUTY_CYCLE,  MAX_PWM_DUTY_CYCLE);
  }
  if(I2C_flag == false && rc_flag == false) {
    throttleValue = 0;
    steeringValue = 0;
  }
  if(throttleValue > THROTTLE_TRIM) {
    throttle_flag = true;
    throttleValue = map(throttleValue,THROTTLE_TRIM,255,0,MAX_PWM_DUTY_CYCLE);
    if(steeringValue > STEERING_TRIM) {
      steering_flag = true;
      steeringValue = map(steeringValue,STEERING_TRIM,255,0,MAX_PWM_DUTY_CYCLE);
    } else if (steeringValue < -STEERING_TRIM) {
      steering_flag = true;
      // steeringValue = map(steeringValue,-STEERING_TRIM,-255,0,-MAX_PWM_DUTY_CYCLE);
      steeringValue = map(steeringValue,-255,-STEERING_TRIM,-MAX_PWM_DUTY_CYCLE,0);
    } else {
      steering_flag = false;
      steeringValue = 0;
    }
    forward_flag = true;
    backward_flag = false;
    Serial.print(steeringValue);
    // Serial.print("Moving forward");
    Serial.print("\t");
    forward_drive(throttleValue, steeringValue);
  } else if(throttleValue < -THROTTLE_TRIM) {
    throttle_flag = true;
    throttleValue = map(abs(throttleValue),THROTTLE_TRIM,255,0,MAX_PWM_DUTY_CYCLE);
    if(steeringValue > STEERING_TRIM) {
      steering_flag = true;
      steeringValue = map(steeringValue,STEERING_TRIM,255,0,MAX_PWM_DUTY_CYCLE);
    } else if (steeringValue < -STEERING_TRIM) {
      steering_flag = true;
      steeringValue = map(steeringValue,-STEERING_TRIM,-255,0,-MAX_PWM_DUTY_CYCLE);
    } else {
      steering_flag = false;
      steeringValue = 0;
    }
    backward_flag = true;
    forward_flag = false;
    Serial.print("Moving backward");
    Serial.print("\t");
    backward_drive(throttleValue, steeringValue);
  } else {
    throttle_flag = false;
    forward_flag = false;
    backward_flag = false;
    if(steeringValue > STEERING_TRIM) {
      steeringValue = map(steeringValue,STEERING_TRIM,255,0,MAX_PWM_DUTY_CYCLE);
    } else if (steeringValue < -STEERING_TRIM) {
      steeringValue = map(steeringValue,-STEERING_TRIM,-255,0,-MAX_PWM_DUTY_CYCLE);
    } else {
      steeringValue = 0;
    }
    throttleValue = 0;
    Serial.print("Hold on");
    Serial.print("\t");
    hold_drive(steeringValue);
  }
}

void left_motors(int8_t mode, uint8_t pwm) {
  switch (mode) {
    case FORWARD:
    digitalWrite(FRONT_LEFT_EN_FW, HIGH);
    digitalWrite(FRONT_LEFT_EN_BW, HIGH);
    digitalWrite(REAR_LEFT_EN_FW, HIGH);
    digitalWrite(REAR_LEFT_EN_BW, HIGH);
    analogWrite(FRONT_LEFT_PWM_BW, 0);
    analogWrite(FRONT_LEFT_PWM_FW, pwm);
    analogWrite(REAR_LEFT_PWM_BW, 0);
    analogWrite(REAR_LEFT_PWM_FW, pwm);
    break;
    case BACKWARD:
    digitalWrite(FRONT_LEFT_EN_FW, HIGH);
    digitalWrite(FRONT_LEFT_EN_BW, HIGH);
    digitalWrite(REAR_LEFT_EN_FW, HIGH);
    digitalWrite(REAR_LEFT_EN_BW, HIGH);
    analogWrite(FRONT_LEFT_PWM_BW, pwm);
    analogWrite(FRONT_LEFT_PWM_FW, 0);
    analogWrite(REAR_LEFT_PWM_BW, pwm);
    analogWrite(REAR_LEFT_PWM_FW, 0);
    break;
    case STOP:
    digitalWrite(FRONT_LEFT_EN_FW, HIGH);
    digitalWrite(FRONT_LEFT_EN_BW, HIGH);
    digitalWrite(REAR_LEFT_EN_FW, HIGH);
    digitalWrite(REAR_LEFT_EN_BW, HIGH);
    analogWrite(FRONT_LEFT_PWM_BW, 0);
    analogWrite(FRONT_LEFT_PWM_FW, 0);
    analogWrite(REAR_LEFT_PWM_BW, 0);
    analogWrite(REAR_LEFT_PWM_FW, 0);
    break;
    default:
    break;
  }
}

void right_motors(int8_t mode, uint8_t pwm) {
  switch (mode) {
    case FORWARD:
    digitalWrite(FRONT_RIGHT_EN_FW, HIGH);
    digitalWrite(FRONT_RIGHT_EN_BW, HIGH);
    digitalWrite(REAR_RIGHT_EN_FW, HIGH);
    digitalWrite(REAR_RIGHT_EN_BW, HIGH);
    analogWrite(FRONT_RIGHT_PWM_BW, 0);
    analogWrite(FRONT_RIGHT_PWM_FW, pwm);
    analogWrite(REAR_RIGHT_PWM_BW, 0);
    analogWrite(REAR_RIGHT_PWM_FW, pwm);
    break;
    case BACKWARD:
    digitalWrite(FRONT_RIGHT_EN_FW, HIGH);
    digitalWrite(FRONT_RIGHT_EN_BW, HIGH);
    digitalWrite(REAR_RIGHT_EN_FW, HIGH);
    digitalWrite(REAR_RIGHT_EN_BW, HIGH);
    analogWrite(FRONT_RIGHT_PWM_BW, pwm);
    analogWrite(FRONT_RIGHT_PWM_FW, 0);
    analogWrite(REAR_RIGHT_PWM_BW, pwm);
    analogWrite(REAR_RIGHT_PWM_FW, 0);
    break;
    case STOP:
    digitalWrite(FRONT_RIGHT_EN_FW, HIGH);
    digitalWrite(FRONT_RIGHT_EN_BW, HIGH);
    digitalWrite(REAR_RIGHT_EN_FW, HIGH);
    digitalWrite(REAR_RIGHT_EN_BW, HIGH);
    analogWrite(FRONT_RIGHT_PWM_BW, 0);
    analogWrite(FRONT_RIGHT_PWM_FW, 0);
    analogWrite(REAR_RIGHT_PWM_BW, 0);
    analogWrite(REAR_RIGHT_PWM_FW, 0);
    break;
    default:
    break;
  }
}

void sbusCheck() {
  while(sbus_rx.data().ch[CHECK_CHANNEL_INDEX] > SBUS_CHECK_TRIM) {
      stop(); // stop the car while debugging the sbus value
      sbus_rx.Read();
      data = sbus_rx.data();
      for (int8_t i = 0; i < data.NUM_CH; i++) {
        // Serial.print(data.ch[i]);
        Serial.print(sbus_rx.data().ch[i]);
        Serial.print("\t");
      }
         /* Display lost frames and failsafe data */
      Serial.print(data.lost_frame);
      Serial.print("\t");
      Serial.println(data.failsafe);
  }
}

void RCDrive() {
  rc_flag = true;
  sbus_rx.Read();
  throttle_steering();
}

void Drive() {
  delay(2000);
  while(1) {
    sbus_rx.Read();
    if(sbus_rx.data().ch[KILL_CHANNEL_INDEX] > SBUS_KILL_TRIM) {
      Serial.println("KILL!");
      stop();
      break;
    }
    if(sbus_rx.data().failsafe == 0 && sbus_rx.Read()==1) {
      Serial.print("\n");
      Serial.println("Acquired RC signal, ready to arm...");
      // while(!(sbus_rx.data().ch[ARM_CHANNEL_INDEX] > SBUS_ARM_TRIM));
      while(!(sbus_rx.data().ch[ARM_CHANNEL_INDEX] > SBUS_ARM_TRIM)) {
        arm_flag = false;
        sbus_rx.Read();
      }
      for(int i = 0; i < 2; i++) {
        digitalWrite(ALARM, HIGH);
        delay(100);
        digitalWrite(ALARM, LOW);
        delay(100);
      }
      Serial.println("Arming...");
      arm_flag = true;
      while(sbus_rx.data().failsafe == 0 && sbus_rx.data().ch[ARM_CHANNEL_INDEX] > SBUS_ARM_TRIM) {
        sbus_rx.Read();
        int rc_mode = sbus_rx.data().ch[MODE_CHANNEL_INDEX];
        int i2c_notify = 0;
        switch (rc_mode) {
          case SBUS_MODE_MANUAL:
            rc_flag = true;
            I2C_flag = false;
            throttle_steering();
            break;
          case SBUS_MODE_MISSION:
            if(I2C_flag == false) {
              Serial.println("Waiting for I2C communication");
              for(int i = 0; i < 1; i++) {
                digitalWrite(ALARM, HIGH);
                delay(50);
                digitalWrite(ALARM, LOW);
                delay(50);
              }
            } else {
                throttle_steering();
            }
            break;
          case SBUS_MODE_OFFBOARD:
            //will have sth else but first use sbusCheck()
            sbusCheck();
            break;
          default:
            break;
        }
        if(sbus_rx.data().ch[KILL_CHANNEL_INDEX] > SBUS_KILL_TRIM) {
          Serial.println("KILL!");
          stop();
          break;
        }
      }
      while(sbus_rx.data().failsafe == 0 && sbus_rx.data().ch[ARM_CHANNEL_INDEX] < SBUS_ARM_TRIM) {
        sbus_rx.Read();
        stop();
        Serial.println("Disarmed");
      }
    } else if(sbus_rx.data().failsafe == 1) {
      Serial.println("Lost RC signal");
      failsafe_handle();
    }
  }
}

void failsafe_handle() {
  stop();
}

/*-----------------------------------------------I2C Comm----------------------------------------------------------------------*/

void receiveData(int byteCount) {
  // I2C_flag = true;
  for(int i = 0; i < byteCount; i++) {
    receivedData[0] = Wire.read();
    receivedData[1] = Wire.read();
    receivedData[2] = Wire.read();
    if(!((receivedData[0] == 255) && (receivedData[1] == 255) && (receivedData[2] == 255))) {
      I2C_flag = receivedData[0] & 0x01;
      I2C_throttle_pwm = receivedData[1];
      I2C_steering_pwm = receivedData[2];
    }
  }
  // Serial.print(I2C_flag);
  // Serial.print("\t");
  // Serial.print(I2C_throttle_pwm);
  // Serial.print("\t");
  // Serial.println(I2C_steering_pwm);
  // Serial.print(receivedData[0]);
  // Serial.print("\t");
  // Serial.println(receivedData[1]);
}
void I2CDrive() {
  sbus_rx.Read();
  throttle_steering();
}



void sendData()
{
  Wire.write(data_to_echo);
}
/////////////////////////////