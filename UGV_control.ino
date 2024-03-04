// #include <SBUS.h>
#include <sbus.h> //ch1: throttle, ch2: steer, ch6: arm. ch7: kill


// Define channels to control the car
#define THROTTLE_CHANNEL_INDEX 0
#define STEERING_CHANNEL_INDEX 1
#define ARM_CHANNEL_INDEX 5
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
#define SBUS_THROTTLE_TRIM 1001
// Define the maximum PWM duty cycle for motor control
#define MAX_PWM_DUTY_CYCLE 225
#define MIN_PWM_DUTY_CYCLE 0
#define THROTTLE_TRIM 25 //apprxmly 1/10 duty cycle
#define STEERING_TRIM 12 //apprxmly 1/2 throttle trim

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

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial2);
/* SBUS data */
bfs::SbusData data;

const int delay_time = 1000;
bool arm_flag = false;
bool throttle_flag = false;
bool steering_flag = false;
uint32_t current_speed = 0;
uint8_t rc_flag = 0;
int32_t left_speed = 0;
int32_t right_speed = 0;

bool forward_flag = false;
bool backward_flag = false;
bool hold_flag = false;
bool left_flag = false;
bool right_flag = false;

void forward_drive(uint8_t fw_pwm,int32_t  turn_pwm);
void backward_drive(uint8_t bw_pwm,int32_t  turn_pwm);
void left_drive(uint8_t pwm);
void right_drive(uint8_t pwm);
void stop();
void test_drive();
void throttle();
void steering();
void failsafe_handle();

void setup() {
  Serial.begin(9600);
  // sbus.begin();
  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();

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
}


void loop() {
  // put your main code here, to run repeatedly:
  // test_drive();
  // while(1) {
  //   if (sbus_rx.Read()) {
  //     /* Grab the received data */
  //     data = sbus_rx.data();
  //     /* Display the received data */
  //     for (int8_t i = 0; i < data.NUM_CH; i++) {
  //       Serial.print(data.ch[i]);
  //       Serial.print("\t");
  //     }
  //     /* Display lost frames and failsafe data */
  //     Serial.print(data.lost_frame);
  //     Serial.print("\t");
  //     Serial.println(data.failsafe);
  //     /* Set the SBUS TX data to the received data */
  //     sbus_tx.data(data);
  //     /* Write the data to the servos */
  //     sbus_tx.Write();
  //   }
  // }
  // while(1) {
  //     sbus_rx.Read();
  //     data = sbus_rx.data();
  //     for (int8_t i = 0; i < data.NUM_CH; i++) {
  //       // Serial.print(data.ch[i]);
  //       Serial.print(sbus_rx.data().ch[i]);
  //       Serial.print("\t");
  //     }
  //        /* Display lost frames and failsafe data */
  //     Serial.print(data.lost_frame);
  //     Serial.print("\t");
  //     Serial.println(data.failsafe);
  // }
  while(1) {
    sbus_rx.Read();
    /* Grab the received data */
    // data = sbus_rx.data();
      // for (int8_t i = 0; i < data.NUM_CH; i++) {
      //   Serial.print(data.ch[i]);
      //   Serial.print("\t");
      // }
    if(sbus_rx.data().failsafe == 0) {
      Serial.print("\n");
      Serial.println("Acquired RC signal, ready to arm...");
      // while(!(sbus_rx.data().ch[ARM_CHANNEL_INDEX] > SBUS_ARM_TRIM));
      while(!(sbus_rx.data().ch[ARM_CHANNEL_INDEX] > SBUS_ARM_TRIM)) {
        arm_flag = false;
        sbus_rx.Read();
      }
      Serial.println("Arming...");
      arm_flag = true;
      while(sbus_rx.data().failsafe == 0 && sbus_rx.data().ch[ARM_CHANNEL_INDEX] > SBUS_ARM_TRIM) {
        sbus_rx.Read();
        throttle_steering();
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

void backward_drive(uint8_t bw_pwm,int32_t  turn_pwm) {
  // Serial.print("Moving backward, speed is ");
  // Serial.print(pwm);
  // Serial.print("\n");
  left_speed = constrain(bw_pwm + turn_pwm, -MAX_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
  right_speed = constrain(bw_pwm - turn_pwm, -MAX_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
  if(turn_pwm > 0) {
    Serial.print("Turning right: ");
    Serial.print("\t");
    if(right_speed >= 0) {
      left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
      right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
      analogWrite(FRONT_LEFT_PWM_BW, left_speed);
      analogWrite(FRONT_RIGHT_PWM_BW, right_speed);
      analogWrite(FRONT_LEFT_PWM_FW, 0);
      analogWrite(FRONT_RIGHT_PWM_FW, 0);
      analogWrite(REAR_LEFT_PWM_BW, left_speed);
      analogWrite(REAR_RIGHT_PWM_BW, right_speed);
      analogWrite(REAR_LEFT_PWM_FW, 0);
      analogWrite(REAR_RIGHT_PWM_FW, 0);
    } else {
      left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
      right_speed = constrain(right_speed, -MAX_PWM_DUTY_CYCLE, 0);
      right_speed = abs(right_speed);
      analogWrite(FRONT_LEFT_PWM_BW, left_speed);
      analogWrite(FRONT_RIGHT_PWM_BW, 0);
      analogWrite(FRONT_LEFT_PWM_FW, 0);
      analogWrite(FRONT_RIGHT_PWM_FW, right_speed);
      analogWrite(REAR_LEFT_PWM_BW, left_speed);
      analogWrite(REAR_RIGHT_PWM_BW, 0);
      analogWrite(REAR_LEFT_PWM_FW, 0);
      analogWrite(REAR_RIGHT_PWM_FW, right_speed);
    }
  }
  if(turn_pwm < 0) {
    Serial.print("Turning left: ");
    Serial.print("\t");
    if(left_speed >= 0) {
      left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
      right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
      analogWrite(FRONT_LEFT_PWM_BW, left_speed);
      analogWrite(FRONT_RIGHT_PWM_BW, right_speed);
      analogWrite(FRONT_LEFT_PWM_FW, 0);
      analogWrite(FRONT_RIGHT_PWM_FW, 0);
      analogWrite(REAR_LEFT_PWM_BW, left_speed);
      analogWrite(REAR_RIGHT_PWM_BW, right_speed);
      analogWrite(REAR_LEFT_PWM_FW, 0);
      analogWrite(REAR_RIGHT_PWM_FW, 0);
    } else {
      left_speed = constrain(left_speed, -MAX_PWM_DUTY_CYCLE, 0);
      right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
      left_speed = abs(left_speed);
      analogWrite(FRONT_LEFT_PWM_BW, 0);
      analogWrite(FRONT_RIGHT_PWM_BW, right_speed);
      analogWrite(FRONT_LEFT_PWM_FW, left_speed);
      analogWrite(FRONT_RIGHT_PWM_FW, 0);
      analogWrite(REAR_LEFT_PWM_BW, 0);
      analogWrite(REAR_RIGHT_PWM_BW, right_speed);
      analogWrite(REAR_LEFT_PWM_FW, left_speed);
      analogWrite(REAR_RIGHT_PWM_FW, 0);
    }
  }
  if(turn_pwm == 0) {
    Serial.print("No turning: ");
    Serial.print("\t");
    left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
    right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
    analogWrite(FRONT_LEFT_PWM_BW, left_speed);
    analogWrite(FRONT_RIGHT_PWM_BW, right_speed);
    analogWrite(FRONT_LEFT_PWM_FW, 0);
    analogWrite(FRONT_RIGHT_PWM_FW, 0);
    analogWrite(REAR_LEFT_PWM_BW, left_speed);
    analogWrite(REAR_RIGHT_PWM_BW, right_speed);
    analogWrite(REAR_LEFT_PWM_FW, 0);
    analogWrite(REAR_RIGHT_PWM_FW, 0);
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
      analogWrite(FRONT_LEFT_PWM_BW, 0);
      analogWrite(FRONT_RIGHT_PWM_BW, 0);
      analogWrite(FRONT_LEFT_PWM_FW, left_speed);
      analogWrite(FRONT_RIGHT_PWM_FW, right_speed);
      analogWrite(REAR_LEFT_PWM_BW, 0);
      analogWrite(REAR_RIGHT_PWM_BW, 0);
      analogWrite(REAR_LEFT_PWM_FW, left_speed);
      analogWrite(REAR_RIGHT_PWM_FW, right_speed);
    } else {
      left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
      right_speed = constrain(right_speed, -MAX_PWM_DUTY_CYCLE, 0);
      right_speed = abs(right_speed);
      analogWrite(FRONT_LEFT_PWM_BW, 0);
      analogWrite(FRONT_RIGHT_PWM_BW, right_speed);
      analogWrite(FRONT_LEFT_PWM_FW, left_speed);
      analogWrite(FRONT_RIGHT_PWM_FW, 0);
      analogWrite(REAR_LEFT_PWM_BW, 0);
      analogWrite(REAR_RIGHT_PWM_BW, right_speed);
      analogWrite(REAR_LEFT_PWM_FW, left_speed);
      analogWrite(REAR_RIGHT_PWM_FW, 0);
    }
  }
  if(turn_pwm < 0) {
    Serial.print("Turning left: ");
    Serial.print("\t");
    if(left_speed >= 0) {
      left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
      right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
      analogWrite(FRONT_LEFT_PWM_BW, 0);
      analogWrite(FRONT_RIGHT_PWM_BW, 0);
      analogWrite(FRONT_LEFT_PWM_FW, left_speed);
      analogWrite(FRONT_RIGHT_PWM_FW, right_speed);
      analogWrite(REAR_LEFT_PWM_BW, 0);
      analogWrite(REAR_RIGHT_PWM_BW, 0);
      analogWrite(REAR_LEFT_PWM_FW, left_speed);
      analogWrite(REAR_RIGHT_PWM_FW, right_speed);
    } else {
      left_speed = constrain(left_speed, -MAX_PWM_DUTY_CYCLE, 0);
      right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
      left_speed = abs(left_speed);
      analogWrite(FRONT_LEFT_PWM_BW, left_speed);
      analogWrite(FRONT_RIGHT_PWM_BW, 0);
      analogWrite(FRONT_LEFT_PWM_FW, 0);
      analogWrite(FRONT_RIGHT_PWM_FW, right_speed);
      analogWrite(REAR_LEFT_PWM_BW, left_speed);
      analogWrite(REAR_RIGHT_PWM_BW, 0);
      analogWrite(REAR_LEFT_PWM_FW, 0);
      analogWrite(REAR_RIGHT_PWM_FW, right_speed);
    }
  }
  if(turn_pwm == 0) {
    Serial.print("No turning: ");
    Serial.print("\t");
    left_speed = constrain(left_speed, 0, MAX_PWM_DUTY_CYCLE);
    right_speed = constrain(right_speed, 0, MAX_PWM_DUTY_CYCLE);
    analogWrite(FRONT_LEFT_PWM_BW, 0);
    analogWrite(FRONT_RIGHT_PWM_BW, 0);
    analogWrite(FRONT_LEFT_PWM_FW, left_speed);
    analogWrite(FRONT_RIGHT_PWM_FW, right_speed);
    analogWrite(REAR_LEFT_PWM_BW, 0);
    analogWrite(REAR_RIGHT_PWM_BW, 0);
    analogWrite(REAR_LEFT_PWM_FW, left_speed);
    analogWrite(REAR_RIGHT_PWM_FW, right_speed);
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
    analogWrite(FRONT_LEFT_PWM_BW, 0);
    analogWrite(FRONT_RIGHT_PWM_BW, right_speed);
    analogWrite(FRONT_LEFT_PWM_FW, left_speed);
    analogWrite(FRONT_RIGHT_PWM_FW, 0);
    analogWrite(REAR_LEFT_PWM_BW, 0);
    analogWrite(REAR_RIGHT_PWM_BW, right_speed);
    analogWrite(REAR_LEFT_PWM_FW, left_speed);
    analogWrite(REAR_RIGHT_PWM_FW, 0);   
  } 
  if(turn_pwm < 0) {
    analogWrite(FRONT_LEFT_PWM_BW, left_speed);
    analogWrite(FRONT_RIGHT_PWM_BW, 0);
    analogWrite(FRONT_LEFT_PWM_FW, 0);
    analogWrite(FRONT_RIGHT_PWM_FW, right_speed);
    analogWrite(REAR_LEFT_PWM_BW, left_speed);
    analogWrite(REAR_RIGHT_PWM_BW, 0);
    analogWrite(REAR_LEFT_PWM_FW, 0);
    analogWrite(REAR_RIGHT_PWM_FW, right_speed); 
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

  digitalWrite(FRONT_LEFT_EN_FW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_FW, HIGH);
  digitalWrite(FRONT_LEFT_EN_BW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_BW, HIGH);
  analogWrite(FRONT_LEFT_PWM_BW, 0);
  analogWrite(FRONT_RIGHT_PWM_BW, 0);
  analogWrite(FRONT_LEFT_PWM_FW, 0);
  analogWrite(FRONT_RIGHT_PWM_FW, 0);

  digitalWrite(REAR_LEFT_EN_FW, HIGH);
  digitalWrite(REAR_RIGHT_EN_FW, HIGH);
  digitalWrite(REAR_LEFT_EN_BW, HIGH);
  digitalWrite(REAR_RIGHT_EN_BW, HIGH);
  analogWrite(REAR_LEFT_PWM_BW, 0);
  analogWrite(REAR_RIGHT_PWM_BW, 0);
  analogWrite(REAR_LEFT_PWM_FW, 0);
  analogWrite(REAR_RIGHT_PWM_FW, 0);
}

// Function to control the car's speed proportional to the throttle value received from channel
void throttle_steering() {
  int32_t throttleValue = map(sbus_rx.data().ch[THROTTLE_CHANNEL_INDEX], SBUS_THROTTLE_MIN, SBUS_THROTTLE_MAX, -MAX_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
  int32_t steeringValue = map(sbus_rx.data().ch[STEERING_CHANNEL_INDEX], SBUS_STEERING_MIN, SBUS_STEERING_MAX, -MAX_PWM_DUTY_CYCLE,  MAX_PWM_DUTY_CYCLE);
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

void failsafe_handle() {
  stop();
}