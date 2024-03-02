// #include <SBUS.h>
#include <sbus.h> //ch1: throttle, ch2: steer, ch6: arm. ch7: kill


// Define channels to control the car
#define THROTTLE_CHANNEL_INDEX 0
#define STEERING_CHANNEL_INDEX 1
// Define the maximum and minimum values for the SBUS channels (throttle and steering)
//trim ch1: 1481
//trim ch2: 1503
#define SBUS_THROTTLE_MIN  988
#define SBUS_THROTTLE_MAX  1974
#define SBUS_STEERING_MIN  994
#define SBUS_STEERING_MAX  2012
// Define the maximum PWM duty cycle for motor control
#define MAX_PWM_DUTY_CYCLE 255
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
bool throttle_flag = false;
bool steering_flag = false;
uint32_t current_speed = 0;

void forward_drive(uint8_t pwm);
void backward_drive(uint8_t pwm);
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

  while(1) {
    /* Grab the received data */
    data = sbus_rx.data();
    if(data.failsafe == 0) {
      if(steering_flag == false) {
        throttle();
      }
      if(throttle_flag == false) {
        steering();
      }
    } else if(data.failsafe == 1) {
      failsafe_handle();
    }
  }

}

void backward_drive(uint8_t pwm) {
  Serial.print("Moving backward, speed is ");
  Serial.print(pwm);
  Serial.print("\n");

  digitalWrite(FRONT_LEFT_EN_FW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_FW, HIGH);
  digitalWrite(FRONT_LEFT_EN_BW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_BW, HIGH);
  analogWrite(FRONT_LEFT_PWM_BW, pwm);
  analogWrite(FRONT_RIGHT_PWM_BW, pwm);
  analogWrite(FRONT_LEFT_PWM_FW, 0);
  analogWrite(FRONT_RIGHT_PWM_FW, 0);

  digitalWrite(REAR_LEFT_EN_FW, HIGH);
  digitalWrite(REAR_RIGHT_EN_FW, HIGH);
  digitalWrite(REAR_LEFT_EN_BW, HIGH);
  digitalWrite(REAR_RIGHT_EN_BW, HIGH);
  analogWrite(REAR_LEFT_PWM_BW, pwm);
  analogWrite(REAR_RIGHT_PWM_BW, pwm);
  analogWrite(REAR_LEFT_PWM_FW, 0);
  analogWrite(REAR_RIGHT_PWM_FW, 0);
}

void forward_drive(uint8_t pwm) {
  Serial.print("Moving forward, speed is ");
  Serial.print(pwm);
  Serial.print("\n");

  digitalWrite(FRONT_LEFT_EN_FW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_FW, HIGH);
  digitalWrite(FRONT_LEFT_EN_BW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_BW, HIGH);
  analogWrite(FRONT_LEFT_PWM_BW, 0);
  analogWrite(FRONT_RIGHT_PWM_BW, 0);
  analogWrite(FRONT_LEFT_PWM_FW, pwm);
  analogWrite(FRONT_RIGHT_PWM_FW, pwm);

  digitalWrite(REAR_LEFT_EN_FW, HIGH);
  digitalWrite(REAR_RIGHT_EN_FW, HIGH);
  digitalWrite(REAR_LEFT_EN_BW, HIGH);
  digitalWrite(REAR_RIGHT_EN_BW, HIGH);
  analogWrite(REAR_LEFT_PWM_BW, 0);
  analogWrite(REAR_RIGHT_PWM_BW, 0);
  analogWrite(REAR_LEFT_PWM_FW, pwm);
  analogWrite(REAR_RIGHT_PWM_FW, pwm);
}

void right_drive(uint8_t pwm) {
  Serial.print("Turning right, speed is ");
  Serial.print(pwm);
  Serial.print("\n");

  digitalWrite(FRONT_LEFT_EN_FW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_FW, HIGH);
  digitalWrite(FRONT_LEFT_EN_BW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_BW, HIGH);
  analogWrite(FRONT_LEFT_PWM_BW, 0);
  analogWrite(FRONT_RIGHT_PWM_BW, pwm);
  analogWrite(FRONT_LEFT_PWM_FW, pwm);
  analogWrite(FRONT_RIGHT_PWM_FW, 0);

  digitalWrite(REAR_LEFT_EN_FW, HIGH);
  digitalWrite(REAR_RIGHT_EN_FW, HIGH);
  digitalWrite(REAR_LEFT_EN_BW, HIGH);
  digitalWrite(REAR_RIGHT_EN_BW, HIGH);
  analogWrite(REAR_LEFT_PWM_BW, 0);
  analogWrite(REAR_RIGHT_PWM_BW, pwm);
  analogWrite(REAR_LEFT_PWM_FW, pwm);
  analogWrite(REAR_RIGHT_PWM_FW, 0);
}

void left_drive(uint8_t pwm) {
  Serial.print("Turning left, speed is ");
  Serial.print(pwm);
  Serial.print("\n");

  digitalWrite(FRONT_LEFT_EN_FW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_FW, HIGH);
  digitalWrite(FRONT_LEFT_EN_BW, HIGH);
  digitalWrite(FRONT_RIGHT_EN_BW, HIGH);
  analogWrite(FRONT_LEFT_PWM_BW, pwm);
  analogWrite(FRONT_RIGHT_PWM_BW, 0);
  analogWrite(FRONT_LEFT_PWM_FW, 0);
  analogWrite(FRONT_RIGHT_PWM_FW, pwm);

  digitalWrite(REAR_LEFT_EN_FW, HIGH);
  digitalWrite(REAR_RIGHT_EN_FW, HIGH);
  digitalWrite(REAR_LEFT_EN_BW, HIGH);
  digitalWrite(REAR_RIGHT_EN_BW, HIGH);
  analogWrite(REAR_LEFT_PWM_BW, pwm);
  analogWrite(REAR_RIGHT_PWM_BW, 0);
  analogWrite(REAR_LEFT_PWM_FW, 0);
  analogWrite(REAR_RIGHT_PWM_FW, pwm);
}

void test_drive() {
  Serial.println("Implementing test drive....");
  for(int i = 0; i < 255; i+=10) {
    forward_drive(i);
    delay(delay_time);
  }
  for(int i = 255; i > 0; i-=10) {
    forward_drive(i);
    delay(delay_time);
  }
  for(int i = 0; i < 255; i+=10) {
    backward_drive(i);
    delay(delay_time);
  }
  for(int i = 255; i > 0; i-=10) {
    backward_drive(i);
    delay(delay_time);
  }
}

void stop() {
  Serial.print("Stop the car. ");
  Serial.print("\n");

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
void throttle() {
  int32_t throttleValue = map(data.ch[THROTTLE_CHANNEL_INDEX], SBUS_THROTTLE_MIN, SBUS_THROTTLE_MAX, -MAX_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
  if(throttleValue >= -MAX_PWM_DUTY_CYCLE && throttleValue <= MAX_PWM_DUTY_CYCLE ) {
    if(throttleValue > THROTTLE_TRIM) {
      throttle_flag = true;
      forward_drive(throttleValue);
      current_speed = throttleValue;
    } else if(throttleValue < -THROTTLE_TRIM) {
      throttle_flag = true;
      backward_drive(abs(throttleValue));
      current_speed = throttleValue;
    } else {
      current_speed = 0;
      throttle_flag = false;
    }
  }
}

// Function to control the car's steering based on the value received from channel
void steering() {
    int32_t steeringValue = map(data.ch[STEERING_CHANNEL_INDEX], SBUS_STEERING_MIN, SBUS_STEERING_MAX, -MAX_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
    if(steeringValue >= -MAX_PWM_DUTY_CYCLE && steeringValue <= MAX_PWM_DUTY_CYCLE ) {
    	if(steeringValue > STEERING_TRIM) {
        steering_flag = true;
    		right_drive(steeringValue);
        current_speed = steeringValue;
    	} else if(steeringValue < -STEERING_TRIM) {
        steering_flag = true;
    		left_drive(abs(steeringValue));
        current_speed = steeringValue;
    	} else {
        steering_flag = false;
        current_speed = 0;
    	}
    }
}

void failsafe_handle() {
  if(throttle_flag == true && steering_flag == false) {
    if(current_speed > 0) {
      for(int i = current_speed; i >= 0; i--) {
        forward_drive(i);
        delay(10);
      }
    } else if(current_speed < 0) {
      for(int i = -current_speed; i >= 0; i--) {
        backward_drive(i);
        delay(10);
      }
    }
  }
  if(steering_flag == true && throttle_flag == false) {
    if(current_speed > 0) {
      for(int i = current_speed; i >= 0; i--) {
        right_drive(i);
        delay(10);
      }
    } else if(current_speed < 0) {
      for(int i = -current_speed; i >= 0; i--) {
        left_drive(i);
        delay(10);
      }
    }
  }
  if(steering_flag == false && throttle_flag == false) {
    stop();
  }
}