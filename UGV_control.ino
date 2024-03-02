// #include <SBUS.h>
#include <sbus.h> //ch1: throttle, ch2: steer, ch6: arm. ch7: kill

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

// RC_Receiver receiver(2,3,4,13);

const int delay_time = 1000;
static int minChannel = INT_MAX;
static int maxChannel = INT_MIN;

void forward_drive(uint8_t pwm);
void backward_drive(uint8_t pwm);
void left_drive(uint8_t pwm);
void right_drive(uint8_t pwm);
void stop();
void test_drive();
void throttle(uint8_t pwm);

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
  while(1) {
    if (sbus_rx.Read()) {
      /* Grab the received data */
      data = sbus_rx.data();
      /* Display the received data */
      for (int8_t i = 0; i < data.NUM_CH; i++) {
        Serial.print(data.ch[i]);
        Serial.print("\t");
      }
      /* Display lost frames and failsafe data */
      Serial.print(data.lost_frame);
      Serial.print("\t");
      Serial.println(data.failsafe);
      /* Set the SBUS TX data to the received data */
      sbus_tx.data(data);
      /* Write the data to the servos */
      sbus_tx.Write();
    }
  }
  // while(1) {
  //   if(sbus.getFailsafeStatus() == SBUS_SIGNAL_OK) {
  //     Serial.println("sbus signal is ok, ready to drive, pick up your RC!");
      
  //   }
  // }


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

void throttle() {
  //
}