#include <SBUS.h>
#include <limits.h>

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



SBUS sbus(Serial2);

const int delay_time = 1000;
static int minChannel = INT_MAX;
static int maxChannel = INT_MIN;

void forward_drive(uint8_t pwm);
void backward_drive(uint8_t pwm);
void left_drive(uint8_t pwm);
void right_drive(uint8_t pwm);
void stop();
void disable();
void test_drive();
void rc_drive();

void setup() {
  Serial.begin(9600);
  sbus.begin();

  pinMode(FRONT_RIGHT_EN_FW, OUTPUT);
  pinMode(FRONT_RIGHT_EN_BW, OUTPUT);
  pinMode(FRONT_LEFT_EN_FW, OUTPUT);
  pinMode(FRONT_LEFT_EN_BW, OUTPUT);
  pinMode(REAR_RIGHT_EN_FW, OUTPUT);
  pinMode(REAR_RIGHT_EN_BW, OUTPUT);
  pinMode(REAR_LEFT_EN_FW, OUTPUT);
  pinMode(REAR_LEFT_EN_BW, OUTPUT);
}

// This is timer 0, which triggers ever 1ms and processes the incoming SBUS datastream.
ISR(TIMER0_COMPA_vect)
{
  sbus.process();
}

// Scale the S.BUS channel values into the range [0, 255] for use as LED brightness values.
int getChannel(int channel) {
  int value = sbus.getChannel(channel);

  if (value < minChannel) {
    minChannel = value;
  }
  if (value > maxChannel) {
    maxChannel = value;
  }

  float result = value;
  
  result -= minChannel;
  result /= (maxChannel - minChannel);
  result *= 255;

  return (int)result; 
}

void loop() {
  // put your main code here, to run repeatedly:

  // test_drive();

  
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