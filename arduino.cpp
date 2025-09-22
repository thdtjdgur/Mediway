#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>                // [추가] 서보모터 라이브러리를 불러옵니다.
#include <std_msgs/Int32.h>       // [추가] 파이썬으로부터 정수(Integer) 메시지를 받기 위해 필요합니다.

// 모터 핀 정의
#define ENABLE_A  4
#define IN1_A     52
#define IN2_A     50
#define ENABLE_B  5
#define IN3_B     42
#define IN4_B     40

// 엔코더 핀 정의
#define encoderAPinA 2
#define encoderAPinB 3
#define encoderBPinA 18
#define encoderBPinB 19

// [추가] 서보모터 관련 설정
Servo medicineServos[4];                        // [추가] 4개의 서보모터 객체를 배열로 선언합니다.
const int servoPins[] = {8, 9, 10, 11};         // [추가] 서보모터를 연결할 아두이노 핀 번호를 지정합니다. (예: 8,9,10,11번)

// PID 변수
float velocity = 0;
float Rvelocity = 0;  // 목표 속도 (RPM)
float Lvelocity = 0;
float Kp = 0.4, Ki = 1.0, Kd = 0.08;
float integralA = 0, integralB = 0;
float previousErrorA = 0, previousErrorB = 0;
unsigned long lastTime = 0;

// 엔코더 값
volatile long encoderAPos = 0;
volatile long encoderBPos = 0;
long lastEncoderAPos = 0;
long lastEncoderBPos = 0;

// ROS 설정
ros::NodeHandle nh;
geometry_msgs::Twist encoder_msg;
ros::Publisher pub_encoder("encoder_pulse", &encoder_msg);

// [추가] 파이썬 노드로부터 서보 제어 명령을 받았을 때 호출될 함수(콜백)
void servoCallback(const std_msgs::Int32& msg) {
  int servo_num = msg.data; // 전달받은 숫자 데이터 (1, 2, 3, 4 중 하나)

  // 1에서 4 사이의 유효한 숫자인지 확인
  if (servo_num >= 1 && servo_num <= 4) {
    int servo_index = servo_num - 1; // 배열은 0부터 시작하므로, 인덱스로 변환 (1->0, 2->1, ...)

    medicineServos[servo_index].write(180); // 해당 서보모터를 180도(열린 상태)로 회전
    delay(5000);                            // 5초 동안 대기
    medicineServos[servo_index].write(0);   // 해당 서보모터를 0도(닫힌 상태)로 복귀
  }
}

// PID 제어 함수 (적분 항 제한 + 감쇠 적용)
float pidControl(float target, float current, float Kp, float Ki, float Kd, float &integral, float &previousError, float deltaTime) {
  float error = target - current;

  // 적분 항 제한 추가
  float maxIntegral = 100.0;  // 적절한 값으로 조정
  integral += error * deltaTime;
  integral = constrain(integral, -maxIntegral, maxIntegral); // 적분 항 제한 적용

  // 적분 항 감쇠 적용 (오차가 작아지면 적분 값 줄이기)
  if (abs(error) < 1.0) {  
      integral *= 0.9;  
  }

  float derivative = (error - previousError) / deltaTime;
  previousError = error;

  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

// 모터 실제 동작 제어 함수
void setMotor(int enablePin, int in1Pin, int in2Pin, float speed) {
  if (speed > 0) {//현재 속도 < 목표 속도(속도를 올려야하는 상황)
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {//현재 속도 > 목표 속도(속도를 내려야하는 상황)
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  analogWrite(enablePin, constrain(abs(speed), 0, 255));//abs는 절댓값
}

// 속도(RPM) 계산 함수
float calculateSpeed(long encoderDelta, float deltaTime) {
  const int pulsesPerRevolution = 1612;  // 한 바퀴당 엔코더 펄스 수//26pulse*31(감속기어비-->모터가 달라지면 바꿔야함)*2
  return (encoderDelta / (float)pulsesPerRevolution) * (60.0 / deltaTime);//실제 바퀴달린 모터의 rpm임
}

// 키보드 명령 처리 (new_cmd_vel 구독)
void MOTOR(const geometry_msgs::Twist &msg) {  
  if (msg.linear.x == 1 && msg.linear.y == 1) { //오른쪽 직진일때 
    Rvelocity = msg.angular.x-7;
    Lvelocity = msg.angular.x+1;
  }else if (msg.linear.x == 1 && msg.linear.y == -1) { //왼쪽 직진일때 
    Rvelocity = msg.angular.x+1;
    Lvelocity = msg.angular.x-7;//6
  } 
  
  else if (msg.linear.x == -1 && msg.linear.y == 1) { //오른쪽 후진일때
    Rvelocity = -msg.angular.x+10;
    Lvelocity = -msg.angular.x-1;
  }else if (msg.linear.x == -1 && msg.linear.y == -1) { //왼쪽 후진일때
    Rvelocity = -msg.angular.x-1;
    Lvelocity = -msg.angular.x+10;//9
  }
  
  else if (msg.linear.x == 1) { // w
    Rvelocity = msg.angular.x;
    Lvelocity = msg.angular.x;
  } else if (msg.linear.x == -1) { // x
    Rvelocity = -msg.angular.x;
    Lvelocity = -msg.angular.x;
  } else if (msg.linear.y == -1) { // a
    Rvelocity = msg.angular.y;
    Lvelocity = -msg.angular.y;
  } else if (msg.linear.y == 1) { // d
    Rvelocity = -msg.angular.y;
    Lvelocity = msg.angular.y;
  } else if (msg.linear.x == 0 && msg.linear.y==0) { // s
    Rvelocity = 0;
    Lvelocity = 0;
  }else if (msg.linear.x == msg.linear.y) {
    digitalWrite(IN1_A, LOW);
    digitalWrite(IN2_A, LOW);
    digitalWrite(IN3_B, LOW);
    digitalWrite(IN4_B, LOW);
  } else {  // 정지
    Rvelocity = 0;
    Lvelocity = 0;
  }
}


ros::Subscriber<geometry_msgs::Twist> sub("new_cmd_vel", &MOTOR);
// [추가] /servo_control 토픽을 구독하고, 메시지가 오면 servoCallback 함수를 실행할 Subscriber 선언
ros::Subscriber<std_msgs::Int32> sub_servo("servo_control", &servoCallback);

void setup() {
  pinMode(IN1_A, OUTPUT); pinMode(IN2_A, OUTPUT); pinMode(ENABLE_A, OUTPUT);
  pinMode(IN3_B, OUTPUT); pinMode(IN4_B, OUTPUT); pinMode(ENABLE_B, OUTPUT);

  pinMode(encoderAPinA, INPUT_PULLUP);
  pinMode(encoderAPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderAPinA), doEncoderAA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderAPinB), doEncoderAB, CHANGE);

  pinMode(encoderBPinA, INPUT_PULLUP);
  pinMode(encoderBPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderBPinA), doEncoderBA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPinB), doEncoderBB, CHANGE);

  // [추가] 4개의 서보모터 핀을 활성화하고, 초기 상태(0도, 닫힘)로 설정합니다.
  for (int i = 0; i < 4; i++) {
    medicineServos[i].attach(servoPins[i]);
    medicineServos[i].write(0);
  }

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub_servo); // [추가] 위에서 선언한 서보 Subscriber를 ROS 시스템에 등록합니다.
  nh.advertise(pub_encoder);
  //Serial.begin(115200);
}

void loop() {
  nh.spinOnce();

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;

  if (deltaTime >= 0.1) { // 100ms마다 업데이트
    lastTime = currentTime;

    // 100ms 동안 증가한 엔코더 값 (PID 제어용)
    long encoderDeltaA = encoderAPos - lastEncoderAPos;
    long encoderDeltaB = encoderBPos - lastEncoderBPos;

    // 속도 계산
    float currentSpeedA = calculateSpeed(encoderDeltaA, deltaTime);
    float currentSpeedB = calculateSpeed(encoderDeltaB, deltaTime);

    // PID 제어
    float controlA = pidControl(Rvelocity, currentSpeedA, Kp, Ki, Kd, integralA, previousErrorA, deltaTime);//controlA는 결국 현재 rpm에 추가적으로
    float controlB = pidControl(Lvelocity, currentSpeedB, Kp, Ki, Kd, integralB, previousErrorB, deltaTime);//더하거나 뺄 rpm값임

    // 모터 속도 적용
    setMotor(ENABLE_A, IN1_A, IN2_A, controlA);//목표rpm과 현재rpm의 오차를 이용한 pid제어함수를 통해 나온 값은 양수(목표rpm이 높을때), 음수(목표rpm이 
    setMotor(ENABLE_B, IN3_B, IN4_B, controlB);//낮을때)중 하나이다. 즉 pid제어함수를 목표rpm이 높은지 낮은지를 
    //판단(양수, 음수에 해당하는 값이 controlA에 표시됨)하고 setMotor에 controlA를 전달해서 모터핀에 +입력을 줄지, -입력을 줄지 결정하게됨,controlA은 pwm입력핀에도 동시에 쓰임

    // ROS 메시지 및 출력
    encoder_msg.angular.x = encoderAPos;  // 누적된 R-모터 엔코더 값
    encoder_msg.angular.y = encoderBPos;  // 누적된 L-모터 엔코더 값
    pub_encoder.publish(&encoder_msg);

    //Serial.print("Encoder A: "); Serial.print(encoderAPos);
    //Serial.print("\tEncoder B: "); Serial.println(encoderBPos);

    // PID 제어용 이전 엔코더 값 업데이트 (초기화 X)
    lastEncoderAPos = encoderAPos;
    lastEncoderBPos = encoderBPos;
  }
}

void doEncoderAA() {
  // 엔코더 A 핀 A의 상태에 따라 엔코더 위치 증가/감소
  if (digitalRead(encoderAPinA) == HIGH) {
    encoderAPos += (digitalRead(encoderAPinB) == LOW) ? 1 : -1;
  } else {
    encoderAPos += (digitalRead(encoderAPinB) == HIGH) ? 1 : -1;
  }
}

void doEncoderAB() {
  // 엔코더 A 핀 B의 상태에 따라 엔코더 위치 증가/감소
  if (digitalRead(encoderAPinB) == HIGH) {
    encoderAPos += (digitalRead(encoderAPinA) == HIGH) ? 1 : -1;
  } else {
    encoderAPos += (digitalRead(encoderAPinA) == LOW) ? 1 : -1;
  }
}

// L-ENCODER ISR
void doEncoderBA() {
  // 엔코더 B 핀 A의 상태에 따라 엔코더 위치 증가/감소
  if (digitalRead(encoderBPinA) == HIGH) {
    encoderBPos += (digitalRead(encoderBPinB) == LOW) ? 1 : -1;
  } else {
    encoderBPos += (digitalRead(encoderBPinB) == HIGH) ? 1 : -1;
  }
}

void doEncoderBB() {
  // 엔코더 B 핀 B의 상태에 따라 엔코더 위치 증가/감소
  if (digitalRead(encoderBPinB) == HIGH) {
    encoderBPos += (digitalRead(encoderBPinA) == HIGH) ? 1 : -1;
  } else {
    encoderBPos += (digitalRead(encoderBPinA) == LOW) ? 1 : -1;
  }
}