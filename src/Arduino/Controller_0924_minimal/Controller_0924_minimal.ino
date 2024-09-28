#if defined(ARDUINO) && (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

const float ALPHA = 0.5; // 필터 계수

const int maxSaturation = 300;
const int maxFrontSaturation = 250; // Front Thruster max값
const int maxDiffSaturation = 350; // L R 차이 최대값
const long interval = 500; // LED 깜빡이는 주기 [ms]


const int LEDBright = 50;

Servo TL, TR, TF;  // Thruster 

// tx12
int sensorPin[] = {0, 8, 9, 10, 11, 12, 13};
int channel[] = {0, 0, 0, 0, 0, 0, 0};

unsigned long previousMillis = 0;
bool ledState = false;

ros::NodeHandle nh;
int data[] = {0, 0, 0};

void messageCallback(const std_msgs::Int16MultiArray &sub_msg) {
  for(int i = 0; i < 3; i++) {
    data[i] = sub_msg.data[i];
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> sub("control", messageCallback); // 일단 보류
#define PIN        3 // NeoPixel 핀
#define NUMPIXELS 50 // NeoPixel 수
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// int previousFilteredL = 0;
// int previousFilteredR = 0;
// int previousFilteredF = 0;

// int filteredL = 0;
// int filteredR = 0;
// int filteredF = 0;


// void lowPassFilter(int l, int r, int f) {
//   filteredL = ALPHA * l + (1 - ALPHA) * previousFilteredL;
//   filteredR = ALPHA * r + (1 - ALPHA) * previousFilteredR;
//   filteredF = ALPHA * f + (1 - ALPHA) * previousFilteredF;
  
//   previousFilteredL = filteredL;
//   previousFilteredR = filteredR;
//   previousFilteredF = filteredF;
// }


// servo 함수에서는 원하는 PWM값을 입력 (-500 to 500)
void thrust(int l = 0, int r = 0, int f = 0) {
  l = constrain(l, -maxSaturation, maxSaturation);
  r = constrain(r, -maxSaturation, maxSaturation);
  f = constrain(f, -maxFrontSaturation, maxFrontSaturation);

  if (l - r > maxDiffSaturation){
    int sum = l + r;
    l = sum + maxDiffSaturation / 2;
    r = sum - maxDiffSaturation / 2;
  }

  else if (r - l > maxDiffSaturation){
    int sum = l + r;
    l = sum - maxDiffSaturation / 2;
    r = sum + maxDiffSaturation / 2;
  }
  
  l = constrain(l, -maxSaturation, maxSaturation);
  r = constrain(r, -maxSaturation, maxSaturation);
  f = constrain(f, -maxFrontSaturation, maxFrontSaturation);

  // lowPassFilter(l, r, f);

  // TL.writeMicroseconds(1500 + filteredL); 
  // TR.writeMicroseconds(1500 + filteredR); 
  // TF.writeMicroseconds(1500 + filteredF); 



  TL.writeMicroseconds(1500 + l); 
  TR.writeMicroseconds(1500 + r); 
  TF.writeMicroseconds(1500 + f); 

}

// Servo 객체와 선을 등록하고 기본상태로 초기화
void setting() {

  TL.attach(5);
  TR.attach(6);
  TF.attach(7);

  TL.writeMicroseconds(1500);
  TR.writeMicroseconds(1500);
  TF.writeMicroseconds(1500);
}

void tx12() {
  for(int i = 1; i < 7; i++) {
    channel[i] = pulseIn(sensorPin[i], HIGH) - 1490;
    if(channel[i] < 40 && channel[i] > -40) channel[i] = 0;
    
    if (i == 3) {
      channel[i] = map(channel[i], -500, 500, -35, 35);
    } else {
      channel[i] = constrain(channel[i], -500, 500);
    }
  }
}

void setup() {
  Serial.begin(57600);
  setting();
  Serial.setTimeout(50);
  nh.initNode();
  nh.subscribe(sub);
  pixels.begin(); // NeoPixel 초기화
}


void loop() {
  tx12();
  unsigned long currentMillis = millis();

  int tl, tr, tf;
  bool automode = false;

  if (channel[5] < -100) { // 수동모드
    automode = false;
    tl = int(-channel[2] - channel[1]);
    tr = int(-channel[2] + channel[1]);
    tf = -channel[1];

    // 초록색으로 설정
    for(int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, LEDBright)); // 초록색
    }
    pixels.show(); // LED 색 업데이트
  } 
  else if (channel[5] > 100) { // 자동모드
    automode = true;
    tl = -data[0], tr = -data[1], tf = data[2];

    // 일정 시간 간격으로 LED 깜빡이기
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      ledState = !ledState; // LED 상태 토글

      // 깜빡일 때 노란색으로 설정
      if (ledState) {
        for (int i = 0; i < NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(LEDBright, 0, LEDBright)); // 노란색
        }
      } else { 
        for (int i = 0; i < NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(0, 0, 0)); // LED 끄기
        }
      }
      pixels.show(); // LED 상태 업데이트
    }
  } 
  else { // 정지모드
    automode = false;
    tl = 0, tr = 0, tf = 0;

    // 빨간색으로 설정
    for(int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(LEDBright, 0, 0)); // 빨간색
    }
    pixels.show(); // LED 색 업데이트
  }

  // 모터에 쓰러스터 값 전달
  thrust(tl, tr, tf);

  if(automode) {
    nh.spinOnce(); // 자동모드에서 ROS 메시지 처리
  }
}
