#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>

//const float ALPHA = 0.5; 필터 계수

const int maxSaturation = 300;
const int maxFrontSaturation = 250; // Front Thruster max값
const int maxDiffSaturation = 350; // L R 차이 최대값


Servo TL, TR, TF;  // Thruster

// tx12
int sensorPin[] = {0, 8, 9, 10, 11, 12, 13};
int channel[] = {0, 0, 0, 0, 0, 0, 0};

bool ledState = false;

ros::NodeHandle nh;
int data[] = {0, 0, 0, 0, 0, 0};

void messageCallback(const std_msgs::Int16MultiArray &sub_msg) {
  for (int i = 0; i < 3; i++) {
    data[i] = sub_msg.data[i];
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> sub("control", messageCallback); // 일단 보류

// servo 함수에서는 원하는 회전 각도를 입력하면 그 각도로 회전 (-45deg to 45deg)
void servo(int l = 0, int r = 0, int f = 0) {
  l = constrain(l, -35, 35);
  r = constrain(r, -35, 35);
  f = constrain(f, -35, 35);

}




//void lowPassFilter(int l, int r, int f) {
//  filteredL = ALPHA * l + (1 - ALPHA) * previousFilteredL;
//  filteredR = ALPHA * r + (1 - ALPHA) * previousFilteredR;
//  filteredF = ALPHA * f + (1 - ALPHA) * previousFilteredF;
//
//  previousFilteredL = filteredL;
//  previousFilteredR = filteredR;
//  previousFilteredF = filteredF;
//}


// servo 함수에서는 원하는 PWM값을 입력 (-500 to 500)
void thrust(int l = 0, int r = 0, int f = 0) {
  l = constrain(l, -maxSaturation, maxSaturation);
  r = constrain(r, -maxSaturation, maxSaturation);
  f = constrain(f, -maxFrontSaturation, maxFrontSaturation);

  if (l - r > maxDiffSaturation) {
    int sum = l + r;
    l = sum + maxDiffSaturation / 2;
    r = sum - maxDiffSaturation / 2;
  }

  else if (r - l > maxDiffSaturation) {
    int sum = l + r;
    l = sum - maxDiffSaturation / 2;
    r = sum + maxDiffSaturation / 2;
  }

  l = constrain(l, -maxSaturation, maxSaturation);
  r = constrain(r, -maxSaturation, maxSaturation);
  f = constrain(f, -maxFrontSaturation, maxFrontSaturation);

  //  lowPassFilter(l, r, f);

  //  TL.writeMicroseconds(1500 + filteredL);
  //  TR.writeMicroseconds(1500 + filteredR);
  //  TF.writeMicroseconds(1500 + filteredF);

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
  for (int i = 1; i < 7; i++) {
    channel[i] = pulseIn(sensorPin[i], HIGH) - 1490;
    if (channel[i] < 40 && channel[i] > -40) channel[i] = 0;

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

  nh.initNode();
  nh.subscribe(sub);
}


void loop() {
  tx12();
  int tl, tr, tf;
  bool automode = false;
  if (channel[5] < -100) 
  {
    automode = false;
    tl = int(-channel[2] - channel[1]);
    tr = int(-channel[2] + channel[1]);
    tf = -channel[1];
  } 
  else if (channel[5] > 100) 
  {
    automode = true;
    tl = -data[0], tr = -data[1], tf = data[2];
  }
  else 
  {
    automode = false;
    tl = 0, tr = 0, tf = 0;
  }

  thrust(tl, tr, tf);
  if(automode) 
    nh.spinOnce();
}
