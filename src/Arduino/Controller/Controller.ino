#if defined(ARDUINO) && (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <Servo.h>                                                         
#include <Adafruit_NeoPixel.h>


////////////////////////////////////////////////////////////////////
// PARAMETERS
const int maxSaturation = 450;
const int maxFrontSaturation = 250; // Front Thruster max값
const int maxDiffSaturation = 350; // L R 차이 최대값

const float LPF_Alpha = 0.5; // 필터 계수
const int LEDBright = 20;

////////////////////////////////////////////////////////////////////
// STATE MANAGE
enum Mode {
    MANUAL,
    AUTONOMOUS,
    STOP
};

Mode CurrentMode = STOP;

////////////////////////////////////////////////////////////////////
// Servo
Servo TL, TR, TF;  // Thruster 

void ServoSetting() {

  TL.attach(5);
  TR.attach(6);
  TF.attach(7);

  TL.writeMicroseconds(1500);
  TR.writeMicroseconds(1500);
  TF.writeMicroseconds(1500);
}

////////////////////////////////////////////////////////////////////
// tx12
int sensorPin[] = {0, 8, 9, 10, 11, 12, 13};
int channel[] = {0, 0, 0, 0, 0, 0, 0};

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

////////////////////////////////////////////////////////////////////
// ROS 
ros::NodeHandle nh;
int data[] = {0, 0, 0};

void messageCallback(const std_msgs::Int16MultiArray &sub_msg) {
  for(int i = 0; i < 3; i++) {
    data[i] = sub_msg.data[i];
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> sub("control", messageCallback);

////////////////////////////////////////////////////////////////////
// LED 
long LEDPeriod = 500; // LED 깜빡이는 주기 [ms]
unsigned long previousMillis = 0;
bool LEDState = false;
int LEDPixelNum = 50;
int LEDPin = 3;
Adafruit_NeoPixel pixels(LEDPixelNum, LEDPin, NEO_RGB + NEO_KHZ800);

void setLEDBrightness(int red, int green, int blue) {
    for (int i = 0; i < LEDPixelNum; i++) {
        pixels.setPixelColor(i, pixels.Color(blue, red, green));
    }
    pixels.show(); // LED 색 업데이트
}

////////////////////////////////////////////////////////////////////
// LPF
int previousFilteredL = 0;
int previousFilteredR = 0;
int previousFilteredF = 0;

int filteredL = 0;
int filteredR = 0;
int filteredF = 0;


void lowPassFilter(int l, int r, int f) {
  filteredL = LPF_Alpha * l + (1 - LPF_Alpha) * previousFilteredL;
  filteredR = LPF_Alpha * r + (1 - LPF_Alpha) * previousFilteredR;
  filteredF = LPF_Alpha * f + (1 - LPF_Alpha) * previousFilteredF;
  
  previousFilteredL = filteredL;
  previousFilteredR = filteredR;
  previousFilteredF = filteredF;
}

////////////////////////////////////////////////////////////////////
// Thuster 원하는 PWM값을 입력 (-500 to 500)
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

  lowPassFilter(l, r, f);

  TL.writeMicroseconds(1500 + filteredL); 
  TR.writeMicroseconds(1500 + filteredR); 
  TF.writeMicroseconds(1500 + filteredF); 
}

////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(57600);
  Serial.setTimeout(50);
  ServoSetting();
  nh.initNode();
  nh.subscribe(sub);
  pixels.begin(); // NeoPixel 초기화
  pinMode(4, OUTPUT);
}


void loop() {
  tx12();
  unsigned long currentMillis = millis();
  int tl, tr, tf;

  if (channel[5] < -100)      CurrentMode = MANUAL;
  else if (channel[5] > 100)  CurrentMode = AUTONOMOUS;
  else                        CurrentMode = STOP;

  if (channel[6] > 90 && CurrentMode == STOP)   digitalWrite(4, HIGH);
  else                                          digitalWrite(4, LOW);

  ////////////////////////////////////////////////////////////////////
  if (CurrentMode == MANUAL) {
    Serial.println("MANUAL");
    tl = int(-channel[2] - channel[1]);
    tr = int(-channel[2] + channel[1]);
    tf = -channel[1];

    setLEDBrightness(0, LEDBright, 0);
  } 
  ////////////////////////////////////////////////////////////////////
  else if (CurrentMode == AUTONOMOUS) {
    Serial.println("AUTO");
    tl = -data[0], tr = -data[1], tf = data[2];

    if (currentMillis - previousMillis >= LEDPeriod) 
    {
      previousMillis = currentMillis;
      LEDState = !LEDState;
    }

    if (LEDState) setLEDBrightness(LEDBright, LEDBright, 0);
    else          setLEDBrightness(0, 0, 0);
  } 
  ////////////////////////////////////////////////////////////////////
  else if(CurrentMode = STOP) {
    Serial.println("STOP");
    tl = 0, tr = 0, tf = 0;
    setLEDBrightness(LEDBright, 0, 0);
  }
  ////////////////////////////////////////////////////////////////////


  thrust(tl, tr, tf);
  if(CurrentMode == AUTONOMOUS) {
    nh.spinOnce();
  }
}