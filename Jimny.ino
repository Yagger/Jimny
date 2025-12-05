//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_BLE

#include <BLEDevice.h>
#include <EEPROM.h>

// RemoteXY connection settings 
#define REMOTEXY_BLUETOOTH_NAME "Jimny"


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  


uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 482 bytes V21 
  { 254,27,0,5,1,0,0,11,0,1,2,0,3,2,0,5,2,0,7,2,
  0,9,2,0,11,2,0,13,1,0,14,1,0,15,4,0,19,4,0,23,
  4,0,182,1,21,0,0,0,74,105,109,110,121,0,31,1,106,200,1,1,
  31,0,67,13,6,24,8,85,2,26,67,41,6,24,8,85,2,26,67,69,
  6,24,8,85,2,26,67,79,21,24,8,85,2,26,67,4,21,24,8,85,
  2,26,3,31,24,44,16,131,2,26,7,6,48,40,10,85,64,2,26,7,
  57,48,40,10,85,64,2,26,7,6,65,40,10,85,64,2,26,7,56,65,
  40,10,85,64,2,26,7,56,100,40,10,85,64,2,26,7,56,82,40,10,
  85,64,2,26,4,6,137,93,15,128,2,26,129,6,42,39,5,64,25,99,
  111,108,108,105,115,105,111,110,32,100,105,115,116,97,110,99,101,0,129,58,
  42,38,5,64,25,114,101,118,101,114,115,101,32,112,114,111,120,105,109,105,
  116,121,0,129,6,60,25,5,64,25,109,97,120,32,115,112,101,101,100,0,
  129,57,60,34,5,64,25,111,118,101,114,116,97,107,101,32,115,112,101,101,
  100,0,129,57,94,31,5,64,25,114,101,118,101,114,115,101,32,115,112,101,
  101,100,0,129,57,76,28,5,64,25,114,101,118,101,114,115,101,32,116,105,
  109,101,0,129,7,132,67,5,64,25,112,114,111,112,111,114,116,105,111,110,
  97,108,32,115,112,101,101,100,32,99,111,101,102,102,105,99,105,101,110,116,
  0,129,34,18,40,5,64,25,79,70,70,32,32,32,65,85,84,79,32,32,
  77,65,78,0,67,7,183,90,10,68,2,26,251,129,8,176,14,5,64,25,
  100,101,98,117,103,0,4,6,158,93,15,128,2,26,129,8,154,43,5,64,
  25,115,116,101,101,114,105,110,103,32,99,111,101,102,102,105,99,105,101,110,
  116,0,7,6,114,40,10,77,64,2,26,5,7,6,82,40,10,77,64,2,
  26,5,129,6,109,6,5,64,25,107,100,0,129,6,77,6,5,64,25,107,
  112,0,7,6,98,40,10,77,64,2,26,5,129,6,93,4,5,64,25,107,
  105,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t mode; // from 0 to 3
  int16_t collision_distance; // -32768 .. +32767
  int16_t reverse_proximity; // -32768 .. +32767
  int16_t max_speed; // -32768 .. +32767
  int16_t overtake_speed; // -32768 .. +32767
  int16_t reverse_speed; // -32768 .. +32767
  int16_t reverse_time; // -32768 .. +32767
  int8_t proportional_speed_coef; // from 0 to 100
  int8_t steering_coef; // from 0 to 100
  float kd;
  float kp;
  float ki;

    // output variables
  int16_t left; // -32768 .. +32767
  int16_t center; // -32768 .. +32767
  int16_t right; // -32768 .. +32767
  int16_t wall_right; // -32768 .. +32767
  int16_t wall_left; // -32768 .. +32767
  char debug[251]; // string UTF8 end zero

} RemoteXY;   



#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <PID_v2.h>
#include <movingAvg.h>

#define LED_PIN D5
#define LED_COUNT 6
Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


// Create instances
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;

//General
const int loop_delay = 30;

// Steering
Servo steer;
const int SERVO = 33;
const int max_left = 1800;
const int straight = 1400;
const int max_right = 1000;

// XSHUT pins
const int XSHUT_1 = D0;
const int XSHUT_2 = D3;
const int XSHUT_3 = D4;
const int XSHUT_4 = D6;
const int XSHUT_5 = D7;

// Control pins
// Servo throttle;
const int PH = 4;
const int EN = 2;

// Lights
const int LIGHTS_FRONT = 25;
const int LIGHTS_REAR = 32;

// New addresses (7-bit)
#define ADDR_1 0x30
#define ADDR_2 0x31
#define ADDR_3 0x32
#define ADDR_4 0x33
#define ADDR_5 0x34

double Kp = 1, Ki = 0, Kd=0;
PID_v2 straightPID(Kp, Ki, Kd, PID::Direct);

uint32_t distanceColor(int mm) {
  // https://colorpicker.dev/#00aeff
  if (mm < 0) mm = 0;
  if (mm > 1200) mm = 1200;
  uint16_t hue = map(mm, 0, 1200, 0, 36400); // (0 to 65535) 0 - 36400 is red to blue range
  return leds.ColorHSV(hue, 255, 255);
}

void overtakeFromLeft(int mm) {
  int v = 400 - map(mm, 0, RemoteXY.collision_distance, 0, 400);
  goSteer(straight + v);
}

void overtakeFromRight(int mm) {
  int v = 400 - map(mm, 0, RemoteXY.collision_distance, 0, 400);
  goSteer(straight - v);
}

void fullSteamAhead(int wall_left, int wall_right) {
  // Get the diff of wall sensors to get our position between
  // map it to the range of steering value
  // use it to control the steering
  if (wall_right > 1200) wall_right = 1200;
  if (wall_left > 1200) wall_left = 1200;
  int wall_diff = wall_right - wall_left;
  //int steering_value = map(wall_diff, -1200, 1200, max_left, max_right);
  //float steering_coef = map(RemoteXY.steering_coef, 0, 100, 0, 200) / 100;
  const double output = straightPID.Run(wall_diff);
  const int steering_value = 1400 + output;
  sprintf(RemoteXY.debug, "%d|%d|%d", wall_diff, output, steering_value);
  goSteer(steering_value); // * steering_coef);
}

void stop() {
  analogWrite(EN, 0);
}

void goSteer(int steering_value) {
  if (steering_value < max_right) steering_value = max_right;
  if (steering_value > max_left) steering_value = max_left;
  steer.writeMicroseconds(steering_value);
  // int steering_delta = abs(straight - steering_value);
  // RemoteXY.speed_delta = map(steering_delta, 0, 400, 0, 150) * (float)RemoteXY.proportional_speed_coef / 100.0;
  // go(RemoteXY.max_speed - RemoteXY.speed_delta);
  go(RemoteXY.max_speed);
}

void go(int s) {
  if (!isArmed()) {stop();return;}
  digitalWrite(PH, HIGH);
  analogWrite(EN, s);
}

void goRev(int s) {
  if (!isArmed()) {stop();return;}
  digitalWrite(PH, LOW);
  analogWrite(EN, s);
}

void reverse_(int steering_value) {
  digitalWrite(LIGHTS_REAR, HIGH);
  stop();
  steer.writeMicroseconds(steering_value);
  RemoteXYEngine.delay(300);
  goRev(RemoteXY.reverse_speed);
  RemoteXYEngine.delay(600);
  digitalWrite(LIGHTS_REAR, LOW);
}

void reverseLeft() {
  reverse_(max_left);
}

void reverseRight() {
  reverse_(max_right);
}

bool isArmed() {
  return RemoteXY.mode > 0;
}

const int N = 10;
movingAvg wallRightMA(N), rightMA(N), centerMA(N), leftMA(N), wallLeftMA(N);

void setup() {
  Serial.begin(115200);
  RemoteXY_Init();  // initialization by macros
  EEPROM.begin(RemoteXYEngine.getEepromSize()); // init EEPROM 
  Wire.begin();

  steer.setPeriodHertz(50);
  steer.attach(SERVO);
  steer.writeMicroseconds(straight);

  leds.begin();
  leds.show();
  leds.setBrightness(50);

  wallRightMA.begin();
  rightMA.begin();
  centerMA.begin();
  leftMA.begin();
  wallLeftMA.begin();

  // Setup pins
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_4, OUTPUT);
  pinMode(XSHUT_5, OUTPUT);
  pinMode(PH, OUTPUT); digitalWrite(PH, HIGH);
  pinMode(EN, OUTPUT); digitalWrite(EN, LOW);
  pinMode(LIGHTS_FRONT, OUTPUT); digitalWrite(LIGHTS_FRONT, HIGH);
  pinMode(LIGHTS_REAR, OUTPUT); digitalWrite(LIGHTS_REAR, LOW);

  // 1. Turn all sensors off
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  digitalWrite(XSHUT_4, LOW);
  digitalWrite(XSHUT_5, LOW);
  RemoteXYEngine.delay(10);

  // Sensor startup
  digitalWrite(XSHUT_1, HIGH); RemoteXYEngine.delay(10); sensor1.init(); sensor1.setAddress(ADDR_1);
  digitalWrite(XSHUT_2, HIGH); RemoteXYEngine.delay(10); sensor2.init(); sensor2.setAddress(ADDR_2);
  digitalWrite(XSHUT_3, HIGH); RemoteXYEngine.delay(10); sensor3.init(); sensor3.setAddress(ADDR_3);
  digitalWrite(XSHUT_4, HIGH); RemoteXYEngine.delay(10); sensor4.init(); sensor4.setAddress(ADDR_4);
  digitalWrite(XSHUT_5, HIGH); RemoteXYEngine.delay(10); sensor5.init(); sensor5.setAddress(ADDR_5);

  // Start ranging
  sensor1.startContinuous(5);
  sensor2.startContinuous(5);
  sensor3.startContinuous(5);
  sensor4.startContinuous(5);
  sensor5.startContinuous(5);

  // PID
  straightPID.Start(0,    // input
              1400, // current output
              0);   // setpoint
  straightPID.SetOutputLimits(-400, 400);
  straightPID.SetSampleTime(loop_delay);
}

void loop() {
  RemoteXYEngine.handler();
  // Init EEPROM
  // RemoteXY.collision_distance = 600;
  // RemoteXY.reverse_proximity = 190;
  // RemoteXY.max_speed = 250;
  // RemoteXY.overtake_speed = 150;
  // RemoteXY.reverse_speed = 100;
  // RemoteXY.reverse_time = 600;
  // RemoteXY.proportional_speed_coef = 50;
  // RemoteXY.steering_coef = 50;
  // RemoteXY.kp = 1;
  // RemoteXY.ki = 0;
  // RemoteXY.kd = 0;

  // PID
  straightPID.SetTunings(RemoteXY.kp, RemoteXY.ki, RemoteXY.kd);

  wallRightMA.reading(sensor1.readRangeContinuousMillimeters());
  RemoteXY.wall_right = wallRightMA.getAvg();

  rightMA.reading(sensor2.readRangeContinuousMillimeters());
  RemoteXY.right = rightMA.getAvg();

  centerMA.reading(sensor3.readRangeContinuousMillimeters());
  RemoteXY.center = centerMA.getAvg();

  leftMA.reading(sensor4.readRangeContinuousMillimeters());
  RemoteXY.left = leftMA.getAvg();

  wallLeftMA.reading(sensor5.readRangeContinuousMillimeters());
  RemoteXY.wall_left = wallLeftMA.getAvg();

  // Apply colors
  if (RemoteXY.mode == 0) {
    leds.setPixelColor(0, leds.ColorHSV(23200, 255, 255)); // green
  } else if (RemoteXY.mode == 1) {
    leds.setPixelColor(0, leds.ColorHSV(0, 255, 255)); // red
  } else if (RemoteXY.mode == 2) {
    leds.setPixelColor(0, leds.ColorHSV(36400, 255, 255)); // blue
  }
  leds.setPixelColor(1, distanceColor(RemoteXY.wall_left));
  leds.setPixelColor(2, distanceColor(RemoteXY.left));
  leds.setPixelColor(3, distanceColor(RemoteXY.center));
  leds.setPixelColor(4, distanceColor(RemoteXY.right));
  leds.setPixelColor(5, distanceColor(RemoteXY.wall_right));
  leds.show();

  // Control
  int L = RemoteXY.left < RemoteXY.collision_distance;
  int C = RemoteXY.center < RemoteXY.collision_distance;
  int R = RemoteXY.right < RemoteXY.collision_distance;
  if (RemoteXY.center < RemoteXY.reverse_proximity) {
    if (RemoteXY.wall_left > RemoteXY.wall_right) {
      reverseRight();
    } else {
      reverseLeft();
    }
  } else if (!L && !C && !R) {
    fullSteamAhead(RemoteXY.wall_left, RemoteXY.wall_right);
  } else if (L && !C && R) {
    fullSteamAhead(RemoteXY.wall_left, RemoteXY.wall_right);
  } else if (L && C && !R) {
    overtakeFromRight((RemoteXY.center+RemoteXY.left)/2);
  } else if (L && !C && !R) {
    overtakeFromRight(RemoteXY.left);
  } else if (!L && C && R) {
    overtakeFromLeft((RemoteXY.center + RemoteXY.right)/2);
  } else if (!L && !C && R) {
    overtakeFromLeft(RemoteXY.right);
  } else if (!L && C && !R) {
    if (RemoteXY.wall_left > RemoteXY.wall_right) {
      overtakeFromLeft(RemoteXY.center);
    } else {
      overtakeFromRight(RemoteXY.center);
    }
  } else if (L && C && R) {
    if (RemoteXY.wall_left > RemoteXY.wall_right) {
      overtakeFromLeft(RemoteXY.center);
    } else {
      overtakeFromRight(RemoteXY.center);
    }
  }
  
  Serial.printf("WR:%6d  R:%6d  C:%6d  L:%6d  WL:%6d MODE:%6d\n", RemoteXY.wall_right, RemoteXY.right, RemoteXY.center, RemoteXY.left, RemoteXY.wall_left, RemoteXY.mode);
  RemoteXYEngine.delay(loop_delay);
}
