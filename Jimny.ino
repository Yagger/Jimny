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
#define REMOTEXY_BLUETOOTH_NAME "Jimmy"


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 276 bytes V21 
  { 254,7,0,32,0,0,0,4,0,3,1,0,4,1,0,5,1,0,6,1,
  0,253,0,21,0,0,0,0,31,1,106,200,2,1,0,9,0,2,33,26,
  44,22,0,2,26,31,31,79,78,0,79,70,70,0,67,4,6,22,8,78,
  2,26,2,67,42,6,22,8,78,2,26,2,67,78,6,22,8,78,2,26,
  2,67,4,61,22,8,78,2,26,2,67,79,61,22,8,78,2,26,2,5,
  12,86,83,83,32,2,26,31,131,7,180,40,14,2,17,2,31,67,111,110,
  116,114,111,108,0,9,131,61,180,40,14,2,17,2,31,83,101,116,117,112,
  0,6,12,0,130,4,45,97,132,27,17,4,77,69,12,86,0,2,26,4,
  46,70,12,86,0,2,26,4,13,70,12,86,0,2,26,129,21,52,62,12,
  64,8,84,117,114,110,105,110,103,32,80,73,68,0,4,7,19,94,13,128,
  2,26,129,36,3,34,12,64,8,83,112,101,101,100,0,67,8,164,25,10,
  78,2,26,2,67,39,164,25,10,78,2,26,2,67,71,164,25,10,78,2,
  26,2,131,4,181,40,14,2,17,2,31,67,111,110,116,114,111,108,0,9,
  131,58,181,40,14,2,17,2,31,83,101,116,117,112,0,6 };
  
// this structure defines all the variables and events of your control interface 
struct {
    // input variables
  uint8_t auto_enabled; // =1 if switch ON and =0 if OFF, from 0 to 1
  int8_t turn; // from -100 to 100
  int8_t move; // from -100 to 100
  int8_t const_d; // from 0 to 100
  int8_t const_i; // from 0 to 100
  int8_t const_p; // from 0 to 100
  int8_t set_speed; // from 0 to 100
    // output variables
  float front_left_dist;
  float front_center_dist;
  float front_right_dist;
  float side_left_dist;
  float side_right_dist;
  float const_p_val;
  float const_i_val;
  float const_d_val;
} RemoteXY;   
#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

#define LED_PIN D5
#define LED_COUNT 6
Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


// Create instances
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;

// Moving
int proportional_speed_coef = 1;

// Steering
Servo steer;
const int SERVO = 33;
const int max_left = 1800;
const int straight = 1400;
const int max_right = 1000;
const int collision_distance = 600;
const int reverse_proximity = 190;

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
bool armed = false;
bool reverse = false;
const int max_speed = 250;

// Lights
const int LIGHTS_FRONT = 25;
const int LIGHTS_REAR = 32;

// New addresses (7-bit)
#define ADDR_1 0x30
#define ADDR_2 0x31
#define ADDR_3 0x32
#define ADDR_4 0x33
#define ADDR_5 0x34

uint32_t distanceColor(int mm) {
  // https://colorpicker.dev/#00aeff
  if (mm < 0) mm = 0;
  if (mm > 1200) mm = 1200;
  uint16_t hue = map(mm, 0, 1200, 0, 36400); // (0 to 65535) 0 - 36400 is red to blue range
  return leds.ColorHSV(hue, 255, 255);
}

void overtakeFromLeft(int mm) {
  int v = 400 - map(mm, 0, collision_distance, 0, 400);
  goSteer(straight + v);
}

void overtakeFromRight(int mm) {
  int v = 400 - map(mm, 0, collision_distance, 0, 400);
  goSteer(straight - v);
}

void fullSteamAhead(int wall_left, int wall_right) {
  // Get the diff of wall sensors to get our position between
  // map it to the range of steering value
  // use it to control the steering
  int wall_diff = wall_right - wall_left;
  int steering_value = map(wall_diff, -1200, 1200,max_left, max_right);

  goSteer(steering_value);
}

void stop() {
  analogWrite(EN, 0);
}

void goSteer(int steering_value) {
  steer.writeMicroseconds(steering_value);
  go(max_speed*proportional_speed_coef);
}

void go(int s) {
  if (!armed) return;
  digitalWrite(PH, HIGH);
  analogWrite(EN, s);
}

void goRev(int s) {
  if (!armed) return;
  digitalWrite(PH, LOW);
  analogWrite(EN, s);
}

void reverse_(int steering_value) {
  digitalWrite(LIGHTS_REAR, HIGH);
  stop();
  steer.writeMicroseconds(steering_value);
  delay(300);
  goRev(100);
  delay(600);
}

void reverseLeft() {
  reverse_(max_left);
}

void reverseRight() {
  reverse_(max_right);
}

struct MovingAverage {
  static const int N = 3;
  int buf[N];
  int idx = 0;
  int sum = 0;
  bool filled = false;

  int update(int v) {
    sum -= buf[idx];
    buf[idx] = v;
    sum += v;

    idx++;
    if (idx >= N) { idx = 0; filled = true; }

    return filled ? sum / N : sum / idx;
  }
};
MovingAverage wallRightMA, rightMA, centerMA, leftMA, wallLeftMA;


void setup() {
  Serial.begin(115200);
  RemoteXY_Init ();  // initialization by macros
  EEPROM.begin(RemoteXYEngine.getEepromSize()); // init EEPROM 
  Wire.begin();

  steer.setPeriodHertz(50);
  steer.attach(SERVO);
  steer.writeMicroseconds(straight);

  leds.begin();
  leds.show();
  leds.setBrightness(20);

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
  delay(10);

  // Sensor startup
  digitalWrite(XSHUT_1, HIGH); delay(10); sensor1.init(); sensor1.setAddress(ADDR_1);
  digitalWrite(XSHUT_2, HIGH); delay(10); sensor2.init(); sensor2.setAddress(ADDR_2);
  digitalWrite(XSHUT_3, HIGH); delay(10); sensor3.init(); sensor3.setAddress(ADDR_3);
  digitalWrite(XSHUT_4, HIGH); delay(10); sensor4.init(); sensor4.setAddress(ADDR_4);
  digitalWrite(XSHUT_5, HIGH); delay(10); sensor5.init(); sensor5.setAddress(ADDR_5);

  // Start ranging
  sensor1.startContinuous(5);
  sensor2.startContinuous(5);
  sensor3.startContinuous(5);
  sensor4.startContinuous(5);
  sensor5.startContinuous(5);
}

int wallLeft = 2000;
void loop() {
  RemoteXYEngine.handler ();
  int oldWallLeft = wallLeft;
  int wallRight = wallRightMA.update(sensor1.readRangeContinuousMillimeters());
  int right = rightMA.update(sensor2.readRangeContinuousMillimeters());
  int center = centerMA.update(sensor3.readRangeContinuousMillimeters());
  int left = leftMA.update(sensor4.readRangeContinuousMillimeters());
  wallLeft = wallLeftMA.update(sensor5.readRangeContinuousMillimeters());
  // int wallRight = sensor1.readRangeContinuousMillimeters();
  // int right = sensor2.readRangeContinuousMillimeters();
  // int center = sensor3.readRangeContinuousMillimeters();
  // int left = sensor4.readRangeContinuousMillimeters();
  // wallLeft = sensor5.readRangeContinuousMillimeters();

  // Temp logic to arm vehicle. Cover the left wall sensor.
  if (!armed && oldWallLeft > 100 && wallLeft < 100) {
    armed = !armed;
  }
  if (reverse) {
    digitalWrite(PH, LOW);
  } else {
    digitalWrite(PH, HIGH);
  }

  // Apply colors
  if (armed) {
    leds.setPixelColor(0, leds.ColorHSV(0, 255, 255)); // red
  } else {
    leds.setPixelColor(0, leds.ColorHSV(23200, 255, 255)); // green
  }
  leds.setPixelColor(1, distanceColor(wallLeft));
  leds.setPixelColor(2, distanceColor(left));
  leds.setPixelColor(3, distanceColor(center));
  leds.setPixelColor(4, distanceColor(right));
  leds.setPixelColor(5, distanceColor(wallRight));
  leds.show();

  // Control
  int lCol = left < collision_distance;
  int cCol = center < collision_distance;
  int rCol = right < collision_distance;
  if (center < reverse_proximity) {
    if (wallLeft > wallRight) {
      reverseRight();
    } else {
      reverseLeft();
    }
  } else if (!cCol && !lCol && !rCol) {
    fullSteamAhead(wallLeft, wallRight);
  } else if (lCol && cCol && !rCol) {
    overtakeFromRight((center+left)/2);
  } else if (!lCol && cCol && rCol) {
    overtakeFromLeft((center + right)/2);
  } else if (lCol && !cCol && !rCol) {
    overtakeFromRight(left);
  } else if (!lCol && !cCol && rCol) {
    overtakeFromLeft(right);
  } else {
    if (wallLeft > wallRight) {
      overtakeFromLeft(0);
    } else {
      overtakeFromRight(0);
    }
  }
  
  Serial.printf("WR:%6d  R:%6d  C:%6d  L:%6d  WL:%6d\n", wallRight, right, center, left, wallLeft);
  RemoteXYEngine.delay(30);
}
