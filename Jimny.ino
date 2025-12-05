//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

struct Conf
{
  int16_t collision_distance;
  int16_t reverse_proximity;
  int16_t max_speed;
  int16_t overtake_speed;
  int16_t reverse_speed;
  int16_t reverse_time;
  int8_t proportional_speed_coef;
  int8_t steering_coef;
  float kp;
  float ki;
  float kd;
};

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


uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 667 bytes V21
  { 254,27,0,82,0,0,0,11,0,1,2,0,3,2,0,5,2,0,7,2,
  0,9,2,0,11,2,0,13,1,0,14,1,0,15,4,0,19,4,0,23,
  4,0,111,2,21,0,0,0,74,105,109,110,121,0,31,1,106,200,2,1,
  0,30,0,67,13,6,24,8,85,2,26,67,41,6,24,8,85,2,26,67,
  69,6,24,8,85,2,26,67,79,21,24,8,85,2,26,67,4,21,24,8,
  85,2,26,3,31,24,44,16,131,2,26,7,6,48,40,10,85,64,2,26,
  7,57,48,40,10,85,64,2,26,7,6,65,40,10,85,64,2,26,7,56,
  65,40,10,85,64,2,26,7,56,100,40,10,85,64,2,26,7,56,82,40,
  10,85,64,2,26,4,6,137,93,15,128,2,26,129,6,42,39,5,64,25,
  99,111,108,108,105,115,105,111,110,32,100,105,115,116,97,110,99,101,0,129,
  58,42,38,5,64,25,114,101,118,101,114,115,101,32,112,114,111,120,105,109,
  105,116,121,0,129,6,60,25,5,64,25,109,97,120,32,115,112,101,101,100,
  0,129,57,60,34,5,64,25,111,118,101,114,116,97,107,101,32,115,112,101,
  101,100,0,129,57,94,31,5,64,25,114,101,118,101,114,115,101,32,115,112,
  101,101,100,0,129,57,76,28,5,64,25,114,101,118,101,114,115,101,32,116,
  105,109,101,0,129,7,132,67,5,64,25,112,114,111,112,111,114,116,105,111,
  110,97,108,32,115,112,101,101,100,32,99,111,101,102,102,105,99,105,101,110,
  116,0,129,34,18,40,5,64,25,79,70,70,32,32,32,65,85,84,79,32,
  32,77,65,78,0,4,6,158,93,15,128,2,26,129,8,154,43,5,64,25,
  115,116,101,101,114,105,110,103,32,99,111,101,102,102,105,99,105,101,110,116,
  0,7,6,114,40,10,77,64,2,26,5,7,6,82,40,10,77,64,2,26,
  5,129,6,109,6,5,64,25,107,100,0,129,6,77,6,5,64,25,107,112,
  0,7,6,98,40,10,77,64,2,26,5,129,6,93,4,5,64,25,107,105,
  0,131,69,186,32,11,2,17,2,31,80,108,111,116,115,0,6,4,0,68,
  0,0,106,64,58,8,94,135,40,31,233,163,177,78,246,105,78,111,32,111,
  98,115,116,97,99,108,101,115,0,76,82,0,76,0,82,0,76,67,0,67,
  82,0,82,111,111,109,32,108,101,102,116,0,82,111,111,109,32,114,105,103,
  104,116,0,82,101,118,32,76,0,82,101,118,32,82,0,131,0,189,34,11,
  2,17,2,31,67,111,110,102,105,103,0,9,68,244,58,118,70,53,8,36,
  135,94,204,233,87,97,108,108,76,101,102,116,0,76,101,102,116,0,67,101,
  110,116,101,114,0,82,105,103,104,116,0,87,97,108,108,82,105,103,104,116,
  0,68,0,122,106,67,51,8,36,135,94,87,97,108,108,32,100,105,102,102,
  0,80,73,68,32,111,117,116,112,117,116,0,83,116,101,101,114,105,110,103,
  32,118,97,108,117,101,0 };
  
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
  float no_obstacles;
  float obs_lr;
  float obs_l;
  float obs_r;
  float obs_lc;
  float obs_cr;
  float more_room_left;
  float more_room_right;
  float reverse_left;
  float reverse_right;
  float wall_left_plot;
  float left_plot;
  float center_plot;
  float right_plot;
  float wall_right_plot;
  float wall_diff;
  float pid_output;
  float steering_value;

} RemoteXY;


#pragma pack(pop)

/////////////////////////////////////////////
//           END RemfoteXY include          //
/////////////////////////////////////////////


#include <Wire.h>
#include <VL53L0X.h>
#include <PID_v2.h>
#include <WS2812FX.h>
#include <ESP32Servo.h>
#include <movingAvg.h>


#define EEPROM_SIZE 1024 * 4

#define LED_PIN D5
#define LED_COUNT 6
WS2812FX leds = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

Conf conf;

// Create instances
VL53L0X sensor1;
movingAvg sensor1avg(6);
VL53L0X sensor2;
movingAvg sensor2avg(6);
VL53L0X sensor3;
movingAvg sensor3avg(6);
VL53L0X sensor4;
movingAvg sensor4avg(6);
VL53L0X sensor5;
movingAvg sensor5avg(6);

//General
const int loop_delay = 30;

// Steering
Servo steer;
const int SERVO = 33;
const int max_left = 1800;
const int straight = 1400;
const int max_right = 1000;
int last_filtered_steering = 1400;

// Speed Ramping
int current_speed = 0;
const int acceleration_step = 15; // How fast to speed up (0-255 range)
const int deceleration_step = 40; // How fast to slow down (braking should be faster)

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

double Kp = 1, Ki = 0, Kd = 0;
PID_v2 straightPID(Kp, Ki, Kd, PID::Direct);

uint32_t distanceColor(int mm) {
  // https://colorpicker.dev/#00aeff
  if (mm < 0) mm = 0;
  if (mm > 1200) mm = 1200;
  uint16_t hue = map(mm, 0, 1200, 0, 36400);  // (0 to 65535) 0 - 36400 is red to blue range
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
  RemoteXY.wall_diff = wall_diff;
  //int steering_value = map(wall_diff, -1200, 1200, max_left, max_right);
  //float steering_coef = map(RemoteXY.steering_coef, 0, 100, 0, 200) / 100;
  const double output = straightPID.Run(wall_diff);
  RemoteXY.pid_output = output;
  const int steering_value = 1400 + output;
  RemoteXY.steering_value = steering_value;


  goSteer(steering_value);  // * steering_coef);
}

void stop() {
  analogWrite(EN, 0);
  current_speed = 0;
}

void goSteer(int steering_value) {
  //low pass filter to smooth out jumpy steering commands (good alpha seems to be somewhere around 0.8)
  float filter_multiplier = (float)conf.steering_coef / 100;
  int filtered_steering = filter_multiplier * steering_value + (1.0 - filter_multiplier) * last_filtered_steering;
  last_filtered_steering = filtered_steering;

  if (filtered_steering < max_right) filtered_steering = max_right;
  if (filtered_steering > max_left) filtered_steering = max_left;
  steer.writeMicroseconds(filtered_steering);
  // int steering_delta = abs(straight - steering_value);
  // RemoteXY.speed_delta = map(steering_delta, 0, 400, 0, 150) * (float)RemoteXY.proportional_speed_coef / 100.0;
  // go(RemoteXY.max_speed - RemoteXY.speed_delta);

  //Turn adaptive speed
  float speed_coef = (float)RemoteXY.proportional_speed_coef/100;
  int max_turn = (max_left - max_right) / 2;
  //Speed reduction on turning betwee 100 and max_speed times speed coef
  int speed_delta = map(abs(filtered_steering - straight), 0, max_turn, 100, RemoteXY.max_speed) * speed_coef;
  go(RemoteXY.max_speed - speed_delta);
}

void go(int s) {
  if (!isAuto()) {stop();return;}

  // Ramp speed towards target 's'
  if (current_speed < s) {
    current_speed += acceleration_step;
    if (current_speed > s) current_speed = s;
  } else if (current_speed > s) {
    current_speed -= deceleration_step;
    if (current_speed < s) current_speed = s;
  }

  digitalWrite(PH, HIGH);
  analogWrite(EN, s);
}

void goRev(int s) {
  if (!isAuto()) {stop();return;}
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

bool isOff() {
  return RemoteXY.mode == 0;
}
bool isAuto() {
  return RemoteXY.mode == 1;
}
bool isMan() {
  return RemoteXY.mode == 2;
}

bool remotexy_initialized = false;
void handleRemoteXYUpdate() {
  if (!remotexy_initialized) {
    RemoteXY.collision_distance = conf.collision_distance;
    RemoteXY.reverse_proximity = conf.reverse_proximity;
    RemoteXY.max_speed = conf.max_speed;
    RemoteXY.overtake_speed = conf.overtake_speed;
    RemoteXY.reverse_speed = conf.reverse_speed;
    RemoteXY.reverse_time = conf.reverse_time;
    RemoteXY.proportional_speed_coef = conf.proportional_speed_coef;
    RemoteXY.steering_coef = conf.steering_coef;
    RemoteXY.kp = conf.kp;
    RemoteXY.ki = conf.ki;
    RemoteXY.kd = 0.1; conf.kd;
    remotexy_initialized = true;
  }

  bool anything_changed = false;
  if (RemoteXY.collision_distance != conf.collision_distance) {
    conf.collision_distance = RemoteXY.collision_distance;
    anything_changed = true;
  }
  if (RemoteXY.reverse_proximity != conf.reverse_proximity) {
    conf.reverse_proximity = RemoteXY.reverse_proximity;
    anything_changed = true;
  }
  if (RemoteXY.max_speed != conf.max_speed) {
    conf.max_speed = RemoteXY.max_speed;
    anything_changed = true;
  }
  if (RemoteXY.overtake_speed != conf.overtake_speed) {
    conf.overtake_speed = RemoteXY.overtake_speed;
    anything_changed = true;
  }
  if (RemoteXY.reverse_speed != conf.reverse_speed) {
    conf.reverse_speed = RemoteXY.reverse_speed;
    anything_changed = true;
  }
  if (RemoteXY.reverse_time != conf.reverse_time) {
    conf.reverse_time = RemoteXY.reverse_time;
    anything_changed = true;
  }
  if (RemoteXY.proportional_speed_coef != conf.proportional_speed_coef) {
    conf.proportional_speed_coef = RemoteXY.proportional_speed_coef;
    anything_changed = true;
  }
  if (RemoteXY.steering_coef != conf.steering_coef) {
    conf.steering_coef = RemoteXY.steering_coef;
    anything_changed = true;
  }
  if (RemoteXY.kp != conf.kp) {
    conf.kp = RemoteXY.kp;
    anything_changed = true;
  }
  if (RemoteXY.ki != conf.ki) {
    conf.ki = RemoteXY.ki;
    anything_changed = true;
  }
  if (RemoteXY.kd != conf.kd) {
    conf.kd = RemoteXY.kd;
    anything_changed = true;
  }
  if (anything_changed) {
    Serial.println("Saving to EEPROM");
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(0, conf);
    EEPROM.end();
  }
}