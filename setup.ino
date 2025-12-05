void setup() {
  Serial.begin(115200);
  RemoteXY_Init();
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, conf);
  EEPROM.end();

  Wire.begin();

  steer.setPeriodHertz(50);
  steer.attach(SERVO);
  steer.writeMicroseconds(straight);

  leds.init();
  leds.setBrightness(50);
  leds.setSpeed(200);
  leds.setMode(FX_MODE_SCAN);
  leds.start();


  // Setup pins
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_4, OUTPUT);
  pinMode(XSHUT_5, OUTPUT);
  pinMode(PH, OUTPUT);
  digitalWrite(PH, HIGH);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);
  pinMode(LIGHTS_FRONT, OUTPUT);
  digitalWrite(LIGHTS_FRONT, HIGH);
  pinMode(LIGHTS_REAR, OUTPUT);
  digitalWrite(LIGHTS_REAR, LOW);

  // 1. Turn all sensors off
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  digitalWrite(XSHUT_4, LOW);
  digitalWrite(XSHUT_5, LOW);
  RemoteXYEngine.delay(10);

  // Sensor startup
  digitalWrite(XSHUT_1, HIGH);
  RemoteXYEngine.delay(10);
  sensor1.init();
  sensor1.setAddress(ADDR_1);
  digitalWrite(XSHUT_2, HIGH);
  RemoteXYEngine.delay(10);
  sensor2.init();
  sensor2.setAddress(ADDR_2);
  digitalWrite(XSHUT_3, HIGH);
  RemoteXYEngine.delay(10);
  sensor3.init();
  sensor3.setAddress(ADDR_3);
  digitalWrite(XSHUT_4, HIGH);
  RemoteXYEngine.delay(10);
  sensor4.init();
  sensor4.setAddress(ADDR_4);
  digitalWrite(XSHUT_5, HIGH);
  RemoteXYEngine.delay(10);
  sensor5.init();
  sensor5.setAddress(ADDR_5);

  // Start ranging
  sensor1.startContinuous(5);
  sensor1avg.begin();
  sensor2.startContinuous(5);
  sensor2avg.begin();
  sensor3.startContinuous(5);
  sensor3avg.begin();
  sensor4.startContinuous(5);
  sensor4avg.begin();
  sensor5.startContinuous(5);
  sensor5avg.begin();

  // PID
  straightPID.Start(0,     // input
                    0,  // current output
                    0);    // setpoint
  straightPID.SetOutputLimits(-400, 400);
  straightPID.SetSampleTime(loop_delay);
}