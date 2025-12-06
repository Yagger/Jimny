void loop() {
  RemoteXYEngine.handler();
  handleRemoteXYUpdate();

  // PID
  if (isOff()) {
    straightPID.SetMode(MANUAL);  // https://github.com/imax9000/Arduino-PID-Library/blob/master/PID_v2.h#L25
  } else {
    straightPID.SetMode(AUTOMATIC);
  }
  straightPID.SetTunings(RemoteXY.kp, RemoteXY.ki, RemoteXY.kd);

  int wall_right = sensor1avg.reading(sensor1.readRangeContinuousMillimeters());
  int right = sensor2avg.reading(sensor2.readRangeContinuousMillimeters());
  int center = sensor3avg.reading(sensor3.readRangeContinuousMillimeters());
  int left = sensor4avg.reading(sensor4.readRangeContinuousMillimeters());
  int wall_left = sensor5avg.reading(sensor5.readRangeContinuousMillimeters());

  RemoteXY.wall_right = wall_right;
  RemoteXY.right = right;
  RemoteXY.center = center;
  RemoteXY.left = left;
  RemoteXY.wall_left = wall_left;

  RemoteXY.wall_right_plot = wall_right;
  RemoteXY.right_plot = right;
  RemoteXY.center_plot = center;
  RemoteXY.left_plot = left;
  RemoteXY.wall_left_plot = wall_left;

  // leds.service();
  // // leds.clear();
  // Apply colors
  if (isOff()) {
    leds.setPixelColor(0, leds.ColorHSV(23200, 255, 255));  // green
  } else if (isAuto()) {
    leds.setPixelColor(0, leds.ColorHSV(0, 255, 255));  // red
  } else if (isMan()) {
    leds.setPixelColor(0, leds.ColorHSV(36400, 255, 255));  // blue
  }
  leds.setPixelColor(1, distanceColor(wall_left));
  leds.setPixelColor(2, distanceColor(left));
  leds.setPixelColor(3, distanceColor(center));
  leds.setPixelColor(4, distanceColor(right));
  leds.setPixelColor(5, distanceColor(wall_right));
  leds.show();

  // Debug which branch we enter with RemoteXY plot
  RemoteXY.no_obstacles = 0;
  RemoteXY.obs_lr = 0;
  RemoteXY.obs_l = 0;
  RemoteXY.obs_r = 0;
  RemoteXY.obs_lc = 0;
  RemoteXY.obs_cr = 0;
  RemoteXY.more_room_left = 0;
  RemoteXY.more_room_right = 0;
  RemoteXY.reverse_left = 0;
  RemoteXY.reverse_right = 0;

  // Control
  if (isOff()) {
    stop();
  } else {
    if (!isRunning) {
      RemoteXYEngine.delay(5000);
      isRunning = true;
    }
    static int ramp_left = 0;
    static int ramp_right = 0;
    int L = left < RemoteXY.collision_distance;
    int C = center < RemoteXY.collision_distance;
    int R = right < RemoteXY.collision_distance;
    if (center < RemoteXY.reverse_proximity) {
      if (wall_left > wall_right) {
        RemoteXY.reverse_right = 1;
        reverseRight();
      } else {
        RemoteXY.reverse_left = 1;
        reverseLeft();
      }
    } else if (!L && !C && !R) { // No obstacles
      steering_offset = 0;
      RemoteXY.no_obstacles = 1;
      ramp_left = ramp_right = 0;
      fullSteamAhead(wall_left, wall_right);
    } else if (L && !C && R) { //Obstacle at left and right, squeeze from middle
      RemoteXY.obs_lr = 1;
      steering_offset = 0;
      ramp_left = ramp_right = 0;
      fullSteamAhead(wall_left, wall_right);
    } else if (L && C && !R) { // Obstacle leftish, overtake from right
      RemoteXY.obs_lc = 1;
      steering_offset = -hard_turn;
      if(ramp_right < 30) ramp_right++;
      if(ramp_left < 30) ramp_left++;
      fullSteamAhead(wall_left - (400 * ramp_right/30), wall_right + (400 * ramp_left/30));
    } else if (L && !C && !R) { // Obstacle far left, overtake from  little right
      RemoteXY.obs_l = 1;
      steering_offset = -slight_turn;
      if(ramp_right < 20) ramp_right++;
      if(ramp_left < 20) ramp_left++;
      fullSteamAhead(wall_left - (300 * ramp_right/20), wall_right + (300 * ramp_left/20));
    } else if (!L && C && R) { //Obstacle slight rightish, overtake from little left
      RemoteXY.obs_cr = 1;
      steering_offset = hard_turn;
      if(ramp_right < 30) ramp_right++;
      if(ramp_left < 30) ramp_left++;
      fullSteamAhead(wall_left + (400 * ramp_left/30), wall_right - (400 * ramp_right/30));
    } else if (!L && !C && R) { //Obstacle far right, overtake from little right
      RemoteXY.obs_r = 1;
      steering_offset = slight_turn;
      if(ramp_right < 20) ramp_right++;
      if(ramp_left < 20) ramp_left++;
      fullSteamAhead(wall_left + (300 * ramp_left/20), wall_right - (300 * ramp_right/20));
    } else { // Obstacle in the middle or everywhere (all three front sensors)
      if (wall_left > wall_right) { // More room at left, overtake from left
        RemoteXY.more_room_left = 1;
        steering_offset = hard_turn;
        if(ramp_right < 30) ramp_right++;
        if(ramp_left < 30) ramp_left++;
        fullSteamAhead(wall_left + (400 * ramp_left/30), wall_right - (400 * ramp_right/30));
      } else { //More room at right, overtake from right
        RemoteXY.more_room_right = 1;
        steering_offset = hard_turn;
        if(ramp_right < 30) ramp_right++;
        if(ramp_left < 30) ramp_left++;
        fullSteamAhead(wall_left - (400 * ramp_right/30), wall_right + (400 * ramp_left/30));
      }
    } 
  }
  // Serial.printf("WR:%6d  R:%6d  C:%6d  L:%6d  WL:%6d MODE:%6d\n", wall_right, right, center, left, wall_left, RemoteXY.mode);
  // Serial.printf("WR:%d,R:%d,C:%d,L:%d,WL:%d\n", wall_right, right, center, left, wall_left);
  RemoteXYEngine.delay(loop_delay);
}
