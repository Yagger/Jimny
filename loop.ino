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

  // Control
  if (isOff()) {
    stop();
  } else {
    int L = left < RemoteXY.collision_distance;
    int C = center < RemoteXY.collision_distance;
    int R = right < RemoteXY.collision_distance;
    if (center < RemoteXY.reverse_proximity) {
      if (wall_left > wall_right) {
        reverseRight();
      } else {
        reverseLeft();
      }
    } else if (!L && !C && !R) { // No obstacles
      fullSteamAhead(wall_left, wall_right);
    } else if (L && !C && R) { //Obstacle at left and right, squeeze from middle
      fullSteamAhead(wall_left, wall_right);
    } else if (L && C && !R) { // Obstacle slight leftish, overtake from right
      fullSteamAhead(wall_left + 300, wall_right - 300);
      //overtakeFromRight((center + left) / 2);
    } else if (L && !C && !R) { // Obstacle far left, overtake from  little left
      fullSteamAhead(wall_left + 200, wall_right - 200);
      //overtakeFromRight(left);
    } else if (!L && C && R) { //Obstacle slight rightish, overtake from little left
      fullSteamAhead(wall_left - 300, wall_right + 300);
      //overtakeFromLeft((center + right) / 2);
    } else if (!L && !C && R) { //Obstacle far right, overtake from little right
      fullSteamAhead(wall_left - 200, wall_right + 200);
      //overtakeFromLeft(right);
    } else if (!L && C && !R) { // Something stright at center
      if (wall_left > wall_right) { // More room at left, overtake from left
        fullSteamAhead(wall_left - 300, wall_right + 300);
        //overtakeFromLeft(center);
      } else { //More room at right, overtake from right
        fullSteamAhead(wall_left + 300, wall_right - 300);
        //overtakeFromRight(center);
      }
    } else if (L && C && R) {
      if (wall_left > wall_right) {
        overtakeFromLeft(center);
      } else {
        overtakeFromRight(center);
      }
    }
  }

  // Serial.printf("WR:%6d  R:%6d  C:%6d  L:%6d  WL:%6d MODE:%6d\n", wall_right, right, center, left, wall_left, RemoteXY.mode);
  Serial.printf("WR:%d,R:%d,C:%d,L:%d,WL:%d\n", wall_right, right, center, left, wall_left);
  RemoteXYEngine.delay(loop_delay);
}
