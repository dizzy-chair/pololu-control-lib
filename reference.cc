#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

OLED display;
Buzzer buzzer;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
LineSensors lineSensors;
BumpSensors bumpSensors;
Motors motors;
Encoders encoders;

uint16_t sensorValues[5];
uint16_t muestras = 100;
uint16_t prediction = 0;
bool flag = false;

int error = 0, d_error = 0, prev_error = 0;

float kp_straight = 0.07;
float kd_straight = 0.15;
float kp_curve    = 0.20;
float kd_curve    = 0.10;

float ka_straight = 0.05;
float ka_curve    = 0.25;

float ctrl_speed = 0;
int left_speed = 0, right_speed = 0;

int min_speed  = 40;
int base_speed = 200;  // raise this value only to go faster
int max_speed  = 400;

void setup() {
  display.clear();
  display.setLayout21x8();
  calibrate();
}

void loop() {
  if (buttonA.getSingleDebouncedPress()) calibrate();

  if (buttonB.getSingleDebouncedPress()) {
    flag = true;
    display.clear();
    display.print("Running...");
  }

  prediction = lineSensors.readLineWhite(sensorValues);

  if (flag) {
    error   = prediction - 2000;
    d_error = error - prev_error;

    int sprint_bonus   = 0;
    int adjusted_speed = 0;

    if (abs(error) < 300) {
      ctrl_speed     = (kp_straight * error) + (kd_straight * d_error);
      ctrl_speed     = constrain(ctrl_speed, -150, 150);
      sprint_bonus   = max(0, 120 - (int)(abs(error) * 0.8));
      adjusted_speed = base_speed - (int)(abs(error) * ka_straight);
    } else {
      ctrl_speed     = (kp_curve * error) + (kd_curve * d_error);
      ctrl_speed     = constrain(ctrl_speed, -400, 400);
      sprint_bonus   = 0;
      adjusted_speed = base_speed - (int)(abs(error) * ka_curve);
    }

    if (sprint_bonus > 0) adjusted_speed += sprint_bonus;
    adjusted_speed = constrain(adjusted_speed, min_speed, max_speed);

    left_speed  = constrain(adjusted_speed + (int)ctrl_speed, -400, 400);
    right_speed = constrain(adjusted_speed - (int)ctrl_speed, -400, 400);

    motors.setSpeeds(left_speed, right_speed);
    prev_error = error;

  } else {
    display.gotoXY(0, 0);
    display.print("DN1:"); display.print(sensorValues[0]); display.print(" ");
    display.gotoXY(0, 1);
    display.print("DN2:"); display.print(sensorValues[1]); display.print(" ");
    display.gotoXY(0, 2);
    display.print("DN3:"); display.print(sensorValues[2]); display.print(" ");
    display.gotoXY(0, 3);
    display.print("DN4:"); display.print(sensorValues[3]); display.print(" ");
    display.gotoXY(0, 4);
    display.print("DN5:"); display.print(sensorValues[4]); display.print(" ");
    display.gotoXY(0, 5);
    display.print("Pred:"); display.print(prediction); display.print(" ");
    display.gotoXY(0, 6);
    display.print("Err:"); display.print(error); display.print(" ");
    delay(50);
  }
}

void calibrate() {
  delay(2000);
  for (uint16_t i = 0; i < muestras; i++) {
    display.gotoXY(0, 6);
    display.print(i);
    if (i > muestras / 4 && i <= 3 * muestras / 4) motors.setSpeeds(-60, 60);
    else motors.setSpeeds(60, -60);
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
  display.clear();
}
