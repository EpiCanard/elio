#include "LSM6DS3.h"
#include "Wire.h"
#include <bluefruit.h>
#include <math.h>

BLEClientUart robot_uart;

LSM6DS3 myIMU(I2C_MODE, 0x6A);

void send_drive_command(float angle, float speed, bool force);

const int CALIBRATE_PIN = 10;
const int STOP_PIN = 8;

const float RADIANS_TO_DEGREES = 57.2957795f;
const uint16_t CALIBRATION_SAMPLES = 200;

// A finger does not move through large angles. Lower FULL_* values make the
// controller reach max command with a smaller tilt.
const float SPEED_DEADZONE_DEG = 3.0f;
const float SPEED_FULL_SCALE_DEG = 24.0f;
const float SPEED_DIRECTION = 1.0f;
const float STEERING_DEADZONE_DEG = 8.0f;
const float STEERING_FULL_SCALE_DEG = 45.0f;
const float MAX_STEERING_COMMAND = 90.0f;

const float SPEED_SMOOTHING = 0.22f;
const float STEERING_SMOOTHING = 0.25f;
const uint16_t SEND_INTERVAL_MS = 50;
const uint16_t DEBUG_INTERVAL_MS = 200;

float speed_tilt_bias = 0.0f;
float roll_bias = 0.0f;
float filtered_speed = 0.0f;
float filtered_angle = 0.0f;
float last_sent_speed = 999.0f;
float last_sent_angle = 999.0f;
uint32_t last_send_ms = 0;
uint32_t last_debug_ms = 0;
bool stop_latched = false;

static float clamp_float(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }

  if (value > maximum) {
    return maximum;
  }

  return value;
}

static float apply_deadzone_scaled(float value, float deadzone, float full_scale, float output_max) {
  float magnitude = fabsf(value);

  if (magnitude < deadzone) {
    return 0.0f;
  }

  float normalized = (magnitude - deadzone) / (full_scale - deadzone);
  normalized = clamp_float(normalized, 0.0f, 1.0f);

  if (value < 0.0f) {
    return -normalized * output_max;
  }

  return normalized * output_max;
}

static float angle_delta_degrees(float angle, float reference) {
  float delta = angle - reference;

  while (delta > 180.0f) {
    delta -= 360.0f;
  }

  while (delta < -180.0f) {
    delta += 360.0f;
  }

  return delta;
}

static float read_accel_speed_tilt_degrees() {
  float accel_x = myIMU.readFloatAccelX();
  float accel_y = myIMU.readFloatAccelY();
  float accel_z = myIMU.readFloatAccelZ();
  (void) accel_x;
  return atan2f(accel_y, accel_z) * RADIANS_TO_DEGREES;
}

static float read_accel_roll_degrees() {
  float accel_x = myIMU.readFloatAccelX();
  float accel_z = myIMU.readFloatAccelZ();
  return atan2f(accel_x, accel_z) * RADIANS_TO_DEGREES;
}

void calibrate_neutral() {
  float speed_tilt_sum = 0.0f;
  float roll_sum = 0.0f;

  digitalWrite(LED_BLUE, LOW);

  for (uint16_t sample = 0; sample < CALIBRATION_SAMPLES; ++sample) {
    speed_tilt_sum += read_accel_speed_tilt_degrees();
    roll_sum += read_accel_roll_degrees();
    delay(5);
  }

  speed_tilt_bias = speed_tilt_sum / CALIBRATION_SAMPLES;
  roll_bias = roll_sum / CALIBRATION_SAMPLES;
  filtered_speed = 0.0f;
  filtered_angle = 0.0f;

  digitalWrite(LED_BLUE, HIGH);

  Serial.printf(
    "Calibration complete: speed_tilt_bias=%.2f roll_bias=%.2f\n",
    speed_tilt_bias,
    roll_bias
  );
}

void scan_callback(ble_gap_evt_adv_report_t* report) {
  if (Bluefruit.Scanner.checkReportForService(report, robot_uart)) {
    Serial.println("Nordic UART device found, connecting...");
    Bluefruit.Central.connect(report);
    return;
  }

  Bluefruit.Scanner.resume();
}

void connect_callback(uint16_t conn_handle) {
  Serial.println("Connected, discovering UART service...");

  if (!robot_uart.discover(conn_handle)) {
    Serial.println("UART service not found, disconnecting");
    Bluefruit.disconnect(conn_handle);
    return;
  }

  robot_uart.enableTXD();
  Serial.println("UART ready");
  send_drive_command(0.0f, 0.0f, true);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected, scanning again");
}

void start_scan() {
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.filterUuid(robot_uart.uuid);
  Bluefruit.Scanner.start(0);
}

void setup_bluetooth() {
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);
  Bluefruit.setName("ElioRing");

  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  robot_uart.begin();
  start_scan();
}

void read_robot_responses() {
  while (robot_uart.available()) {
    Serial.write(robot_uart.read());
  }
}

void send_drive_command(float angle, float speed, bool force) {
  if (!Bluefruit.connected() || !robot_uart.discovered()) {
    return;
  }

  uint32_t now = millis();
  bool send_due = now - last_send_ms >= SEND_INTERVAL_MS;

  if (!force && !send_due) {
    return;
  }

  int command_angle = (int)lroundf(clamp_float(angle, -90.0f, 90.0f));
  int command_speed = (int)lroundf(clamp_float(speed, -100.0f, 100.0f));

  robot_uart.printf("%d %d\n", command_angle, command_speed);
  last_sent_angle = command_angle;
  last_sent_speed = command_speed;
  last_send_ms = now;

  Serial.printf("sent: angle=%d speed=%d\n", command_angle, command_speed);
}

void print_motion_debug(float speed_tilt, float roll, float angle, float speed) {
  uint32_t now = millis();

  if (now - last_debug_ms < DEBUG_INTERVAL_MS) {
    return;
  }

  last_debug_ms = now;
  Serial.printf(
    "motion: speed_tilt=%.1f roll=%.1f angle=%.1f speed=%.1f stop_pin=%d calibrate_pin=%d\n",
    speed_tilt,
    roll,
    angle,
    speed,
    digitalRead(STOP_PIN),
    digitalRead(CALIBRATE_PIN)
  );
}

void setup() {
  Serial.begin(115200);

  pinMode(CALIBRATE_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  myIMU.settings.accelSampleRate = 104;
  myIMU.settings.accelRange = 2;
  myIMU.settings.gyroSampleRate = 104;
  myIMU.settings.gyroRange = 500;

  if (myIMU.begin() != 0) {
    Serial.println("IMU init failed");
    digitalWrite(LED_RED, LOW);
    while (true) {
      delay(100);
    }
  }

  calibrate_neutral();
  setup_bluetooth();
}

void loop() {
  if (!Bluefruit.connected()) {
    delay(20);
    return;
  }

  read_robot_responses();

  if (digitalRead(CALIBRATE_PIN) == LOW) {
    calibrate_neutral();
    send_drive_command(0.0f, 0.0f, true);
    delay(250);
    return;
  }

  if (digitalRead(STOP_PIN) == LOW) {
    send_drive_command(0.0f, 0.0f, true);
    stop_latched = true;
    delay(20);
    return;
  }

  if (stop_latched) {
    stop_latched = false;
    filtered_speed = 0.0f;
    filtered_angle = 0.0f;
  }

  float speed_tilt = angle_delta_degrees(read_accel_speed_tilt_degrees(), speed_tilt_bias) * SPEED_DIRECTION;
  float roll = read_accel_roll_degrees() - roll_bias;

  float target_speed = apply_deadzone_scaled(speed_tilt, SPEED_DEADZONE_DEG, SPEED_FULL_SCALE_DEG, 100.0f);
  float target_angle = apply_deadzone_scaled(
    roll,
    STEERING_DEADZONE_DEG,
    STEERING_FULL_SCALE_DEG,
    MAX_STEERING_COMMAND
  );

  filtered_speed += (target_speed - filtered_speed) * SPEED_SMOOTHING;
  filtered_angle += (target_angle - filtered_angle) * STEERING_SMOOTHING;

  print_motion_debug(speed_tilt, roll, filtered_angle, filtered_speed);
  send_drive_command(filtered_angle, filtered_speed, false);
  delay(10);
}
