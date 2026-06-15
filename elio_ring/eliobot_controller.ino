#include "LSM6DS3.h"
#include "Wire.h"
#include <bluefruit.h>
#include <math.h>

BLEClientUart robot_uart;

LSM6DS3 myIMU(I2C_MODE, 0x6A);

void send_drive_command(float angle, float speed, bool force);
void send_eye_toggle_command();

const int CALIBRATE_PIN = 10;
const int STOP_PIN = 8;

const float RADIANS_TO_DEGREES = 57.2957795f;
const uint16_t CALIBRATION_SAMPLES = 200;
const bool IMU_DEBUG_ONLY = false;

// A finger does not move through large angles. Lower FULL_* values make the
// controller reach max command with a smaller tilt.
const float SPEED_DEADZONE_DEG = 3.0f;
const float SPEED_FULL_SCALE_DEG = 24.0f;
const float SPEED_DIRECTION = -1.0f;
const float STEERING_DEADZONE_DEG = 8.0f;
const float STEERING_FULL_SCALE_DEG = 45.0f;
const float MAX_STEERING_COMMAND = 90.0f;

const float SPEED_SMOOTHING = 0.22f;
const float STEERING_SMOOTHING = 0.25f;
const uint16_t SEND_INTERVAL_MS = 50;
const uint16_t DEBUG_INTERVAL_MS = 200;
const uint16_t BUTTON_DEBOUNCE_MS = 200;
const uint16_t STOP_LONG_PRESS_MS = 1000;

float speed_tilt_bias = 0.0f;
float roll_bias = 0.0f;
float raw_x_bias = 0.0f;
float raw_y_bias = 0.0f;
float raw_z_bias = 0.0f;
float filtered_speed = 0.0f;
float filtered_angle = 0.0f;
float last_sent_speed = 999.0f;
float last_sent_angle = 999.0f;
uint32_t last_send_ms = 0;
uint32_t last_debug_ms = 0;
uint32_t last_stop_button_ms = 0;
uint32_t last_eye_button_ms = 0;
bool stop_enabled = false;
bool stop_button_was_pressed = false;
bool stop_button_press_active = false;
bool stop_button_long_press_handled = false;
uint32_t stop_button_press_start_ms = 0;
bool eye_button_was_pressed = false;
bool imu_debug_output_enabled = true;

struct AccelReading {
  float raw_x;
  float raw_y;
  float raw_z;
};

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

static AccelReading read_accel() {
  AccelReading reading;

  reading.raw_x = myIMU.readFloatAccelX();
  reading.raw_y = myIMU.readFloatAccelY();
  reading.raw_z = myIMU.readFloatAccelZ();
  return reading;
}

static float speed_tilt_degrees(const AccelReading& reading) {
  return atan2f(reading.raw_z, reading.raw_y) * RADIANS_TO_DEGREES;
}

static float roll_degrees(const AccelReading& reading) {
  return atan2f(reading.raw_x, reading.raw_y) * RADIANS_TO_DEGREES;
}

void calibrate_neutral() {
  float speed_tilt_sum = 0.0f;
  float roll_sum = 0.0f;
  float raw_x_sum = 0.0f;
  float raw_y_sum = 0.0f;
  float raw_z_sum = 0.0f;

  digitalWrite(LED_BLUE, LOW);

  for (uint16_t sample = 0; sample < CALIBRATION_SAMPLES; ++sample) {
    AccelReading reading = read_accel();
    speed_tilt_sum += speed_tilt_degrees(reading);
    roll_sum += roll_degrees(reading);
    raw_x_sum += reading.raw_x;
    raw_y_sum += reading.raw_y;
    raw_z_sum += reading.raw_z;
    delay(5);
  }

  speed_tilt_bias = speed_tilt_sum / CALIBRATION_SAMPLES;
  roll_bias = roll_sum / CALIBRATION_SAMPLES;
  raw_x_bias = raw_x_sum / CALIBRATION_SAMPLES;
  raw_y_bias = raw_y_sum / CALIBRATION_SAMPLES;
  raw_z_bias = raw_z_sum / CALIBRATION_SAMPLES;
  filtered_speed = 0.0f;
  filtered_angle = 0.0f;

  digitalWrite(LED_BLUE, HIGH);

  Serial.printf(
    "Calibration complete: speed_tilt_bias=%.2f roll_bias=%.2f raw_bias=(%.2f,%.2f,%.2f)\n",
    speed_tilt_bias,
    roll_bias,
    raw_x_bias,
    raw_y_bias,
    raw_z_bias
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

void send_eye_toggle_command() {
  if (!Bluefruit.connected() || !robot_uart.discovered()) {
    return;
  }

  robot_uart.print("eyes\n");
  Serial.println("sent: eyes");
}

void print_motion_debug(const AccelReading& reading, float speed_tilt, float roll, float angle, float speed) {
  uint32_t now = millis();

  if (now - last_debug_ms < DEBUG_INTERVAL_MS) {
    return;
  }

  last_debug_ms = now;
  Serial.printf(
    "motion: raw=(%.2f,%.2f,%.2f) speed_tilt=%.1f roll=%.1f angle=%.1f speed=%.1f stop_pin=%d eye_pin=%d\n",
    reading.raw_x,
    reading.raw_y,
    reading.raw_z,
    speed_tilt,
    roll,
    angle,
    speed,
    digitalRead(STOP_PIN),
    digitalRead(CALIBRATE_PIN)
  );
}

void print_imu_debug_only() {
  uint32_t now = millis();

  if (now - last_debug_ms < DEBUG_INTERVAL_MS) {
    return;
  }

  last_debug_ms = now;

  AccelReading reading = read_accel();
  Serial.printf(
    "imu: raw=(%.3f,%.3f,%.3f) delta=(%.3f,%.3f,%.3f) speed_tilt=%.1f roll=%.1f\n",
    reading.raw_x,
    reading.raw_y,
    reading.raw_z,
    reading.raw_x - raw_x_bias,
    reading.raw_y - raw_y_bias,
    reading.raw_z - raw_z_bias,
    angle_delta_degrees(speed_tilt_degrees(reading), speed_tilt_bias),
    roll_degrees(reading) - roll_bias
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

  if (IMU_DEBUG_ONLY) {
    Serial.println("IMU debug only: BLE drive disabled");
    return;
  }

  setup_bluetooth();
}

void loop() {
  if (IMU_DEBUG_ONLY) {
    if (digitalRead(CALIBRATE_PIN) == LOW) {
      calibrate_neutral();
      delay(250);
      return;
    }

    bool stop_button_pressed = digitalRead(STOP_PIN) == LOW;
    uint32_t now = millis();

    if (
      stop_button_pressed &&
      !stop_button_was_pressed &&
      now - last_stop_button_ms >= BUTTON_DEBOUNCE_MS
    ) {
      imu_debug_output_enabled = !imu_debug_output_enabled;
      last_stop_button_ms = now;

      if (imu_debug_output_enabled) {
        Serial.println("debug_output: on");
        last_debug_ms = 0;
      } else {
        Serial.println("debug_output: off");
      }
    }

    stop_button_was_pressed = stop_button_pressed;

    if (imu_debug_output_enabled) {
      print_imu_debug_only();
    }

    delay(10);
    return;
  }

  if (!Bluefruit.connected()) {
    delay(20);
    return;
  }

  read_robot_responses();

  bool eye_button_pressed = digitalRead(CALIBRATE_PIN) == LOW;
  uint32_t now = millis();

  if (
    eye_button_pressed &&
    !eye_button_was_pressed &&
    now - last_eye_button_ms >= BUTTON_DEBOUNCE_MS
  ) {
    send_eye_toggle_command();
    last_eye_button_ms = now;
  }

  eye_button_was_pressed = eye_button_pressed;

  bool stop_button_pressed = digitalRead(STOP_PIN) == LOW;

  if (
    stop_button_pressed &&
    !stop_button_was_pressed &&
    now - last_stop_button_ms >= BUTTON_DEBOUNCE_MS
  ) {
    stop_button_press_active = true;
    stop_button_long_press_handled = false;
    stop_button_press_start_ms = now;
    last_stop_button_ms = now;
  }

  if (
    stop_button_pressed &&
    stop_button_press_active &&
    !stop_button_long_press_handled &&
    now - stop_button_press_start_ms >= STOP_LONG_PRESS_MS
  ) {
    stop_button_long_press_handled = true;
    stop_enabled = false;
    filtered_speed = 0.0f;
    filtered_angle = 0.0f;
    send_drive_command(0.0f, 0.0f, true);
    calibrate_neutral();
    send_drive_command(0.0f, 0.0f, true);
    Serial.println("controller_reset: long_stop_press");
  }

  if (!stop_button_pressed && stop_button_was_pressed && stop_button_press_active) {
    if (!stop_button_long_press_handled) {
      stop_enabled = !stop_enabled;
      filtered_speed = 0.0f;
      filtered_angle = 0.0f;
      send_drive_command(0.0f, 0.0f, true);
      Serial.printf("stop_toggle: %s\n", stop_enabled ? "on" : "off");
    }

    stop_button_press_active = false;
    stop_button_long_press_handled = false;
    last_stop_button_ms = now;
  }

  stop_button_was_pressed = stop_button_pressed;

  if (stop_button_long_press_handled) {
    delay(10);
    return;
  }

  if (stop_enabled) {
    send_drive_command(0.0f, 0.0f, false);
    delay(10);
    return;
  }

  AccelReading reading = read_accel();
  float speed_tilt = angle_delta_degrees(speed_tilt_degrees(reading), speed_tilt_bias) * SPEED_DIRECTION;
  float roll = roll_degrees(reading) - roll_bias;

  float target_speed = apply_deadzone_scaled(speed_tilt, SPEED_DEADZONE_DEG, SPEED_FULL_SCALE_DEG, 100.0f);
  float target_angle = apply_deadzone_scaled(
    roll,
    STEERING_DEADZONE_DEG,
    STEERING_FULL_SCALE_DEG,
    MAX_STEERING_COMMAND
  );

  filtered_speed += (target_speed - filtered_speed) * SPEED_SMOOTHING;
  filtered_angle += (target_angle - filtered_angle) * STEERING_SMOOTHING;

  print_motion_debug(reading, speed_tilt, roll, filtered_angle, filtered_speed);
  send_drive_command(filtered_angle, filtered_speed, false);
  delay(10);
}
