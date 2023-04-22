#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <SD.h>

// ------------------------------------- SENSOR SETUP ------------------------------------------
//const int BUZZER_PIN = 4;

// #define ARMED_LED_PIN 10
// #define BUZZER_PIN 2
#define CAMERA_CONTROL_PIN 17

// #define MOTOR_CONTROL_PIN 16

// BME280
// #define BME_SCK 13
// #define BME_MISO 12
// #define BME_MOSI 11
// #define BME_CS 10

// Adafruit_BME280 bme;

// // #define BME280_SDA_PIN 33
// // #define BME280_SCL_PIN 36
// // // int bme_init_ok = 0;
// // Adafruit_BME280 bme;  // Create BME280 sensor object
// // inline void print_BME280_data(
// //   float bme_temp,
// //   float bme_press,
// //   float bme_hum
// // );

// Function predefs
inline void check_threshold(
    float bme_temp,
    float bme_press,
    float bme_hum);

uint16_t ArmAccel = 2000;  // TODO : UPDATE! Arming acceleration, ideally, speed off the rail (Upon arm allows cameras to start recording, data logging, pump activation logic runs. etc.)
bool AccelArmState = true; // True = not active, false = active experiment

// MPU6050
int accelgyro_init_ok = 0;
inline void print_MPU6050_data(
    int16_t ax,
    int16_t ay,
    int16_t az,
    int16_t gx,
    int16_t gy,
    int16_t gz);

MPU6050 mpu; // Create mpu6050 object

// SD CARD and file logging
uint8_t SD_CS_PIN = 29;
#define SD_SAVE_INTERVAL 5000ul // ms
String log_file_dir = "/";
String log_file_name = "HPR";
int log_file_number = 0;
String log_file_extension = ".txt";
int log_file_open = 0;
File log_file; // Initialize SD card

inline String get_log_file_path();

int clear_SD_files();

// ------------------------------------- PUMP ACTIVATION LOGIC FUNCTIONS ------------------------------------------
// Threshold Value in G
float THRESHOLD_G = 0.7;

// Hard Coded Activation Values
int start_time = 30; // Sims say 27 seconds from launch. TODO: Make more tolerant?
int end_time = 100;  // Sims say 100 seconds from launch

// exponential smoothing filter for acceleration values from MPU6050
inline void filter_accel(
    int16_t ax,
    int16_t ay,
    int16_t az,
    int16_t *axf,
    int16_t *ayf,
    int16_t *azf,
    float filter_alpha)
{
  static bool first_call = true;

  if (first_call)
  {
    // use unfiltered acceleration values for first call
    *axf = ax;
    *ayf = ay;
    *azf = az;
    first_call = false;
  }
  else
  {
    // apply exponential smoothing filter to acceleration values
    // output = a * current reading + (1 - a) * last reading
    *axf = filter_alpha * ax + (1 - filter_alpha) * *axf;
    *ayf = filter_alpha * ay + (1 - filter_alpha) * *ayf;
    *azf = filter_alpha * az + (1 - filter_alpha) * *azf;
  }
}

// !IF NEEDED
// Returns true if signal is stable for specified debounce time: here it is 100ms.
bool debounce(bool value, bool &last_value, int &debounce_start_time)
{
  static unsigned long last_time = 0;
  static const unsigned long DEBOUNCE_TIME = 100; // milliseconds

  unsigned long current_time = millis(); // get current time in milliseconds
  unsigned long elapsed_time = current_time - last_time;

  if (value != last_value)
  {
    last_value = value;
    debounce_start_time = current_time;
    return false;
  }
  else if (elapsed_time >= DEBOUNCE_TIME)
  {
    last_time = current_time;
    return true;
  }

  return false;
}

// Returns true if magnitude under threshold
bool check_threshold(double magnitude)
{
  static const double THRESHOLD = THRESHOLD_G * 9.81; // TODO: move somewhere else?
  if (magnitude < THRESHOLD)
    return true;
  else
    return false;
}

#define IDLE_STATE 0
#define FIRST_ACTIVATION_STATE 1
#define SECOND_ACTIVATION_STATE 2
#define SECOND_ACTIVATION_HELD_STATE 3
#define DONE_STATE 4

// Will ignore the first activation time regardless of the length of activation. Upon exiting activation it will then accept the second activation.
// Should just return true upon second activation below threshold and will keep returning true until exiting the threshold.
// Upon this will no longer return true regardless of threshold values.

// TODO: Can update this code so that it just returns true upon each threshold cross. So rather than continously returning true. It returns true once which activates pump. Then upon returning false it deactivates pump.
// Therfore doesn't continously update pump activation. Activates pump once and then shuts off once. Much more simple!
bool isSecondActivation(bool isBelowThreshold)
{
  static unsigned char state = IDLE_STATE;
  bool secondActivationDetected = false;

  switch (state)
  {
  case IDLE_STATE:
    if (isBelowThreshold)
    {
      state = FIRST_ACTIVATION_STATE;
    }
    break;

  case FIRST_ACTIVATION_STATE:
    if (!isBelowThreshold)
    {
      state = SECOND_ACTIVATION_STATE;
    }
    break;

  case SECOND_ACTIVATION_STATE:
    if (isBelowThreshold)
    {
      secondActivationDetected = true;
      state = SECOND_ACTIVATION_HELD_STATE;
    }
    break;

  case SECOND_ACTIVATION_HELD_STATE:
    if (!isBelowThreshold)
    {
      state = DONE_STATE;
    }
    else
    {
      secondActivationDetected = true;
    }
    break;

  case DONE_STATE:
    // do nothing
    break;
  }

  return secondActivationDetected;
}

// ------------------------------- SETUP -------------------------------
void setup()
{
  Serial.begin(115200);

  Wire.begin();

  // MPU6050
  mpu.initialize();
  delay(1000);
  Serial.println("MPU6050 Initialization...");
  accelgyro_init_ok = mpu.testConnection();
  Serial.println(accelgyro_init_ok ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setFullScaleAccelRange(3); // MPU6050_ACCEL_FS_16 (meant to be this?)
  mpu.setFullScaleGyroRange(3);  // MPU6050_GYRO_FS_2000 (as above)

  // SD Card
  pinMode(SD_CS_PIN, OUTPUT);
  Serial.println("Starting SD card.");
  if (!SD.begin())
  {
    Serial.println("SD initialization failed!");
  }
  else
  {
    Serial.println("Will write to " + get_log_file_path());
  }
  delay(1000);

  // BUZZER CODE?

  // OPENMV Cameras
  pinMode(CAMERA_CONTROL_PIN, OUTPUT);   // Set camera control pin as OUTPUT
  digitalWrite(CAMERA_CONTROL_PIN, LOW); // Set initial state to LOW (not recording)
  Serial.println("Camera Initialise");

  // BME280
  // status = bme.begin();
  // if (!status) {
  //   Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  //   Serial.print("SensorID was: 0x");
  //   Serial.println(bme.sensorID(), 16);
  //   Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  //   Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  //   Serial.print("        ID of 0x60 represents a BME 280.\n");
  //   Serial.print("        ID of 0x61 represents a BME 680.\n");
  //   while (1) delay(10);
  // }
  // // // Other test BME280
  // // Wire.begin(BME280_SDA_PIN, BME280_SCL_PIN); // Initialize I2C with the specified SDA and SCL pins
  // // if (!bme.begin(0x76)) { // Initialize the BME280 sensor with the specified I2C address
  // //   Serial.println("Could not find a valid BME280 sensor, check wiring!");
  // //   while (1);
  // // }

  // Pump Activation
  // pinMode(MOTOR_CONTROL_PIN, OUTPUT);  // Set camera control pin as OUTPUT
}

// ------------------------------- LOOP -------------------------------
void loop()
{
  // ------------------------------- RUNS IN ALL STATES -------------------------------
  // Read MPU6050 raw sensor values
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  print_MPU6050_data(ax, ay, az, gx, gy, gz);

  digitalWrite(CAMERA_CONTROL_PIN, HIGH);
  Serial.println("camera running loop");

  // Convert accelerometer readings to m/s^2
  // float ax_mps2 = ax * 9.80665 / 16384.0; // Convert accelerometer values to m/s^2
  // float ay_mps2 = ay * 9.80665 / 16384.0;
  // float az_mps2 = az * 9.80665 / 16384.0;

  // BME280 Sensor Data
  // float bme_temp = bme.readTemperature(); // Read temperature in Celsius
  // float bme_hum = bme.readHumidity(); // Read humidity in %
  // float bme_press = bme.readPressure() / 100.0F; // Read pressure in hPa

  // Serial.print("Temperature: ");
  // Serial.print(bme_temp);
  // Serial.print(" °C, Humidity: ");
  // Serial.print(bme_hum);
  // Serial.print(" %, Pressure: ");
  // Serial.print(bme_press);
  // Serial.println(" hPa");

  // SD Card Logging
  // TODO: Clean up this code
  static unsigned long last_file_save_timestamp = 0;
  // re-open closed file
  // (we may have 'saved' it by closing it, or not yet ever opened it)
  if (!log_file_open)
  {
    Serial.println("Trying to open " + get_log_file_path() + " in append mode.");
    log_file = SD.open(get_log_file_path(), FILE_APPEND);
    log_file_open = log_file != false;
    if (!log_file_open)
    {
      Serial.println("Trying to open " + get_log_file_path() + " as a new file in write mode.");
      log_file = SD.open(get_log_file_path(), FILE_WRITE);
      log_file_open = log_file != false;
    }
    if (!log_file_open)
    {
      Serial.println("Could not open " + get_log_file_path() + " for writing!");
    }
    else
    {
      Serial.println("Opened " + get_log_file_path() + " for writing!");
    }
  }

  // do a 'save' by closing, or flushing
  if (log_file_open && (millis() - last_file_save_timestamp > SD_SAVE_INTERVAL))
  {
    log_file.flush();
    // log_file.close();
    // log_file_open = 0;
    Serial.println("Closed/saved " + get_log_file_path());
    last_file_save_timestamp = millis();
    // log_file_number++;
  }

  // Off the Rod Arming
  // If average accel is greater than arm accel. Arm and start logging stuff:
  int16_t avg = ((ax + ay + az)) / 3; //=(((ax^2+ay^2+az^2)^(1/2))/16)/3
  if (avg > ArmAccel)
  {
    AccelArmState = false; // Active now
    Serial.print("Accel Arm Succesful");
    // digitalWrite(ARMED_LED_PIN, HIGH);
  }

  // Log sensor data to SD card
  print_MPU6050_data(ax, ay, az, gx, gy, gz);
  // print_BME280_data(bme_temp, bme_press, bme_hum);

  // ------------------------------- ACTIVE FLIGHT PERIOD (CAMERA + ACTIVATiON LOGIC) -------------------------------
  if (AccelArmState == false)
  { // Experiment ready to be active
    static bool activeStartTimeSet = false;
    static unsigned long activeStartTime;

    if (!activeStartTimeSet)
    {
      activeStartTime = millis(); // set active start time on the first iteration
      activeStartTimeSet = true;  // set flag to true to prevent setting it again
    }

    unsigned long currentMillis = millis(); // grab current time

    // Probably can do whatever you need to do here. This is the active state AFAIK.

    // Start Camera Recording
    digitalWrite(CAMERA_CONTROL_PIN, HIGH); // TODO: Update camera code as RN it's just ~ 30 seconds recording or something.
    Serial.println("Camera Started Recording");

    // PUMP ACTIVATION LOGIC
    static int16_t axf, ayf, azf; // filtered acceleration values
    float filter_alpha = 0.2;     // amount of smoothing applied by filter
    filter_accel(ax, ay, az, &axf, &ayf, &azf, filter_alpha);

    // calculate magnitude value of acceleration
    int16_t magnitude = sqrt(axf * axf + ayf * ayf + azf * azf);

    // Check magnitude against threshold
    bool pump_active_input = check_threshold(magnitude);

    // Debounce input value
    static bool last_value = false;
    static int debounce_start_time = 0;
    bool debounced_value = debounce(pump_active_input, last_value, debounce_start_time);

    // Pump Running start time
    static bool pumpStartSet = false;
    static int16_t pumpRunStartTime;

    static bool pumpActivated = false;

    // Handle debounced value
    if (isSecondActivation(debounced_value))
    {
      // TODO: Clean this up.
      if (!pumpStartSet)
      {
        pumpRunStartTime = millis(); // Sets start time
        pumpStartSet = true;
        // TODO: Maybe update from millis() as using millis() for all time features is probably not great practice?
      }

      // ACTIVATE PUMP FOR REAL!

      pumpActivated = true; // Set the pumpActivated flag. This is for the backup pump logic.

      // TODO: ADD MOTOR CONTROL FUNCTION CALL HERE.
      // Maybe just set variable to true. Then at bottom below backup logic depending on variable value can call pump activation function. Therefore avoids some issues with the backup logic.

      // TODO: Do we want to block it from shutting off from here after <x seconds? I.e. if returns false after 20seconds do we ovveride here or in backup?
    }

    // ----------- BACKUP PUMP ACTIVATION LOGIC --------------
    // TODO: Update as currently this will mess with above logic and may switch values very quickly?
    // If pump hasn't activated by start_time, activate it regardless of acceleration threshold
    if (currentMillis - activeStartTime >= start_time * 1000 && !pumpActivated)
    {
      Serial.println("Backup Pump Activation Activated");
      if (!pumpStartSet)
      {
        pumpRunStartTime = millis(); // Sets start time
        pumpStartSet = true;
      }

      // pumpActiveBool = true;
    }

    // Activate the pump if the flag is set
    // If its past the start time and the pump hasn't been activated before. Backup time!
    // if (activatePump && !pumpActivated) {
    //   // Two ways to do this. 1) Activate seperate function once that runs it for ~30 seconds and then shuts off. 2) Checks each time.
    //   // activatePump = false;               // reset the flag
    //   // pumpActivated = true;               // set the pumpActivated flag

    //   // Second Method:
    //   Serial.println("Activating Pump from backup");
    //   if (!pumpStartSet) {
    //     pumpRunStartTime = millis();  // Sets start time
    //     pumpStartSet = true;
    //     // TODO: Maybe update from millis() as using millis() for all time features is probably not great practice?
    //   }

    // TODO: ADD MOTOR CONTROL FUNCTION CALL HERE.
    // OR pumpActiveBool = true;

    // Stop activation if elapsed time since armed accel is greater than end time.
    if (pumpStartSet && currentMillis - activeStartTime >= end_time * 1000)
    {
      Serial.println("Stopping backup pump activation");
      // pumpActiveBool = false;
    }

    // SECONDARY METHOD:
    // if (pumpActiveBool) {
    // activate pump here not above. This will work better with backup logic
    //}
  }
}

// ------------------------------- FUNCTIONS -------------------------------
inline String get_log_file_path()
{
  return log_file_dir + log_file_name + String(log_file_number) + log_file_extension;
}

// Motor controller code.
// TODO: Update! Currently not working.
// inline void motor_rotation(uint8_t pin, int runtime) {
//   int t = 1800;  //do 1800 //Forward rotation, mapped from 5-10% duty cycle between 1500 to 2000. PPM protocol is weird lol
//   unsigned long startTime = millis();

//   while (millis() - startTime < runtime * 1000) {
//     Serial.println("motor rotate");
//     digitalWrite(pin, HIGH);
//     delayMicroseconds(t);
//     digitalWrite(pin, LOW);
//     delayMicroseconds(10000);
//     delayMicroseconds(10000 - t);
//   }
// }

// TODO: Clean up both these logging functions.
// MPU6050 Log data to SD
inline void print_MPU6050_data(
    int16_t ax,
    int16_t ay,
    int16_t az,
    int16_t gx,
    int16_t gy,
    int16_t gz)
{

  uint32_t timestamp = millis();

#ifdef LOG_IN_BINARY_FORMAT

#ifdef MIRROR_SD_WRITES_TO_SERIAL

  Serial.print("\r\nAG");
  Serial.write((byte *)&timestamp, sizeof(uint32_t));
  Serial.write((byte *)&ax, sizeof(int16_t));
  Serial.write((byte *)&ay, sizeof(int16_t));
  Serial.write((byte *)&az, sizeof(int16_t));
  Serial.write((byte *)&gx, sizeof(int16_t));
  Serial.write((byte *)&gy, sizeof(int16_t));
  Serial.write((byte *)&gz, sizeof(int16_t));

#endif

  log_file.print("\r\nAG");
  log_file.write((byte *)&timestamp, sizeof(uint32_t));
  log_file.write((byte *)&ax, sizeof(int16_t));
  log_file.write((byte *)&ay, sizeof(int16_t));
  log_file.write((byte *)&az, sizeof(int16_t));
  log_file.write((byte *)&gx, sizeof(int16_t));
  log_file.write((byte *)&gy, sizeof(int16_t));
  log_file.write((byte *)&gz, sizeof(int16_t));

#else

#ifdef MIRROR_SD_WRITES_TO_SERIAL

  Serial.print(timestamp);
  Serial.print(", a/g, ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.print(az);
  Serial.print(", ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.println(gz);
#endif

  log_file.print(timestamp);
  log_file.print(", a/g, ");
  log_file.print(ax);
  log_file.print(", ");
  log_file.print(ay);
  log_file.print(", ");
  log_file.print(az);
  log_file.print(", ");
  log_file.print(gx);
  log_file.print(", ");
  log_file.print(gy);
  log_file.print(", ");
  log_file.println(gz);

#endif
}

// BME280 Log data to SD
inline void print_BME280_data(
    float bme_temp,
    float bme_press,
    float bme_hum)
{

  uint32_t timestamp = millis();

#ifdef LOG_IN_BINARY_FORMAT

#ifdef MIRROR_SD_WRITES_TO_SERIAL

  Serial.print("\r\nBME");
  Serial.write((byte *)&timestamp, sizeof(uint32_t));
  Serial.write((byte *)&bme_temp, sizeof(float));
  Serial.write((byte *)&bme_press, sizeof(float));
  Serial.write((byte *)&bme_hum, sizeof(float));

#endif

  log_file.print("\r\nBME");
  log_file.write((byte *)&timestamp, sizeof(uint32_t));
  log_file.write((byte *)&bme_temp, sizeof(float));
  log_file.write((byte *)&bme_press, sizeof(float));
  log_file.write((byte *)&bme_hum, sizeof(float));

#else

#ifdef MIRROR_SD_WRITES_TO_SERIAL

  Serial.print(timestamp);
  Serial.print(", tph, ");
  Serial.print(bme_temp, 3);
  Serial.print(", ");
  Serial.print(bme_press, 3);
  Serial.print(", ");
  Serial.println(bme_hum, 3);

#endif

  log_file.print(timestamp);
  log_file.print(", tph, ");
  log_file.print(bme_temp, 3);
  log_file.print(", ");
  log_file.print(bme_press, 3);
  log_file.print(", ");
  log_file.println(bme_hum, 3);

#endif
}