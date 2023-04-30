#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <SD.h>
#include <I2Cdev.h>

// ------------------------------------- SENSOR SETUP ------------------------------------------

#define BUZZER_PIN 4
#define MOTOR_CONTROL_PIN 16
#define CAMERA_CONTROL_PIN 17
#define LED_PIN_1 32
#define LED_PIN_2 33
#define LED_PIN_3 25


Adafruit_BME280 bme;


const int freq = 1000;      // set the PWM frequency to 1khz
const int motorChannel = 0;   // set the PWM channel
const int resolution = 8;   // set PWM resolution

const int buzzerChannel = 8;

// Function predefs
inline void check_threshold(
    float bme_temp,
    float bme_press,
    float bme_hum);

uint16_t ArmAccel = 2000;  // TODO : UPDATE! Arming acceleration, ideally, speed off the rail (Upon arm allows cameras to start recording, data logging, pump activation logic runs. etc.)
bool AccelArmState = true; // True = not active, false = active experiment
uint16_t armState = 0;
bool accelArmActivated = false;   // Set acceleration armed to inactive. This is the arming state of the pump activation cycle.

// MPU6050
int accelgyro_init_ok = 0;
inline void print_MPU6050_data(
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz);

MPU6050 mpu; // Create mpu6050 object

unsigned long pump_millis = 0;
unsigned long experiment_length_s = 20*1000;
//This is the code to disable the pumps.
void IRAM_ATTR onTimer(){
  ledcWrite(motorChannel, 0);
  finalise_and_close();
}

// SD CARD and file logging
uint8_t SD_CS_PIN = 29;
#define SD_SAVE_INTERVAL 5000ul // ms
String log_file_dir = "/";
String log_file_name = "Malyrie_Test";
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
    float *axf,
    float *ayf,
    float *azf,
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

void led_test()
{

  digitalWrite(LED_PIN_1, HIGH);
  delay(500);
  digitalWrite(LED_PIN_2, HIGH);
  delay(500);
  digitalWrite(LED_PIN_1, LOW);
  delay(500);
  digitalWrite(LED_PIN_3, HIGH);
  delay(500);
  digitalWrite(LED_PIN_2, LOW);
  delay(500);
  digitalWrite(LED_PIN_3, LOW);
}
void playNote(int frequency, int duration) {
  ledcWrite(buzzerChannel, 128);
  ledcWriteTone(buzzerChannel, frequency); // Play the note
  delay(duration); // Wait for the note to play
  ledcWrite(buzzerChannel, 0); // Stop the note
}

void buzzerTest()
{
   const int melody[] = {
  370, 415, 466, 0, 622, 554	
  };

// Note durations (1 = full note, 2 = half note, 4 = quarter note, etc.)
const int noteDurations[] = {
  8, 8, 8, 16, 2, 1
};

  for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
    int noteDuration = 1000 / noteDurations[i];
    playNote(melody[i], noteDuration);

    // Add a brief pause between notes
    delay(noteDuration * 0.3);
  }



}

void motor_test()
{
  ledcWrite(motorChannel, 200);
  delay(1000);
  ledcWrite(motorChannel, 0);  
}

// Returns new value if signal is stable for specified debounce time: here it is 100ms.
bool debounce(bool value, bool& last_value, unsigned long& stable_time) {
  static const unsigned long DEBOUNCE_TIME = 250; // ms
  static bool valueState = false;

  // Check if the value has changed
  if (value != last_value) {
    // If the value has changed, reset the stable_since time
    stable_time = millis();
  }
  if ((millis() - stable_time) >= DEBOUNCE_TIME) {
    // If the value has been stable for 100ms, return the new value
    if (value != valueState) {
      valueState = value;
    }
  }

  last_value = value;
  // If the value hasn't been stable for 100ms, return the old value
  return valueState;
}



// Returns true if magnitude under threshold
bool check_threshold(float magnitude)
{
  static const double THRESHOLD = THRESHOLD_G * 2048; // TODO: move somewhere else?
  if (magnitude < THRESHOLD)
    return true;
  else
    return false;
}
void enter_arming()
{
  //Turn on LED 1
  digitalWrite(LED_PIN_1, HIGH);
  armState = 0;
}
//The payload is now armed, it has detected launch. In order to prevent misdetection due to movement on the pad, we will trigger a buzzer here. 
//If this buzzer goes off while on the pad, then we need to restart the payload as the experiment will not trigger. 
//This will enable the camera pin, turn on LED 2 and LED 1, and enter the armed state
void begin_armed()
{
  digitalWrite(LED_PIN_2, HIGH);
  armState = 1;
  playNote(440, 500);

}

//The payload has now detected the experimental phase, we now need to begin the experiment.
//Turn on LED 3, begin pump PWM (can we use an interrupt and timer for this?)
void begin_experiment()
{
  //Enable experiment
  digitalWrite(LED_PIN_3, HIGH);
  //Write motor driver PWM
  ledcWrite(motorChannel, 200);
  //Begin interrupt to disable the motors.
  pump_millis = millis();
  armState = 2;
  playNote(440, 200);
}
//Experiment is now finished, note this in the SD card log, turn off LEDs 1 and 2, disable pump. Cameras will turn off by themselves. Close SD card file
void finalise_and_close()
{
  digitalWrite(LED_PIN_1, LOW);
  digitalWrite(LED_PIN_2, LOW);
  armState = 3;

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
  //Initialize serial for debugging
  Serial.begin(115200);
  delay(500);
  //Begin i2c for BME280 and MPU6050, this is needed because the ESP32 is stupid
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Serial.println("Beginning I2c using wire library");
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
        Serial.println("Beginning I2c using fastwire library");
  #endif
  Serial.println("Beginning stack initialization");
  // MPU6050 initialization
  mpu.initialize();
  delay(1000);
  Serial.println("MPU6050 Initialization...");
  accelgyro_init_ok = mpu.testConnection();
  //Serial.println(accelgyro_init_ok ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setFullScaleAccelRange(3); // MPU6050_ACCEL_FS_16 (meant to be this?)
  mpu.setFullScaleGyroRange(3);  // MPU6050_GYRO_FS_2000 (as above)
    
  // SD Card
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

  // Initialize PWM controller on motor pins
  ledcAttachPin(MOTOR_CONTROL_PIN, motorChannel);
  ledcSetup(motorChannel, freq, resolution);  // define the PWM Setup
  motor_test();
  
  // Initialize buzzer PWM controller
  ledcSetup(buzzerChannel, 2000, 8);
  ledcAttachPin(BUZZER_PIN, buzzerChannel);
  buzzerTest();

  if(!accelgyro_init_ok)
  {
    while(true)
    {
      //If we end up in this code, the gyro has not initialized and we CANNOT arm the payload
      playNote(1000.00, 1000);
      delay(1000);
    }
  }

  // OPENMV Cameras
  pinMode(CAMERA_CONTROL_PIN, OUTPUT);   // Set camera control pin as OUTPUT
  digitalWrite(CAMERA_CONTROL_PIN, LOW); // Set initial state to LOW (not recording)
  Serial.println("Camera Initialise");


  //Initialize LEDs and turn on
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);
  led_test();

  // BME280 initialize
  bool status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  } else {
    Serial.println("BME280 initialized!");
    //BME280 is not critical, if it doesn't initialize we can just ignore it.
  }

  //timerAttachInterrupt(My_timer, &onTimer, true);
  //timerAlarmWrite(My_timer, experiment_length, true);
  enter_arming();
}
//Enter arming: This is the first mode, prior to being launched. When in this mode, LED 1 will be on, cameras are not recording, and it is awaiting launch detection
//Add a flag to the SD card indicating the current state

// ------------------------------- LOOP -------------------------------
void loop()
{
  // ------------------------------- RUNS IN ALL STATES -------------------------------
  // Read MPU6050 raw sensor values
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //print_MPU6050_data(ax, ay, az, gx, gy, gz);

  // BME280 Sensor Data
  float bme_temp = bme.readTemperature(); // Read temperature in Celsius
  float bme_hum = bme.readHumidity(); // Read humidity in %
  float bme_press = bme.readPressure() / 100.0F; // Read pressure in hPa

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
  if (avg > ArmAccel && !accelArmActivated) // If hasn't been armed before i.e. not launched off rod. 
  {
    AccelArmState = false; // Active now
    Serial.print("Accel Arm Succesful");
    begin_armed();
    accelArmActivated = true;
  }

  // Log sensor data to SD card - Log data slower when not launched
  static unsigned long previousMillis = 0;
  static const unsigned logInterval = 5000; // ms
  if (AccelArmState) { // Not launched
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= logInterval) {
      // Log at slower rate
      print_MPU6050_data((float)ax/2048, (float)ay/2048, (float)az/2048, (float)gx, (float)gy, (float)gz);
      print_BME280_data(bme_temp, bme_press, bme_hum);
      previousMillis = currentMillis;
    }
  } else { // Launched
    // Log every time
    print_MPU6050_data((float)ax/2048, (float)ay/2048, (float)az/2048, (float)gx, (float)gy, (float)gz);
    print_BME280_data(bme_temp, bme_press, bme_hum);
  }

  //log current arming state
  log_file.println(armState);

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
    static float axf, ayf, azf; // filtered acceleration values
    float filter_alpha = 0.2;     // amount of smoothing applied by filter
    filter_accel(ax, ay, az, &axf, &ayf, &azf, filter_alpha);

    // calculate magnitude value of acceleration
    float magnitude = sqrt(axf * axf + ayf * ayf + azf * azf);

    // Check magnitude against threshold
    bool pump_active_input = check_threshold(magnitude);

    // Debounce input value
    static bool last_value = false;
    static unsigned long debounce_start_time = 0;
    bool debounced_value = debounce(pump_active_input, last_value, debounce_start_time);

    // Pump Running start time
    static bool pumpStartSet = false;
    static unsigned long pumpRunStartTime;

    static bool pumpActivated = false;

    if (isSecondActivation(debounced_value)) {
      if (!pumpStartSet)
      {
        pumpRunStartTime = millis(); // Sets start time
        pumpStartSet = true;
        begin_experiment();
        // TODO: Maybe update from millis() as using millis() for all time features is probably not great practice?
      }
    }

    // Handle debounced value
    // if (isSecondActivation(debounced_value))
    // {
    //   // TODO: Clean this up.
    //   if (!pumpStartSet)
    //   {
    //     pumpRunStartTime = millis(); // Sets start time
    //     pumpStartSet = true;
    //     begin_experiment();
    //     // TODO: Maybe update from millis() as using millis() for all time features is probably not great practice?
    //   }

    //   // ACTIVATE PUMP FOR REAL!

    //   pumpActivated = true; // Set the pumpActivated flag. This is for the backup pump logic.

    //   // TODO: ADD MOTOR CONTROL FUNCTION CALL HERE.
    //   // Maybe just set variable to true. Then at bottom below backup logic depending on variable value can call pump activation function. Therefore avoids some issues with the backup logic.

    //   // TODO: Do we want to block it from shutting off from here after <x seconds? I.e. if returns false after 20seconds do we ovveride here or in backup?
    // }

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
        begin_experiment();
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
    //TODO: Add pump shutdown based on the new code
    // pump_millis = 0;
    //unsigned long experiment_length_s = 20*1000;
    if ((millis() - pump_millis < experiment_length_s) && armState == 2) {
    // Your code here, it will run until the specified time has passed
    ledcWrite(motorChannel, 200);
    } 
    else if(armState == 2)
    {
      ledcWrite(motorChannel, 0);
      finalise_and_close();
    }
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

// TODO: Clean up both these logging functions.
// MPU6050 Log data to SD
inline void print_MPU6050_data(
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz)
{

  uint32_t timestamp = millis();

#ifdef LOG_IN_BINARY_FORMAT

#ifdef MIRROR_SD_WRITES_TO_SERIAL

  Serial.print("\r\nAG");
  Serial.write((byte *)&timestamp, sizeof(uint32_t));
  Serial.write((byte *)&ax, sizeof(float));
  Serial.write((byte *)&ay, sizeof(float));
  Serial.write((byte *)&az, sizeof(float));
  Serial.write((byte *)&gx, sizeof(float));
  Serial.write((byte *)&gy, sizeof(float));
  Serial.write((byte *)&gz, sizeof(float));

#endif

  log_file.print("\r\nAG");
  log_file.write((byte *)&timestamp, sizeof(uint32_t));
  log_file.write((byte *)&ax, sizeof(float));
  log_file.write((byte *)&ay, sizeof(float));
  log_file.write((byte *)&az, sizeof(float));
  log_file.write((byte *)&gx, sizeof(float));
  log_file.write((byte *)&gy, sizeof(float));
  log_file.write((byte *)&gz, sizeof(float));

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
