//#define MIRROR_SD_WRITES_TO_SERIAL
//#define LOG_IN_BINARY_FORMAT

#define ARMED_LED_PIN 32
#define RXD2 16
#define TXD2 17
#define BATTERY_VOLTAGE_PIN 35
#define BATTERY_VOLTAGE_MULT 2


#include "I2Cdev.h"
#include "MPU6050.h"
#include "QMC5883L.h"
#include "BME280.h"
#include "Wire.h"

MPU6050 accelgyro(0x68);
int16_t ax, ay, az;
int16_t gx, gy, gz;
int accelgyro_init_ok = 0;

QMC5883L mag;
int16_t mx, my, mz, mt;
int mag_init_ok = 0;

BME280 bme(0x76);
float bme_temp;
float bme_press;
float bme_hum;
int bme_init_ok = 0;

char gps_string[200];
int gps_string_end_index = 0;


#include "FS.h"
#include "SD.h"
#include "SPI.h"
#define SD_CS_PIN 29
#define SD_SAVE_INTERVAL 5000ul // ms
String log_file_dir = "/";
String log_file_name = "HPR";
int log_file_number = 0;
String log_file_extension = ".txt";
int log_file_open = 0;
File log_file;

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <string.h> // for strtok
const char* ssid = "Naboo(b)";
const char* password = "monashHPR";
WiFiServer server(80);

// ------ predefs -----
inline String get_log_file_path();

int clear_SD_files();

String parse_get_request(String line);
void serve_main_page(WiFiClient &client);
void serve_redirect_to_main_page(WiFiClient &client);

inline void print_state(byte currentState);

inline void print_MPU6050_data(
  int16_t ax,
  int16_t ay,
  int16_t az,
  int16_t gx,
  int16_t gy,
  int16_t gz
);

inline void print_HMC5883L_data(
  int16_t mx,
  int16_t my,
  int16_t mz,
  int16_t mt
);

inline void print_BME280_data(
  float bme_temp,
  float bme_press,
  float bme_hum
);

inline void print_GPS_data(
  char* dat_string
);

typedef enum {
  AWAITING_ARM_SIGNAL,
  ARMED
} State;

int period = 20000; //Time to deploy after acceleration arming
unsigned long previousMillis = 0; //intial time

uint16_t ArmAccel = 2000; //Arming acceleration, ideally, speed off the rail

int deployPin = 4;//GPIO4

bool AccelArmState = true;
bool something = true;

void setup() {

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  pinMode(ARMED_LED_PIN, OUTPUT);
  digitalWrite(ARMED_LED_PIN, LOW);

  pinMode(deployPin, OUTPUT);
  digitalWrite(deployPin, LOW);

  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  Wire.begin();
  
  accelgyro.initialize();
  Serial.println("Testing MPU6050 connection...");
  accelgyro_init_ok = accelgyro.testConnection();
  Serial.println(accelgyro_init_ok ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
//  MPU6050_self_test(accelgyro);
  
  mag.init();
  mag.setSamplingRate(200);
  Serial.println("Testing HMC5883L connection...");
  mag_init_ok = mag.ready();
  Serial.println(mag_init_ok ? "HMC5883L connection successful" : "HMC5883L connection failed");

  bme.init();
  Serial.println("Testing BME280 connection...");
  bme_init_ok = bme.checkConnection();
  Serial.println(bme_init_ok ? "BME280 connection successful" : "BME280 connection failed");

  pinMode(SD_CS_PIN, OUTPUT);
  Serial.println("Starting SD card.");
  if (!SD.begin()) {
    Serial.println("SD initialization failed!");
  } else {
    Serial.println("Will write to " + get_log_file_path());
  }

  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
  Serial.println("Server and WiFi started");

  delay(1000);
}


void loop() {
  
  static unsigned long last_file_save_timestamp = 0;
  static byte currentState = AWAITING_ARM_SIGNAL;


  // ---------- stuff that runs in all states -------

  // re-open closed file 
  // (we may have 'saved' it by closing it, or not yet ever opened it)
  if (!log_file_open) {
    Serial.println("Trying to open " + get_log_file_path() + " in append mode.");
    log_file = SD.open(get_log_file_path(), FILE_APPEND);
    log_file_open = log_file != false;
    if (!log_file_open) {
      Serial.println("Trying to open " + get_log_file_path() + " as a new file in write mode.");
      log_file = SD.open(get_log_file_path(), FILE_WRITE);
      log_file_open = log_file != false;
    }
    if (!log_file_open) {
      Serial.println("Could not open " + get_log_file_path() + " for writing!");
    } else {
      Serial.println("Opened " + get_log_file_path() + " for writing!");
    }
  }

  // do a 'save' by closing, or flushing
  if (log_file_open && (millis() - last_file_save_timestamp > SD_SAVE_INTERVAL)) {
    log_file.flush();
    //log_file.close();
    //log_file_open = 0;
    Serial.println("Closed/saved " + get_log_file_path());
    last_file_save_timestamp = millis();
    //log_file_number++;
  }

  // --------- state transition checks -----------
  
  if (currentState == AWAITING_ARM_SIGNAL) {

    int armed = 0;

    WiFiClient client = server.available();   // listen for incoming clients
  
    if (client) {                             // if you get a client,
      Serial.println("New Client.");           // print a message out the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected()) {            // loop while the client's connected
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          if (c == '\n') {                    // if the byte is a newline character

            String url = parse_get_request(currentLine);
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            Serial.println("requested url is: " + url);
            if (url.equals("/") || url.equals("")) {

              Serial.println("serving main page");
              serve_main_page(client);
              break;
              
            } else if (url.equals("/H")) {

              Serial.println("arming, redirect");
              armed = 1;               // GET /H turns the LED on
              serve_redirect_to_main_page(client);
              break;
              
            } else if (url.equals("/L")) {

              Serial.println("clearing SD file, redirect");
              if (log_file_open) {
                log_file.close();
                log_file_open = 0;
              }
              if (SD.remove(get_log_file_path())) {
                  Serial.println("\nLog file[s] deleted");
              } else {
                  Serial.println("\nLog file delete failed");
              }
              serve_redirect_to_main_page(client);
              break;
              
            } else {
              // TODO: serve 404
              Serial.println("url not found");
            }

            currentLine = "";
            
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }

        }
      }
      // close the connection:
      client.stop();
      Serial.println("Client Disconnected.");
    }
    
    if (armed) {
      
      currentState = ARMED;
      
      server.close();
      WiFi.mode(WIFI_MODE_NULL);
      print_state(currentState);
      
    }
    
  } else if (currentState == ARMED) {
    
  }


  // -------- do the actual stuff expected in each state ----------
  if (currentState == AWAITING_ARM_SIGNAL) {
    
    
  } else if (currentState == ARMED) {

      // just log stuff
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      int16_t avg = ((ax + ay + az))/3; //=(((ax^2+ay^2+az^2)^(1/2))/16)/3

      if (avg > ArmAccel){

        AccelArmState = false;
        Serial.print("Accelero Arm Succesful");
        digitalWrite(ARMED_LED_PIN, HIGH);
        
        }

      
      if(AccelArmState == false){

        unsigned long currentMillis = millis(); // grab current time
 
         while ((unsigned long)(currentMillis - previousMillis) >= period) {
           // save the "current" time
          
           if(something == false){
            
           digitalWrite(deployPin, HIGH);
           Serial.println("DEPLOYED!");
           delay(5000);
           
           }

           something = false;
          
           previousMillis = millis();
         }  
    
      }

      print_MPU6050_data(ax, ay, az, gx, gy, gz);

      mag.readRaw(&mx, &my, &mz, &mt);
      print_HMC5883L_data(mx, my, mz, mt);

      bme.getValues(&bme_temp, &bme_press, &bme_hum);
      print_BME280_data(bme_temp, bme_press, bme_hum);

      while (Serial2.available()) {

        char c = char(Serial2.read());
        //Serial.print(c);

        if (c == '\n') {

          gps_string[gps_string_end_index - 1] = '\0';
          
          // check if we should save what was captured
          if (1) { //strncmp(gps_string, "$GPRMC", 6) == 0) {

            print_GPS_data(gps_string);
            
          }

          // clear up
          gps_string_end_index = 0;
          
        } else {
          
          gps_string[gps_string_end_index++] = c;
        
        }
        
     }
      
  }
  
}

inline void print_state(byte currentState) {
  
  uint32_t timestamp = millis();
  
#ifdef LOG_IN_BINARY_FORMAT

  #ifdef MIRROR_SD_WRITES_TO_SERIAL

  Serial.print("\r\nS");
  Serial.write( (byte*) &timestamp, sizeof(uint32_t) );
  Serial.write( (byte*) &currentState, sizeof(byte) );

  #endif

  log_file.print("\r\nS");
  log_file.write( (byte*) &timestamp, sizeof(uint32_t) );
  log_file.write( (byte*) &currentState, sizeof(byte) );  
  
#else
    
  #ifdef MIRROR_SD_WRITES_TO_SERIAL
  
    Serial.print(timestamp); 
    Serial.print(", new state entered, ");
    Serial.println(currentState, DEC);
    
  #endif

    log_file.print(timestamp); 
    log_file.print(", new state entered, ");
    log_file.println(currentState, DEC);
    
#endif
  
}


inline void print_MPU6050_data(
  int16_t ax,
  int16_t ay,
  int16_t az,
  int16_t gx,
  int16_t gy,
  int16_t gz
) {
  
  uint32_t timestamp = millis();
  
#ifdef LOG_IN_BINARY_FORMAT

  #ifdef MIRROR_SD_WRITES_TO_SERIAL

  Serial.print("\r\nAG");
  Serial.write( (byte*) &timestamp, sizeof(uint32_t) );
  Serial.write( (byte*) &ax, sizeof(int16_t) );
  Serial.write( (byte*) &ay, sizeof(int16_t) );
  Serial.write( (byte*) &az, sizeof(int16_t) );
  Serial.write( (byte*) &gx, sizeof(int16_t) );
  Serial.write( (byte*) &gy, sizeof(int16_t) );
  Serial.write( (byte*) &gz, sizeof(int16_t) );

  #endif

  log_file.print("\r\nAG");
  log_file.write( (byte*) &timestamp, sizeof(uint32_t) );
  log_file.write( (byte*) &ax, sizeof(int16_t) );
  log_file.write( (byte*) &ay, sizeof(int16_t) );
  log_file.write( (byte*) &az, sizeof(int16_t) );
  log_file.write( (byte*) &gx, sizeof(int16_t) );
  log_file.write( (byte*) &gy, sizeof(int16_t) );
  log_file.write( (byte*) &gz, sizeof(int16_t) );  
  
#else
    
  #ifdef MIRROR_SD_WRITES_TO_SERIAL
  
    Serial.print(timestamp);
    Serial.print(", a/g, ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az); Serial.print(", ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.println(gz);
    
  #endif

    log_file.print(timestamp); 
    log_file.print(", a/g, ");
    log_file.print(ax); log_file.print(", ");
    log_file.print(ay); log_file.print(", ");
    log_file.print(az); log_file.print(", ");
    log_file.print(gx); log_file.print(", ");
    log_file.print(gy); log_file.print(", ");
    log_file.println(gz);
    
#endif

}

inline void print_HMC5883L_data(
  int16_t mx,
  int16_t my,
  int16_t mz,
  int16_t mt
) {
  
  uint32_t timestamp = millis();
  
#ifdef LOG_IN_BINARY_FORMAT

  #ifdef MIRROR_SD_WRITES_TO_SERIAL

  Serial.print("\r\nM");
  Serial.write( (byte*) &timestamp, sizeof(uint32_t) );
  Serial.write( (byte*) &mx, sizeof(int16_t) );
  Serial.write( (byte*) &my, sizeof(int16_t) );
  Serial.write( (byte*) &mz, sizeof(int16_t) );
  Serial.write( (byte*) &mt, sizeof(int16_t) );

  #endif

  log_file.print("\r\nM");
  log_file.write( (byte*) &timestamp, sizeof(uint32_t) );
  log_file.write( (byte*) &mx, sizeof(int16_t) );
  log_file.write( (byte*) &my, sizeof(int16_t) );
  log_file.write( (byte*) &mz, sizeof(int16_t) );
  log_file.write( (byte*) &mt, sizeof(int16_t) );
  
#else
    
  #ifdef MIRROR_SD_WRITES_TO_SERIAL
  
    Serial.print(timestamp);
    Serial.print(", m, ");
    Serial.print(mx); Serial.print(", ");
    Serial.print(my); Serial.print(", ");
    Serial.print(mz); Serial.print(", ");
    Serial.println(mt);
    
  #endif

    log_file.print(timestamp); 
    log_file.print(", m, ");
    log_file.print(mx); log_file.print(", ");
    log_file.print(my); log_file.print(", ");
    log_file.print(mz); log_file.print(", ");
    log_file.println(mt);
    
#endif

}

inline void print_BME280_data(
  float bme_temp,
  float bme_press,
  float bme_hum
) {
  
  uint32_t timestamp = millis();
  
#ifdef LOG_IN_BINARY_FORMAT

  #ifdef MIRROR_SD_WRITES_TO_SERIAL

  Serial.print("\r\nBME");
  Serial.write( (byte*) &timestamp, sizeof(uint32_t) );
  Serial.write( (byte*) &bme_temp, sizeof(float) );
  Serial.write( (byte*) &bme_press, sizeof(float) );
  Serial.write( (byte*) &bme_hum, sizeof(float) );

  #endif

  log_file.print("\r\nBME");
  log_file.write( (byte*) &timestamp, sizeof(uint32_t) );
  log_file.write( (byte*) &bme_temp, sizeof(float) );
  log_file.write( (byte*) &bme_press, sizeof(float) );
  log_file.write( (byte*) &bme_hum, sizeof(float) ); 
  
#else
    
  #ifdef MIRROR_SD_WRITES_TO_SERIAL
  
    Serial.print(timestamp);
    Serial.print(", tph, ");
    Serial.print(bme_temp, 3); Serial.print(", ");
    Serial.print(bme_press, 3); Serial.print(", ");
    Serial.println(bme_hum, 3);
    
  #endif

    log_file.print(timestamp); 
    log_file.print(", tph, ");
    log_file.print(bme_temp, 3); log_file.print(", ");
    log_file.print(bme_press, 3); log_file.print(", ");
    log_file.println(bme_hum, 3);
    
#endif

}

inline void print_GPS_data(
  char* dat_string
) {
  
  uint32_t timestamp = millis();
  
#ifdef LOG_IN_BINARY_FORMAT

  #ifdef MIRROR_SD_WRITES_TO_SERIAL

  Serial.print("\r\nGPS");
  Serial.write( (byte*) &timestamp, sizeof(uint32_t) );
  Serial.print(dat_string);

  #endif

  log_file.print("\r\nGPS");
  log_file.write( (byte*) &timestamp, sizeof(uint32_t) );
  log_file.print(dat_string); 
  
#else
    
  #ifdef MIRROR_SD_WRITES_TO_SERIAL
  
    Serial.print(timestamp);
    Serial.print(", GPS, ");
    Serial.println(dat_string);
    
  #endif

    log_file.print(timestamp);
    log_file.print(", GPS, ");
    log_file.println(dat_string);
    
#endif

}

int MPU6050_self_test(MPU6050 *mpu) {
  return 0;
}

inline String get_log_file_path() {
  return log_file_dir + log_file_name + String(log_file_number) + log_file_extension;
}

String parse_get_request(String line) {
  char* target = "GET";

  char s[200];
  strncpy(s, line.c_str(), sizeof(s)); 
  char delimeters[7] = " \t\r\n\v\f";
  char* token = strtok(s, delimeters);
  int take_next_token = 0;
  while (token != NULL) {
    if (take_next_token) {
      return String(token);
    }
    if (strcmp(token, target)==0) take_next_token = 1;
    token = strtok(NULL, delimeters);
  }

  return "";
}

void serve_main_page(WiFiClient &client) {
  // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  // and a content-type so the client knows what's coming, then a blank line:
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println();
  
  // the content of the HTTP response follows the header:
  int batteryVoltagemV = (analogRead(BATTERY_VOLTAGE_PIN)*BATTERY_VOLTAGE_MULT*3620) / 4096;
  client.print("Battery level: "); client.print(batteryVoltagemV); client.print(" mV<br>");
  client.print("Click <a href=\"/H\">here</a> to start logging.<br>");
  client.printf("accel/gyro: %d, mag: %d, bme: %d, log_file: %d<br>", accelgyro_init_ok, mag_init_ok, bme_init_ok, log_file_open);
  client.print("Click <a href=\"/L\">here</a> to clear the log file on the SD card.<br>");
  
  // The HTTP response ends with another blank line:
  client.println();
}

void serve_redirect_to_main_page(WiFiClient &client) {
  client.println("HTTP/1.1 307 Redirect");
  client.println("Location: /");
  client.println("Content-type:text/html");
  client.println("Connection:close");
  client.println("<html>Refreshing...</html>");
  client.println();
}
