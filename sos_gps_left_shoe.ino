//Obstacle detection+ stairs(up and down)+ GPS + SOS//

// LIBRARIES
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h> //For GPS
#include <WiFi.h>       
#include <HTTPClient.h>  //For Telegram


//Pin definitions and variables (ultrasound + tof)
#define TRIG 23 //ultrasound
#define ECHO 19 //ultrasound
#define XSHUT_PIN_2 32 //TOF (VL53L0X)
#define SENSOR_1_ADDRESS 0x30 //TOF (VL53L0X)

Adafruit_MPU6050 mpu; 
VL53L0X sensor2;

int buzzer_pin = 4;
float gx, gy, gz;
float thresold = 1.2;
float mag;
float initializing_count = 200;  //10secs 10000/final_delay (10000/50 = 200)
float count = 0;
int count_obs = 0;
int count_nor = 0;

float tof2;             
float normal_tof2 = 0;  //do +-60 //491.2 //used to measure normal distance of ground

int max_count = 10;  // 600/final_delay (rn its 50 so 500/50 = 10)
int count_stair_up_tof2 = 0;
int count_stair_down_tof2 = 0;  //used to check till we are sure about stair up or stair down

unsigned long buzzer_off_time = 0;
bool buzzer_is_on = false;

bool just_returned_to_normal = true;  //Tracks if we've just landed on normal ground

long duration;
float distance;


//Pin definitions and variables  (GPS + Wifi)

TinyGPSPlus gps;
#define gpsSerial Serial2 // Using hardware serial 2 (Pins TX2/RX2 on ESP32)

const char* ssid = "xxxxx"; //for internet connectivity
const char* password = "xxxxx";
String botToken = "8566007978:AAG3wkX0U6zmp0ZqqUc0dHlXeMgD9rg4UsM"; //Bot token
String chatID   = "8261488047"; //Chat ID

int buttonPin = 34; //Pushbutton pin for SOS
int buttonNew; //Variable to store current state of pushbutton
int buttonOld = 1; //Variable to store previous state of pushbutton. 0 when pressed and 1 when not pressed

unsigned long lastTelegramSend = 0;
const long telegramInterval = 2*60*1000; //2min delay


//Sending message on telegram
void sendTelegramMessage(String text) {
  if (WiFi.status()!= WL_CONNECTED) { //Check for wifi connectivity
    Serial.println("WiFi not connected. Cannot send Telegram message.");
    return;
  }

  //URL-encode the text to handle special characters like newlines
  String urlEncodedText = text;
  urlEncodedText.replace("\n", "%0A"); 

  String url = "https://api.telegram.org/bot" + botToken +
               "/sendMessage?chat_id=" + chatID +
               "&text=" + urlEncodedText;

  HTTPClient http; //Create the HTTP Client object
  http.begin(url);
  int httpResponseCode = http.GET(); //Send request

  //Check response
  if (httpResponseCode > 0) {
    Serial.print("Telegram response code: ");
    Serial.println(httpResponseCode);
  } 
  else {
    Serial.print("Telegram error code: ");
    Serial.println(httpResponseCode);
  }

  http.end(); //Close the connection
}

//Printing GPS Data to serial monitor
void displayLocationInfo() {
  Serial.println(F("-------------------------------------"));
  Serial.println(F("\n Location Info (Serial Monitor):"));

  Serial.print("Latitude:   ");
  Serial.print(gps.location.lat(), 6);
  Serial.print(" ");
  Serial.println(gps.location.rawLat().negative ? "S" : "N");

  Serial.print("Longitude:  ");
  Serial.print(gps.location.lng(), 6);
  Serial.print(" ");
  Serial.println(gps.location.rawLng().negative ? "W" : "E");

  Serial.print("Fix Quality: ");
  Serial.println(gps.location.isValid() ? "Valid" : "Invalid");

  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());

  Serial.println(F("-------------------------------------"));
}

// Helper Functions (Buzzer & Sensors)

void triggerBuzzer(long duration) {
  if (!buzzer_is_on) {  //Only trigger if it's not already on
    buzzer_is_on = true;
    digitalWrite(buzzer_pin, LOW);           //Turn buzzer ON
    buzzer_off_time = millis() + duration;  //Set when to turn it off
  }
}

void beep(int on_ms, int off_ms) {
  digitalWrite(buzzer_pin, LOW);  //Buzzer ON
  delay(on_ms);
  digitalWrite(buzzer_pin, HIGH);  //Buzzer OFF
  delay(off_ms);
}

void beepStairDown() {//3 beeps
  Serial.println("BEEP - STAIR DOWN (DANGER)");
  beep(250, 250);  // 250ms ON, 250ms OFF
  beep(250, 250); 
  beep(250, 250);
}

void beepStairUp() {//2 beeps
  Serial.println("BEEP - STAIR UP");
  beep(333, 333);  // 333ms ON, 333ms OFF
  beep(333, 333);  
}

void beepNormalGround() {//1 long beep
  Serial.println("BEEP - NORMAL GROUND");
  beep(1000, 40);  // 1000ms ON, 40ms OFF
}

void beepObstacle() {//5 beeps
  Serial.println("BEEP - OBSTACLE");
  beep(150, 150);  // 150ms ON, 150ms OFF
  beep(150, 150);
  beep(150, 150);
  beep(150, 150);
  beep(150, 150);
}


//Setup function
void setup() {
  Serial.begin(115200);
  
  //Wifi setup
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  //GPS setup
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("GPS Serial started on pins 16 & 17.");
  
  //Pins setup
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, HIGH);
  
  //Setup button pin for pushbutton
  pinMode(buttonPin, INPUT);
  Serial.println("Button pin (34) initialized.");

  Wire.begin();

  while (!Serial)
    delay(10);  //will pause Zero, Leonardo, etc until serial console opens

  //MPU6050 Setup
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
     }
  }

  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  //The accelerometer can measure forces up to ±8 Gs
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);       //The gyroscope can measure rotational speeds up to ±500 degrees per second.
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    //The sensor will automatically ignore (filter out) high-frequency noise and vibrations faster than 21 Hz.
  Serial.println("IMU Sensor configured.");
  delay(100);

  // VL53L0X Setup
  // Set XSHUT pins as outputs
  pinMode(XSHUT_PIN_2, OUTPUT);
  //Pull both XSHUT pins LOW to put both sensors in shutdown mode
  Serial.println(F("Both sensors in shutdown mode..."));
  digitalWrite(XSHUT_PIN_2, LOW);
  delay(100);

  Serial.println(F("Bringing Sensor 2 online..."));
  //Wake up Sensor 2
  pinMode(XSHUT_PIN_2, INPUT);
  delay(10); 
  //Initialize Sensor 2 (it will use the default 0x29 address)
  sensor2.init(true);
  delay(100);
  Serial.println(F("Sensor 2 booted at default 0x29"));

  //Set high-speed mode
  sensor2.startContinuous();
  Serial.println(F("\nAll systems ready! Starting calibration..."));
  Serial.println("Waiting for GPS fix and satellites...");
}



void loop() {
  //Track GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // Display info when a new sentence is parsed
      displayLocationInfo();
    }
  }

  //Sendor Readings for Stairs and Obstacle Detection
  
  if (buzzer_is_on && (millis() >= buzzer_off_time)) {
    digitalWrite(buzzer_pin, HIGH);  //turn buzzer OFF
    buzzer_is_on = false;
  }

  //Read ultrasonic
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = duration * 0.034 / 2;

  //Read MPU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
  mag = sqrt(gx * gx + gy * gy + gz * gz);

  //Read Tof
  tof2 = sensor2.readRangeContinuousMillimeters() / 10.00;

  //Calibaration
  if (count < initializing_count && count >= 0) {
    if (tof2 < 800) {
      count++;
      Serial.println(tof2);
      normal_tof2 += tof2;
    }
  }
  else if (count == initializing_count) {
    normal_tof2 = normal_tof2 / initializing_count;
    count = -1;
    Serial.print(F("\nCalibration complete. Normal Tof2: "));
    Serial.println(normal_tof2);
  }
  else {
    if (mag >= thresold) { //Swing Phase
      count_stair_up_tof2 = 0;
      count_stair_down_tof2 = 0;
      count_obs = 0;
    } 
    else { //Static phase

      //Check for Stair Down
      if (tof2 >= normal_tof2 + 60.00) {
        Serial.print("down  ");
        Serial.print(distance);
        Serial.print("    ");
        Serial.println(tof2);
        count_stair_down_tof2++;
        count_stair_up_tof2 = 0;
        count_obs = 0;

        if (count_stair_down_tof2 >= max_count) {
          Serial.println("STAIR DOWN");
          count_nor = 0;
          just_returned_to_normal = true;

            beepStairDown();
           
          count_stair_up_tof2 = 0;
          count_stair_down_tof2 = 0;
        }
      }
      //Check for Stair up
      else if (tof2 <= 0.55 * normal_tof2) {
        Serial.print("up  ");
        Serial.print(distance);
        Serial.print("    ");
        Serial.println(tof2);
        count_stair_up_tof2++;
        count_stair_down_tof2 = 0;
        count_obs = 0;

        if (count_stair_up_tof2 >= max_count) {          
            Serial.println("STAIR UP");
            
              beepStairUp();            
            just_returned_to_normal = true;
          count_stair_up_tof2 = 0;
          count_stair_down_tof2 = 0;
        }
      }
      //Check for Normal Ground/ Obstacle
      else {
        count_stair_up_tof2 = 0;
        count_stair_down_tof2 = 0;
        bool isObstacle = (distance >= tof2 - 4.50) && (distance <= tof2 + 7.00);

        if (isObstacle) {
          count_obs++;
          count_nor = 0;
          if (count_obs >= max_count) {
            Serial.print("OBSTACLE-----");
            Serial.print(distance);
            Serial.print("    ");
            Serial.println(tof2);

            beepObstacle();
            just_returned_to_normal = true; 
          }
        } 
        else {
          count_obs = 0;
          digitalWrite(buzzer_pin, HIGH);
          count_nor++;
          if (count_nor >= max_count && just_returned_to_normal) {
            beepNormalGround();
            just_returned_to_normal = false;
            Serial.println("RETURNED TO NORMAL");
            count_nor = 0;
          }
          Serial.print("normal ");
          Serial.print(distance);
          Serial.print("    ");
          Serial.println(tof2);
        }
      }
    }
  }
  
  //Check for Pushbutton press (SOS)
  buttonNew = digitalRead(buttonPin);
  if (buttonOld == 0 && buttonNew == 1) { //Message is sent everytime the pushbutton is released
    Serial.println("Button pressed! Sending message to Telegram.");

    //Create a string message that is to be sent over telegram
    String message = "ALERT!!! The person is requesting for help!! Current Location:\n";
      message += "Latitude: " + String(gps.location.lat(), 6);
      message += (gps.location.rawLat().negative ? " S" : " N");
      message += "\n";
      message += "Longitude: " + String(gps.location.lng(), 6);
      message += (gps.location.rawLng().negative ? " W" : " E");
      
      String mapsLink = "https://www.google.com/maps/search/?api=1&query="
                    + String(gps.location.lat(), 6) + "%2C"
                    + String(gps.location.lng(), 6);

    //Attach the google maps link
    message += "\nGoogle Maps: " + mapsLink;
   
    sendTelegramMessage(message);
  }
  buttonOld = buttonNew; //Update the pushbutton state

  //Check timer for location update
  unsigned long currentMillis = millis();
  if (currentMillis - lastTelegramSend >= telegramInterval) {
    lastTelegramSend = currentMillis; //Reset the timer

    Serial.println("2-minute timer elapsed. Checking for location data...");

    if (gps.location.isValid()) { //Check if location data is valid
      Serial.println("Valid location found. Sending to Telegram.");
      
      String message = "Location Update:\n"; //Create a message string to send on telegram
      message += "Latitude: " + String(gps.location.lat(), 6);
      message += (gps.location.rawLat().negative ? " S" : " N");
      message += "\n";
      message += "Longitude: " + String(gps.location.lng(), 6);
      message += (gps.location.rawLng().negative ? " W" : " E");

      sendTelegramMessage(message);
      
    } else {
      Serial.println("Location data is not valid yet. Skipping Telegram send."); //Do not send location updates till location data is valid
    }
  }

  //Check GPS hardware to avoid spam
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
  }

  delay(50); 
}