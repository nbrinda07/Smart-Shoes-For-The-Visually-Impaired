#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <MFRC522.h>

//Pin Definitions
#define TRIG 26  
#define ECHO 27  
#define XSHUT_PIN_2 32
#define RST_PIN 33    
#define SS_PIN 5      
#define BUZZER_PIN_RFID 4 

//Objects & Variables
Adafruit_MPU6050 mpu;
MFRC522 mfrc522(SS_PIN, RST_PIN);

#define SENSOR_1_ADDRESS 0x30

//IMU and Motion Variables
float gx, gy, gz;
float thresold = 1.2; //Sensitivity for foot movement detection
float mag;            //Magnitude of gyroscope vector
float initializing_count = 200;
float count = 0;

//Logic Counters
int count_obs = 0;    //Counter to confirm obstacle persistence
int count_nor = 0;    //Counter to confirm normal ground return
int max_count = 10;   //Threshold for confirming state

bool just_returned_to_normal = true; //State flag to trigger "Safe" beep

//Ultrasonic Variables
long duration;
float distance;       //Distance in cm

//RFID Anti-Spam Variables
String lastTagScanned = "";
unsigned long lastScanTime = 0;
const long scanDelay = 3000; //3-second delay before re-reading same tag

//Audio Feedback Functions

//Plays a tone when returning to flat ground
void beepNormalGround() {
  Serial.println("BEEP - NORMAL GROUND");
  playTone(2100, 1000);//2100 Hz for 1000ms
}

//Plays rapid beeps when an obstacle is close
void beepObstacle() {
  Serial.println("BEEP - OBSTACLE");
  playTone(2100, 150);//2100Hz for 150ms
  delay(150);//150ms pause
  playTone(2100, 150);
  delay(150);
  playTone(2100, 150);
  delay(150);
  playTone(2100, 150);
  delay(150);
  playTone(2100, 150);
}

//RFID Helper functions

//Converts binary UID to a String for comparison
String getTagUID(byte *uid, byte uidSize) {
  String uidString = "";
  for (byte i = 0; i < uidSize; i++) {
    if (i > 0) { uidString += " "; }
    if (uid[i] < 0x10) { uidString += "0"; }
    uidString += String(uid[i], HEX);
  }
  uidString.toUpperCase();
  return uidString;
}

//wrapper for ledcWriteTone (ESP32 Buzzer function)
void playTone(int frequency, int duration) {
  ledcWriteTone(BUZZER_PIN_RFID, frequency);
  delay(duration);
  ledcWriteTone(BUZZER_PIN_RFID, 0);
}

//Location Patterns

// Pattern 1: Two long beeps
void playPattern1() {
  playTone(900, 1000);//900Hz for 1000ms
  delay(750);//750ms pause
  playTone(900, 1000);
}

//Pattern 2: One very long beep
void playPattern2() {
  playTone(900, 2000);//900Hz for 2000ms
}

//Pattern 3: Three medium beeps
void playPattern3() {
  playTone(900, 750);//900Hz for 750ms
  delay(250);//250ms pause
  playTone(900, 750);
  delay(250);
  playTone(900, 750);
}


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  //Init Pins & I2C
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Wire.begin();

  //Init MPU6050 (IMU)
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("IMU Sensor configured.");
  delay(100);

  //Init Buzzer & RFID
  ledcAttach(BUZZER_PIN_RFID, 5000, 8); //Attach buzzer pin

  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println("RFID Reader Initialized.");
  Serial.println("Scanning for tags...");
}

void loop() {
  //RFID Scanning Logic
  //Check if a new card is present
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      String currentTagID = getTagUID(mfrc522.uid.uidByte, mfrc522.uid.size);
      Serial.print("Scanned Tag: ");
      Serial.println(currentTagID);

      //Anti-Spam: Ignore if same tag scanned recently
      if (currentTagID == lastTagScanned && (millis() - lastScanTime < scanDelay)) {
        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
      } else {
          //Check Tag ID and play corresponding sound
          if (currentTagID == "73 77 95 39") {
            Serial.println("Location: Room 1");
            playPattern1();
          } else if (currentTagID == "A3 93 9C 39") {
            Serial.println("Location: Room 2");
            playPattern2();
          } else if (currentTagID == "03 C0 A7 39") {
            Serial.println("Location: Room 3");
            playPattern3();
          }
          //Update tracking variables
          lastTagScanned = currentTagID;
          lastScanTime = millis();

          //Stop reading current card
          mfrc522.PICC_HaltA();
          mfrc522.PCD_StopCrypto1();
      }
  }

  //Ultrasonic Distance Measurement
  //Send 10us trigger pulse
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  //Read echo duration and convert to cm
  duration = pulseIn(ECHO, HIGH, 30000);
  distance = duration * 0.034 / 2;

  //IMU Motion Detection
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;

  //Calculate movement magnitude
  mag = sqrt(gx * gx + gy * gy + gz * gz);

    //Main Logic Decision
    //If magnitude is high, foot is swinging/moving
    if (mag >= thresold) {
      count_obs = 0; //Reset obstacle counter while moving
    }
    else {
        //Foot is static (on ground) - Check for obstacles
        //Check if object is within danger zone (10cm to 35cm)
        if (distance <= 35.00  && distance >= 10.00) {
          count_obs++;
          count_nor = 0;
          //If obstacle persists for enough cycles, trigger alarm
          if (count_obs >= max_count) {
            Serial.print("OBSTACLE-----");
            Serial.println(distance);
            beepObstacle();
            just_returned_to_normal = false;
          }
        }
        else {
          //No obstacle detected (Normal Ground)
          count_obs = 0;
          count_nor++;
          //Play "Safe" beep if we just finished an obstacle event
          if (count_nor >= max_count && just_returned_to_normal) {
            beepNormalGround();
            just_returned_to_normal = false;
            Serial.println("RETURNED TO NORMAL");
            count_nor = 0;
          }
          Serial.print("normal ");
          Serial.println(distance);
        }
    }

  delay(50); //Loop delay
}