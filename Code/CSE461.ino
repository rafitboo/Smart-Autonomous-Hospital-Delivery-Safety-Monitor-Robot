#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

// --- PIN DEFINITIONS ---
#define RST_PIN 7 
#define SS_PIN 10
MFRC522 mfrc522(SS_PIN, RST_PIN);

#define TRIG_PIN 4 
#define ECHO_PIN 8
#define SERVO_PIN 2
Servo cargoLock;

#define SMOKE_PIN 0 

// IR Line Tracking Pins
#define IR1 1  
#define IR2 A0
#define IR3 A1
#define IR4 A2
#define IR5 A3 

// Motor Driver Pins (PWM Enabled)
#define IN1 3 // Left Motor Reverse
#define IN2 5 // Left Motor Forward
#define IN3 6 // Right Motor Forward
#define IN4 9 // Right Motor Reverse

#define PCF_ADDRESS 0x20
LiquidCrystal_I2C lcd(0x27, 16, 2);

int currentState = 0; 
String destination = "";
byte pcfOutput = 0xFF; 
unsigned long lastSonarPing = 0;
unsigned long obstacleTimer = 0; 
int distance = 999;
bool passedMainJunction = false; 
bool currentAlarmState = false; 

unsigned long turnCooldown = 0; 
bool isTurning = false; 

// Base motor speeds (Compensating for heavier right side)
int leftDriveSpeed = 100;  
int rightDriveSpeed = 130; // Increase if it still drifts right, decrease if it overcompensates left

// --- PID CONTROL VARIABLES ---
float Kp = 35.0; // Adjust this if the robot wobbles side-to-side
float Ki = 0.0;  // Leave at 0 for standard line following
float Kd = 15.0; // Adjust this to smooth out sudden jerks
float P = 0, I = 0, D = 0, previousError = 0;

// RFID Coordinate UIDs
String mainJunctionUID = "8BA2AD05"; 
String wardAUID = "C09D035C";
String wardBUID = "B0B6175C";
String wardCUID = "B06C5B5C"; 

void setup() {
  Wire.begin();
  Wire.beginTransmission(PCF_ADDRESS);
  Wire.write(0xFF); 
  Wire.endTransmission();
  
  lcd.init();
  lcd.backlight();
  SPI.begin();
  mfrc522.PCD_Init();
  cargoLock.attach(SERVO_PIN);
  
  // --- Box opens initially when switched on ---
  cargoLock.write(90); 
  
  pinMode(SMOKE_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  motorStop(); 

  lcd.setCursor(0,0);
  lcd.print("Testing RFID...");
  delay(1000);
  
  byte version = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  if (version == 0x00 || version == 0xFF) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("RFID ERROR!");
    lcd.setCursor(0,1);
    lcd.print("Check Pins!");
    while(1); 
  } else {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("RFID OK!");
    delay(1000);
  }
}

void loop() {
  Wire.requestFrom(PCF_ADDRESS, 1);
  byte pcfState = 0xFF;
  if (Wire.available()) { pcfState = Wire.read(); }

  bool btnWardA = !(pcfState & 0x01);        
  bool btnWardB = !(pcfState & 0x02);        
  bool btnWardC = !(pcfState & 0x04);        
  bool btnConfirmReset = !(pcfState & 0x10); 

  if (millis() - lastSonarPing > 50) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
    distance = (duration == 0) ? 999 : duration * 0.034 / 2;
    lastSonarPing = millis();
  }

  // --- STRICT BLACK LINE LOGIC ---
  bool L_far = false;
  bool L_mid = false;
  bool Center = false;
  bool R_mid = false;
  bool R_far = false;

  // Temporarily disable sensor reading if the robot is currently executing a turn
  if (!isTurning) {
    L_far  = (digitalRead(IR1) == LOW);
    L_mid  = (digitalRead(IR2) == LOW);
    Center = (digitalRead(IR3) == LOW);
    R_mid  = (digitalRead(IR4) == LOW);
    R_far  = (digitalRead(IR5) == LOW);
  }

  bool smokeDetected = (digitalRead(SMOKE_PIN) == LOW);
  bool flameDetected = (pcfState & 0x80); 
  bool obstacleAlarm = (currentState == 2 && obstacleTimer > 0 && (millis() - obstacleTimer > 2000));
  
  bool criticalAlarm = (smokeDetected || flameDetected);
  bool targetAlarmState = (criticalAlarm || obstacleAlarm);

  if (targetAlarmState != currentAlarmState) {
    currentAlarmState = targetAlarmState;
    if (currentAlarmState) {
      pcfOutput &= ~(1 << 5); 
      pcfOutput &= ~(1 << 6); 
    } else {
      pcfOutput |= (1 << 5);  
      pcfOutput |= (1 << 6);  
    }
    pcfOutput |= 0x97; 
    Wire.beginTransmission(PCF_ADDRESS);
    Wire.write(pcfOutput);
    Wire.endTransmission();
  }

  switch (currentState) {
    
    case 0: // --- IDLE STATE ---
      if (criticalAlarm) {
        updateDisplay("EMERGENCY ALERT!", "Fire/Smoke Det!!");
      } else {
        updateDisplay("Select Dest:", "System Ready");
      }
      motorStop(); 
      
      // Allows you to manually open or close the box while idle using the Confirm Button
      if (btnConfirmReset) {
        if (cargoLock.read() == 90) { // If it is currently open
          cargoLock.write(0);         // Close it
        } else {                      // If it is currently closed
          cargoLock.write(90);        // Open it
        }
        delay(500); // Mechanical debounce to prevent rapid toggling
      }
      
      if (btnWardA) { destination = "Ward A"; }
      else if (btnWardB) { destination = "Ward B"; }
      else if (btnWardC) { destination = "Ward C"; }
      
      if (destination != "") {
        // Box auto-closes when a ward button is pressed
        cargoLock.write(0); 
        delay(500); 
        
        passedMainJunction = false; 
        currentState = 1; 
      }
      break;

    case 1: // --- DRIVING STATE ---
      if (criticalAlarm) {
        updateDisplay("EMERGENCY ALERT!", "Fire/Smoke Det!!");
        motorStop(); 
      } else {
        updateDisplay("Moving: " + destination, "Tracking Line...");
        
        if (distance > 2 && distance < 20) {
          currentState = 2; 
        }
        // 1. MAIN JUNCTION CHECK (Cross-line / LEDs OFF)
        else if (!passedMainJunction && L_mid && Center && R_mid) {
          currentState = 3; 
        }
        // 2. DESTINATION CHECK (End of line / All 5 LEDs ON)
        else if (passedMainJunction && (millis() - turnCooldown > 5000) && !L_far && !L_mid && !Center && !R_mid && !R_far) {
          currentState = 3; 
        }
        // 3. NORMAL LINE TRACKING (PID CONTROL)
        else if (L_far || L_mid || Center || R_mid || R_far) {
          
          // Calculate average error based on active sensors
          int activeSensors = L_far + L_mid + Center + R_mid + R_far;
          float error = ((L_far * -2.0) + (L_mid * -1.0) + (Center * 0.0) + (R_mid * 1.0) + (R_far * 2.0)) / activeSensors;
          
          // Compute PID
          P = error;
          I = I + error;
          D = error - previousError;
          previousError = error;
          
          int PID_value = (Kp * P) + (Ki * I) + (Kd * D);
          
          // Apply differential speeds from the corrected baselines
          int leftSpeed = constrain(leftDriveSpeed + PID_value, 0, 255);
          int rightSpeed = constrain(rightDriveSpeed - PID_value, 0, 255);
          
          motorPID(leftSpeed, rightSpeed);
        }
        else {
          motorStop(); 
        }
      }
      break;

    case 2: // --- OBSTACLE HALT ---
      if (criticalAlarm) {
        updateDisplay("EMERGENCY ALERT!", "Fire/Smoke Det!!");
      } else {
        updateDisplay("OBSTACLE AHEAD!", "Motors Halted");
      }
      motorStop(); 
      
      if (obstacleTimer == 0) {
        obstacleTimer = millis(); 
      } 

      if (distance > 25 || distance == 999) {
        obstacleTimer = 0; 
        currentState = 1; 
      }
      break;

    case 3: // --- JUNCTION & RFID ROUTING ---
          motorStop(); 
          
          if (criticalAlarm) {
            updateDisplay("EMERGENCY ALERT!", "Fire/Smoke Det!!");
          } else if (!passedMainJunction) {
            updateDisplay("Junction Reached", "Scanning RFID...");
          } else {
            updateDisplay("Dest. Reached?", "Scanning RFID...");
          }
          
          mfrc522.PCD_Init(); 
          
          if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            String readUID = "";
            for (byte i = 0; i < mfrc522.uid.size; i++) {
              if(mfrc522.uid.uidByte[i] < 0x10) readUID += "0";
              readUID += String(mfrc522.uid.uidByte[i], HEX);
            }
            readUID.toUpperCase();
            mfrc522.PICC_HaltA(); 
            
            // --- NEW: Beep once when any card is successfully scanned ---
            rfidBeep();
            
            updateDisplay("Tag Scanned:", readUID);
            delay(1500);

            if (readUID == mainJunctionUID) {
              isTurning = true; // FLAG ACTIVATED: Stop reading IR sensors
              
              if (destination == "Ward A") {
                updateDisplay("At Main Junct", "Aligning Left...");
                motorForward();
                delay(420); 
                motorTurnLeft();
                delay(950); 
                
                while (digitalRead(IR3) == LOW) { } // Hunt for line
                
                motorStop();
                delay(300); // Settling delay to kill momentum
              } 
              else if (destination == "Ward B") {
                updateDisplay("At Main Junct", "Going Straight");
                motorForward();
                delay(300); 
              } 
              else if (destination == "Ward C") {
                updateDisplay("At Main Junct", "Aligning Right...");
                motorForward();
                delay(420); 
                motorTurnRight();
                delay(730); 
                
                while (digitalRead(IR3) == LOW) { } // Hunt for line
                
                motorStop();
                delay(300); // Settling delay to kill momentum
              }
              
              isTurning = false; // FLAG DEACTIVATED: Resume IR sensor readings
              passedMainJunction = true; 
              turnCooldown = millis(); // Starts the 5-second safety timer
              
              // Reset PID parameters for a clean start on the new line
              P = 0; I = 0; D = 0; previousError = 0;
              
              currentState = 1; 
            } 
            else if ((readUID == wardAUID && destination == "Ward A") ||
                    (readUID == wardBUID && destination == "Ward B") ||
                    (readUID == wardCUID && destination == "Ward C")) {
              currentState = 4; 
            } 
            else {
              updateDisplay("UID NOT MATCHED", "Invalid Route");
              delay(2000);
            }
          }
          break;

    case 4: // --- ARRIVED STATE ---
      if (criticalAlarm) {
        updateDisplay("EMERGENCY ALERT!", "Fire/Smoke Det!!");
      } else {
        updateDisplay("Arrived: " + destination, "Press ConfirmBtn");
      }
      motorStop(); 
      cargoLock.write(90); // Box opens when destination is reached
      
      if (btnConfirmReset) { 
        // Box closes when the confirm button is pressed
        cargoLock.write(0); 
        delay(500); 
        
        destination = "";
        currentState = 0; 
      }
      break;
  }
}

// ---------------------------------------------------------
// MOTOR & AUXILIARY FUNCTIONS 
// ---------------------------------------------------------
void rfidBeep() {
  // Temporarily triggers the PCF8574 alarm pins (Buzzer/LED) for a 150ms confirmation beep
  byte beepState = pcfOutput & ~(1 << 5) & ~(1 << 6); 
  Wire.beginTransmission(PCF_ADDRESS);
  Wire.write(beepState);
  Wire.endTransmission();
  
  delay(150); 
  
  Wire.beginTransmission(PCF_ADDRESS);
  Wire.write(pcfOutput); // Restores normal output state
  Wire.endTransmission();
}

void motorPID(int leftSpeed, int rightSpeed) {
  analogWrite(IN1, leftSpeed); // Left Forward
  analogWrite(IN2, 0);         // Left Reverse OFF
  analogWrite(IN3, rightSpeed);// Right Forward
  digitalWrite(IN4, LOW);      // Right Reverse OFF
}

void motorForward() {
  analogWrite(IN1, leftDriveSpeed);  // Left Forward
  analogWrite(IN2, 0);               // Left Reverse OFF
  analogWrite(IN3, rightDriveSpeed); // Right Forward
  analogWrite(IN4, 0);               // Right Reverse OFF
}

void motorStop() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void motorTurnLeft() { 
  analogWrite(IN1, 0);               // Left Forward OFF
  analogWrite(IN2, 0);               // Left Reverses
  analogWrite(IN3, rightDriveSpeed); // Right Forwards
  analogWrite(IN4, 0);               // Right Reverse OFF
}

void motorTurnRight() { 
  analogWrite(IN1, leftDriveSpeed);  // Left Forwards
  analogWrite(IN2, 0);               // Left Reverse OFF
  analogWrite(IN3, 0);               // Right Forward OFF
  digitalWrite(IN4, LOW);            // Right Reverses (bypasses Timer 1 PWM lock)
}

void updateDisplay(String row0, String row1) {
  while(row0.length() < 16) row0 += " ";
  while(row1.length() < 16) row1 += " ";
  static String lastRow0 = "";
  static String lastRow1 = "";
  if (row0 != lastRow0 || row1 != lastRow1) {
    lcd.setCursor(0, 0);
    lcd.print(row0);
    lcd.setCursor(0, 1);
    lcd.print(row1);
    lastRow0 = row0;
    lastRow1 = row1;
  }
}