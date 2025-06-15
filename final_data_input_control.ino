#include <Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

         

Servo m1, m2, m3, m4, m5, m6, m7,m8;
int targetAngles[8];
int servoAngles[8];
void moveServoToAngleSmooth(int targetAngle1, int targetAngle2, int targetAngle3, 
                            int targetAngle4, int targetAngle5, int targetAngle6,int targetAngle7,int targetAngle8, 
                            int durationMs) {
  int minPulseWidth = 543;
  int maxPulseWidth = 2500;

  int targetPulseWidth1 = map(targetAngle1, 0, 180, minPulseWidth, maxPulseWidth);
  int targetPulseWidth2 = map(targetAngle2, 0, 180, minPulseWidth, maxPulseWidth);
  int targetPulseWidth3 = map(targetAngle3, 0, 180, minPulseWidth, maxPulseWidth);
  int targetPulseWidth4 = map(targetAngle4, 0, 180, minPulseWidth, maxPulseWidth);
  int targetPulseWidth5 = map(targetAngle5, 0, 180, minPulseWidth, maxPulseWidth);
  int targetPulseWidth6 = map(targetAngle6, 0, 180, minPulseWidth, maxPulseWidth);
  int targetPulseWidth7 = map(targetAngle7, 0, 180, minPulseWidth, maxPulseWidth);
  

  int currentPulseWidth1 = m1.readMicroseconds();
  int currentPulseWidth2 = m2.readMicroseconds();
  int currentPulseWidth3 = m3.readMicroseconds();
  int currentPulseWidth4 = m4.readMicroseconds();
  int currentPulseWidth5 = m5.readMicroseconds();
  int currentPulseWidth6 = m6.readMicroseconds();
  int currentPulseWidth7 = m7.readMicroseconds();
  

  float stepDelay = 18;
  int totalSteps = durationMs / stepDelay;
  float increment = 1.8 / totalSteps;

  for (float t = 0.0; t <= 1.0; t += increment) {
    float easedValue = (t < 0.5) ? 4 * pow(t, 3) : 1 - pow(-2 * t + 2, 3) / 2;

    int newPulseWidth1 = currentPulseWidth1 + easedValue * (targetPulseWidth1 - currentPulseWidth1);
    int newPulseWidth2 = currentPulseWidth2 + easedValue * (targetPulseWidth2 - currentPulseWidth2);
    int newPulseWidth3 = currentPulseWidth3 + easedValue * (targetPulseWidth3 - currentPulseWidth3);
    int newPulseWidth4 = currentPulseWidth4 + easedValue * (targetPulseWidth4 - currentPulseWidth4);
    int newPulseWidth5 = currentPulseWidth5 + easedValue * (targetPulseWidth5 - currentPulseWidth5);
    int newPulseWidth6 = currentPulseWidth6 + easedValue * (targetPulseWidth6 - currentPulseWidth6);
    int newPulseWidth7 = currentPulseWidth7 + easedValue * (targetPulseWidth7 - currentPulseWidth7);
    

    m1.writeMicroseconds(newPulseWidth1);
    m2.writeMicroseconds(newPulseWidth2);
    m3.writeMicroseconds(newPulseWidth3);
    m4.writeMicroseconds(newPulseWidth4);
    m5.writeMicroseconds(newPulseWidth5);
    m6.writeMicroseconds(newPulseWidth6);
    m7.writeMicroseconds(newPulseWidth7);
    m8.write(targetAngle8);
   

    delay(stepDelay);
  }
}

void setup() {
  Serial.begin(9600);
    while (!Serial) delay(1);

  Wire.begin();

  if (!lox.begin()) {
    Serial.println(F("Failed to initialize VL53L0X sensor!"));
    while (1);
  }
  m1.attach(2);
  m2.attach(3);
  m3.attach(4);
  m4.attach(5);
  m5.attach(6);
  m6.attach(7);
  m7.attach(8);
  m8.attach(9);

 // Serial.println("Enter 6 angles separated by spaces (e.g., 90 45 120 60 30 150)");
    

}

void loop() {
  
 VL53L0X_RangingMeasurementData_t measure;
 lox.rangingTest(&measure, false);
 int rawDistance = (measure.RangeMilliMeter / 10.0) - 4;  // Convert mm to cm and calibrate
   Serial.println(rawDistance);  

  if (Serial.available()) { 

    String input = Serial.readStringUntil('\n');
    int values[9];
    int index = 0;
    char *token = strtok((char *)input.c_str(), " ");

    while (token != NULL && index < 9) {
      values[index] = atoi(token);
      token = strtok(NULL, " ");
      index++;
    }

    if (index == 9) {
      for (int i = 0; i < 9; i++) {
        targetAngles[i] = values[i];
      }
      moveServoToAngleSmooth(targetAngles[0], targetAngles[1], targetAngles[2],
                             targetAngles[3], targetAngles[4], targetAngles[5],targetAngles[6], targetAngles[7],targetAngles[8]);
    }
    else if (index == 7){
      for (int i =0; i <7; i++){
        servoAngles[i] = values[i];   
        servoAngles[i] = map(servoAngles[i], 0, 180, 543, 2500);
      }
       m5.writeMicroseconds(servoAngles[1]);
       m6.writeMicroseconds(servoAngles[2]);
       m1.writeMicroseconds(servoAngles[0]);
       m2.writeMicroseconds(servoAngles[3]);
       m3.writeMicroseconds(servoAngles[4]);
       m4.writeMicroseconds(servoAngles[5]);
       m7.writeMicroseconds(servoAngles[6]);


    }
    else if (index == 3) {
      for (int i = 0; i<3; i++){
        servoAngles[i] = values[i];
        servoAngles[i] = map(servoAngles[i], 0, 180, 543, 2500);
        }
       m5.writeMicroseconds(servoAngles[1]);
       m6.writeMicroseconds(servoAngles[2]);
       m1.writeMicroseconds(servoAngles[0]);

    }
    }
  }
