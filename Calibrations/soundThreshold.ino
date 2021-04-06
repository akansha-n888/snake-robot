#include <Servo.h>

//Mircophone Sensor Parameters
// external sound sensor calibration
int SoundSensor = A0;
double SoundThreshold;               // threshold value from sound threshold calibration -- 77 no motors, 900 with motors (subject to change)
double SoundSensorvalNM,SoundSensorvalM;

// Servo Motor Parameters
Servo servo1;                         // servo1 - move two phase memeber
Servo servo2;                         // servo2 - move two phase member
Servo servo3;                         // servo3 - orient snake upward
int pos = 0;                          // variable to store the servo position


void setup() {
  servo1.attach(9);                   // attaches the servo on pin 9 to the servo object
  servo2.attach(10);                  // attaches the servo on pin 10 to the servo object
  servo3.attach(11);                  // attaches the servo on pin 10 to the servo object

  
  // Serial Monitor
  Serial.begin(9600); 

}

void loop() {

/*-------------------------------------- NO MOVING MOTORS---------------------------------------------

    SoundSensorvalNM = analogRead(SoundSensor);
    Serial.print("Sound Sensor(no motors):");
    Serial.println(SoundSensorvalNM);                          //find sensor values for no moving motors
    Serial.print("\t");
    delay(10);

  
/*--------------------------------------MOVING MOTORS-----------------------------------------------*/
    for (pos = 0; pos <= 180; pos += 10) { // goes from 0 degrees to 180 degrees
      servo1.write(pos);              // tell servo to go to position in variable 'pos'
      servo2.write(pos);              // tell servo to go to position in variable 'pos'
      servo3.write(pos);              // tell servo to go to position in variable 'pos'
    }
    for (pos = 180; pos >= 0; pos -= 10) { // goes from 180 degrees to 0 degrees
      servo1.write(pos);              // tell servo to go to position in variable 'pos'
      servo2.write(pos);              // tell servo to go to position in variable 'pos'
      servo3.write(pos);              // tell servo to go to position in variable 'pos'
    }
    SoundSensorvalM = analogRead(SoundSensor);
    Serial.print("Sound Sensor(with motors):");
    Serial.println(SoundSensorvalM);                          //find sensor values for no moving motors
    Serial.print("\t");
    delay(10);
  
  
  int sensorMax = max(SoundSensorvalNM,SoundSensorvalM);
  int sensorMin = min(SoundSensorvalNM,SoundSensorvalM);
  SoundThreshold = sensorMax + sensorMin;
  Serial.print("Sound Sensor Min:");
  Serial.print(sensorMin);                          
  Serial.print("\t");
  Serial.print("Sound Sensor Max:");
  Serial.print(sensorMax);                          
  Serial.print("\t");
  Serial.print("Sound Sensor Threshold:");
  Serial.println(SoundThreshold);                          
  Serial.print("\t");

}



  

  
