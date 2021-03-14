/* autonomous snake robot with basic sensing capabilities*/
/*MEC825 - CAPSTONE*/
#include <Servo.h>
#include <PID_v1.h>

// Servo Motor Parameters
Servo servo1;                         // servo1 - move two phase memeber
Servo servo2;                         // servo2 - move two phase member
Servo servo3;                         // servo3 - orient snake upward
int pos = 0;                          // variable to store the servo position

//PID Parameters
double SetPoint, Input, Output;
double Kp = 1, Ki = 0.005, Kd = 0;    //subject to change bc of tuning
PID PID_control(&Input, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);


//Ultrasonic Sensor Parameters
// external ultrasonic sensor calibration
const int trigPin = 11;
const int echoPin = 12;
float duration;
double distance;
int distanceThreshold = 30;           //distance threshold is 30 cm - subject to change

//Mircophone Sensor Parameters
// external sound sensor calibration
int SoundSensor = A0;
double SoundThreshold;

// Time Constraints
long previousMillis = 0;
long interval = 10000;
unsigned long currentMillis = millis();
 
void setup() {
  servo1.attach(9);                   // attaches the servo on pin 9 to the servo object
  servo2.attach(10);                  // attaches the servo on pin 10 to the servo object
  servo3.attach(11);                  // attaches the servo on pin 10 to the servo object
  pinMode(trigPin, OUTPUT);           //ultrasonic sensor trig pin
  pinMode(echoPin, INPUT);            //ultrasonic sensor echo pin
  pinMode(soundSensor,INPUT);         //sound sensor analog - can also use digital pin
 
  //PID control
  PID_control.SetMode(AUTOMATIC);     
  PID_control.SetTunings(Kp, Ki, Kd);
  PID_control.SetOutputLimits(-255,255);

  // Serial Monitor
  Serial.begin(115200);                //change depending on baud rate of python radar code
  Serial.println("Radar Start");
  
  // Sound sensor threshold
  SoundThreshold = thresholdCalc();
}

void loop() {

  distance = calculateDistance();
  Input = distance;
  Setpoint = distanceThreshold
  PID_control.Compute();              //calculates the Ouput Value

  /*-----------------------------------------SERVO MOVE SNAKE FORWARD-------------------------------------------*/
 if( distance > distanceThreshold | distance < distanceThreshold){
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    servo2.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);
    distance = calculateDistance(); // Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree
    Serial.print("Servo 1 & 2 Angle:");
    Serial.print(pos);
    Serial.print("  ");
    delay(10);
    Serial.print("Distance from Target:");
    Serial.print(distance);
    Serial.print("  ");
    delay(10);
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    servo2.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);
    distance = calculateDistance(); // Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree
    Serial.print("Servo 1 & 2 Angle:");
    Serial.print(pos);
    Serial.print("  ");
    delay(10);
    Serial.print("Distance from Target:");
    Serial.print(distance);
    Serial.print("  ");
    delay(10);
    } 
 }

   /*----------------------------------------SWEEP SERRVO FOR RADAR-------------------------------------------------*/ 
 if(distance <= distanceThreshold  && SoundSensorval == SoundThreshold){ 
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo3.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);
    distance = calculateDistance(); // Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree
    Serial.print("Servo3 Angle:");
    Serial.print(pos);
    Serial.print("  ");
    delay(10);
    Serial.print("Distance from Target:");
    Serial.print(distance);
    Serial.print("  ");
    delay(10);
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo3.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);
    distance = calculateDistance(); // Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree
    Serial.print("Servo3 Angle:");
    Serial.print(pos);
    Serial.print("  ");
    delay(10);
    Serial.print("Distance from Target:");
    Serial.print(distance);
    Serial.print("  ");
    delay(10);
    }
  }
  /*---------------------------------serial plot to match input and setpoint for PID tuning--------------------------*/
  Serial.print("Input:");
  Serial.print(Input);
  Serial.print("  ");
  delay(10);
  
  Serial.print("Output:");
  Serial.print(Output);
  Serial.print("  ");
  delay(10);

  Serial.print("SetPoint:");
  Serial.println(SetPoint);
  delay(10);
  
}

float calculateDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

float thresholdCalc(){
  // account for the sound made by the motors to set as threshold
  while (currentMillis - previousMillis > interval){        // will run for 10 seconds
    int SoundSensorval = analogRead(SoundSensor);
    Serial.print("Sound Sensor(no motors):");
    Serial.print(SoundSensorval);                          //find sensor values for no moving motors
    Serial.print("\t");
    delay(10);
    int sensorMin = min(Serial.read());
  }
  
  delay(100);
  
  // move all three motors at the same time to find relative max threshold noise
  while (currentMillis - previousMillis > interval){        // will run for 10 seconds
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servo1.write(pos);              // tell servo to go to position in variable 'pos'
      servo2.write(pos);              // tell servo to go to position in variable 'pos'
      servo3.write(pos);              // tell servo to go to position in variable 'pos'
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      servo1.write(pos);              // tell servo to go to position in variable 'pos'
      servo2.write(pos);              // tell servo to go to position in variable 'pos'
      servo3.write(pos);              // tell servo to go to position in variable 'pos'
    }
    int SoundSensorval = analogRead(SoundSensor);
    Serial.print("Sound Sensor(with motors):");
    Serial.print(SoundSensorval);                          //find sensor values for no moving motors
    Serial.print("\t");
    delay(10);
    int sensorMax = max(Serial.read());
  }
  SoundThreshold = sensorMax + sensorMin;
  return SoundThreshold;
}
