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
double Kp = 0.89, Ki = 0.005, Kd = 0.007;    //subject to change bc of tuning

PID PID_control(&Input, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);

// Response Time Parameters
unsigned long myTime;


//Ultrasonic Sensor Parameters
// external ultrasonic sensor calibration
const int trigPin = 4;
const int echoPin = 5;
float duration;
double distance;
double distanceThreshold = 30.0;           //distance threshold is 30 cm - subject to change

//Mircophone Sensor Parameters
// external sound sensor calibration
int SoundSensor = A0;
double SoundThreshold = 900;               // threshold value from sound threshold calibration
double SoundSensorvalNM,SoundSensorvalM,SoundSensorval;

 
void setup() {
  servo1.attach(9);                   // attaches the servo on pin 9 to the servo object
  servo2.attach(10);                  // attaches the servo on pin 10 to the servo object
  servo3.attach(11);                  // attaches the servo on pin 10 to the servo object
  pinMode(trigPin, OUTPUT);           //ultrasonic sensor trig pin
  pinMode(echoPin, INPUT);            //ultrasonic sensor echo pin
  pinMode(SoundSensor,INPUT);         //sound sensor analog - can also use digital pin
 
  //PID control
  PID_control.SetMode(AUTOMATIC);     
  PID_control.SetTunings(Kp, Ki, Kd);
  PID_control.SetOutputLimits(-255,255);

  // Serial Monitor
  Serial.begin(9600);                //change depending on baud rate of python radar code, 115200
 // Serial.println("Radar Start");

}

void loop() {
 
 // Serial.print("Time: ");
 // myTime = millis();
  
  distance = calculateDistance();
  Input = distance;
  SetPoint = distanceThreshold;
  PID_control.Compute();              //calculates the Ouput Value
  SoundSensorval = analogRead(SoundSensor);

  /*-----------------------------------------SERVO MOVE SNAKE FORWARD-------------------------------------------*/
 if( distance > distanceThreshold || Output != 0){       //distance > distanceThreshold
  for (pos = 0; pos <= 180; pos += 10) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    servo2.write(pos);              // tell servo to go to position in variable 'pos'
    //delay(10);
    //Serial.print("Distance from Target:");
    //Serial.println(distance);
    //delay(10);
  }
  for (pos = 180; pos >= 0; pos -= 10) { // goes from 180 degrees to 0 degrees
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    servo2.write(pos);              // tell servo to go to position in variable 'pos'
   // delay(10);
    //Serial.print("Distance from Target:");
    //Serial.println(distance);
    //delay(10);
    } 
 }

 /*------------------------------------DON'T LET SNAKE MOVE---------------------------------------------------------*/
 if( distance < distanceThreshold || Output == 0){
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    servo2.write(pos);              // tell servo to go to position in variable 'pos'
    //Serial.print("Distance from Target (Less):");
    //Serial.println(distance);
    //delay(10);
  }
   /*----------------------------------------SWEEP SERRVO FOR RADAR-------------------------------------------------*/ 
 if(distance <= distanceThreshold  && SoundSensorval >= SoundThreshold){ 

  // rotates the servo motor from 0 to 180 degrees
  for(int i=0;i<=180;i++){  
  servo3.write(i);
  delay(30);
  distance = calculateDistance();// Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree
  Serial.print(i);
  Serial.print(","); // comma separate variables
  Serial.println(distance); // print distance in cm
  
  }
  // Repeats the previous lines from 180 to 0 degrees
  for(int i=180;i>0;i--){  
  servo3.write(i);
  delay(30);
  distance = calculateDistance();
  Serial.print(i);
  Serial.print(","); // comma separate variables
  Serial.println(distance); // print distance in cm
    }
  }

    //Serial.print("Sound Sensor Value:");
   // Serial.print(SoundSensorval);
    //Serial.print("  ");
    //delay(10);
    //Serial.print("Distance from Target:");
    //Serial.println(distance);

    //distance = calculateDistance(); // Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree
    //Serial.print("Sound Sensor Value:");
    //Serial.print(SoundSensorval);
    //Serial.print("  ");
    //delay(10);
    //Serial.print("Distance from Target:");
    //Serial.println(distance);


    
    
  /*---------------------------------serial plot to match input and setpoint for PID tuning-------------------------- 

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
  */

 // Serial.println(myTime);
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
