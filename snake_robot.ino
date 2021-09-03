/* autonomous snake robot with basic sensing capabilities*/
#include <Servo.h>
#include <PID_v1.h>

// Servo Motor Parameters
Servo servo1;                         // servo1 - move two phase memeber
Servo servo2;                         // servo2 - move two phase member
Servo servo3;                         // servo3 - orient snake upward
int pos;                              // variable to store the servo position (1 & 2)
int i;                                // variable to store servo 3 position

//PID Parameters
double SetPoint, Input, Output;
double Kp = 0.89, Ki = 0.005, Kd = 0.007;    //subject to change bc of tuning

PID PID_control(&Input, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);

// Response Time Parameters
unsigned long myTime;


//Ultrasonic Sensor Parameters
// external ultrasonic sensor calibration
const int trigPin = 3;            //digital pins
const int echoPin = 2;            // digital pins
float duration;
double distance;
double distanceThreshold = 12.0;           //distance threshold is 30 cm - subject to change

//Mircophone Sensor Parameters
// external sound sensor calibration
int SoundSensor = A0;                       //convert AC eletrical signals - analog
double SoundThreshold = 800;               // threshold value from sound threshold calibration
double SoundSensorval;
 
void setup() {
  servo1.attach(9);                   // attaches the servo on pin 9 to the servo object
  servo2.attach(10);                  // attaches the servo on pin 10 to the servo object
  servo3.attach(11);                  // attaches the servo on pin 11 to the servo object
  pinMode(trigPin, OUTPUT);           //ultrasonic sensor trig pin
  pinMode(echoPin, INPUT);            //ultrasonic sensor echo pin
  pinMode(SoundSensor,INPUT);         //sound sensor analog - can also use digital pin
 
  //PID control
  PID_control.SetMode(AUTOMATIC);     
  PID_control.SetTunings(Kp, Ki, Kd);

  // Serial Monitor
  Serial.begin(9600);                //change depending on baud rate of python radar code, 115200

}

void loop() {
  distance = calculateDistance();
  Input = distance;
  SetPoint = distanceThreshold;
  PID_control.Compute();              //calculates the Output Value
  SoundSensorval = analogRead(SoundSensor);

  Serial.print("Distance:");      
  Serial.print(distance);
  Serial.print(" , ");  
  Serial.print("Sound Sensor Value:");
  Serial.println(SoundSensorval);

    /*---------------------------------serial plot to match input and setpoint for PID tuning-------------------------- */
/*
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
  delay(10); */
  
    
  /*-----------------------------------------SERVO MOVE SNAKE FORWARD-------------------------------------------*/
 if( Input > SetPoint ){       //distance > distanceThreshold 
  for (pos = 0; pos <= 180; pos += 10) { // goes from 0 degrees to 180 degrees
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    servo2.write(pos);              // tell servo to go to position in variable 'pos'                                                                               
  }
 }

 /*------------------------------------DON'T LET SNAKE MOVE---------------------------------------------------------*/
 if(Input < SetPoint){      //distance < distanceThreshold ||
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    servo2.write(pos);              // tell servo to go to position in variable 'pos'                                        
  }
   /*----------------------------------------SWEEP SERRVO FOR RADAR-------------------------------------------------*/ 
 if(distance <= distanceThreshold  && SoundSensorval >= SoundThreshold){ 
  
  for(i = 0; i <= 180; i++){      // rotates the servo motor from 0 to 180 degrees, use i to distguish from servo1 and servo2 at base
    servo3.write(i);
    delay(100);  
    }
  }
}

/*------------------------------------------ distance function -----------------------------------------------------*/
float calculateDistance() {       
  digitalWrite(trigPin, LOW);         //clear trig pin
  delayMicroseconds(2);               //in low state for 2 micro seconds
  digitalWrite(trigPin, HIGH);        
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034/2;        //340 m/s speed of sound
  return distance;
}
