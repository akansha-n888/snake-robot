int soundPin = A0;
int sensorValue = 0;
int dB,PdB;
 
void setup () 
{
Serial.begin (9600);
}
 
void loop () 
{
    PdB = dB;                                   //Store the previous of dB here
    sensorValue = analogRead (soundPin);
    dB = 20*log(sensorValue/10);        //Convert ADC value to dB using Regression values
    delay (10);

  if (PdB!=dB){
  Serial.println (dB);
  }
}
