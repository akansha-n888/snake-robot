%Capstone Ultrasonic Sensor Arduino Interface
% create an arduino object
arduinoObj = arduino('COM4','Uno',"Libraries","Ultrasonic") ;
%trigger pin set to D2 and the echo pin set to D4
sensor = ultrasonic(arduinoObj,'D2','D4');
%Measure sensed Distance in meters
val = readDistance(sensor);
sprintf('Current distance is %.4f meters\n', val)
% measure time of travel
t = readEchoTime(sensor);
disp(t)
clear sensor
clear arduinoObj