//Function to get altitude from Ultrasonic Sensor
double getAlt(){

  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2); 

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 

  digitalWrite(trigPin, LOW);
  return duration = pulseIn(echoPin, HIGH);
}


