#include <Servo.h> 

Servo myservo;  // create servo object to control a servo 

int val = 0;    // variable to read the value from the analog pin 


void setSpeed(int val){
  // From some testing I found my ESC stopped at a value of 59 and
  // maxed out at a value of 133. The next line takes an input from 
  // 0-100 and maps it to 59-133.
  int speed = map(val, 0, 100, 59, 133);
  // Send the speed to the motor
  myservo.write(speed); 
  // Print out what's happening
  Serial.print("Setting Speed to ");
  Serial.println(speed, DEC);
}


void setup() 
{ 
  // Open a serial port to allow values to be sent
  Serial.begin(9600);
  // Attach the servo on pin 9 to the servo object
  myservo.attach(9);   
} 

void loop() 
{
  // We only want to change the speed when we send a new value,
  // otherwise don't change anything
  if (Serial.available()) {
    // Read the value from the serial port as an integer
    val = Serial.parseInt();
    // Call the setSpeed function from above
    setSpeed(val); 
  }
}


