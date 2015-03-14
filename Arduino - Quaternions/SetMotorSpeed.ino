//Function to set the speed of the motors
void setSpeed(int OutputAlt, double OutputPitch, double OutputRoll){

  //OutputPitch_tmp = 0;
  //OutputRoll_tmp = 0;
  //OutputAlt = 0;

  //Map 0-100 to 55-133, 55 is no power, 133 is full power
 // int OutputAlt_tmp = map(OutputAlt, 0, 100, 1000, 2);
  motor1.writeMicroseconds(OutputAlt - OutputPitch/2 + OutputRoll/2); // Front
  motor2.writeMicroseconds(OutputAlt + OutputPitch/2 - OutputRoll/2); // Back
  motor3.writeMicroseconds(OutputAlt - OutputPitch/2 - OutputRoll/2); // Left
  motor4.writeMicroseconds(OutputAlt + OutputPitch/2 + OutputRoll/2); // Right

  //motor1.writeMicroseconds(1000); // Front
  //motor2.writeMicroseconds(1000); // Back
  //motor3.writeMicroseconds(1000); // Left
  //motor4.writeMicroseconds(1000); // Right
}

