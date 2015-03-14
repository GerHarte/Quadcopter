
void calibSensors(){

// initialize sensors
  gyro.init();
  accel.initialize();
  mag.initialize();

  gyro.enableDefault();


  // Calculate bias for the Gyro i.e. the values it gives when it's not moving
  for(int i=1; i < 100; i++){
    gyro.read();
    gyroBiasX += (int)gyro.g.x;
    gyroBiasY += (int)gyro.g.y;
    gyroBiasZ += (int)gyro.g.z;  

    accBiasX += accel.getAccelerationX();
    accBiasY += accel.getAccelerationY();
    accBiasZ += accel.getAccelerationZ();

    delay(1);
  }
  

  gyroBiasX = gyroBiasX / 100;
  gyroBiasY = gyroBiasY / 100;
  gyroBiasZ = gyroBiasZ / 100;

  accBiasX = accBiasX / 100;
  accBiasY = accBiasY / 100;
  accBiasZ = accBiasZ / 100;
  

  //Get Starting Pitch and Roll
  accel.getAcceleration(&accValX, &accValY, &accValZ);
  accPitch = (atan2(-accValX,-accValZ)+PI)*RAD_TO_DEG;
  accRoll = (atan2(accValY,-accValZ)+PI)*RAD_TO_DEG;

  if (accPitch <= 360 & accPitch >= 180){
    accPitch = accPitch - 360;
  }

  if (accRoll <= 360 & accRoll >= 180){
    accRoll = accRoll - 360;
  }
  

  // Set starting angle for Kalman
  kalmanPitch.setAngle(accPitch); 
  kalmanRoll.setAngle(accRoll);
  gyroPitch = accPitch;
  gyroRoll = accRoll;
  //pitchComp = accPitch;
  //rollComp = accRoll;
  
  
    // Set up Ultrasonic Sensor
  //pinMode(trigPin, OUTPUT);
  //pinMode(echoPin, INPUT);
  //double startAlt = getAlt();
  
}
