void calibSensors(){


  hmc.init();
  delay(10);
  adxl.init();
  delay(10);
  gyro.init();
  delay(10);
// initialize sensors
  gyro.enableDefault();

  /*
  // Calculate bias for the Gyro i.e. the values it gives when it's not moving
  for(int i=1; i < 100; i++){
    gyro.read();
    gyroBiasX += (int)gyro.g.x;
    gyroBiasY += (int)gyro.g.y;
    gyroBiasZ += (int)gyro.g.z;  

    delay(1);
  }
  

  gyroBiasX = gyroBiasX / 100;
  gyroBiasY = gyroBiasY / 100;
  gyroBiasZ = gyroBiasZ / 100;
  
  */
  imu::Vector<3> acc = adxl.read_acc();
  imu::Vector<3> mag = hmc.read_mag();

  uimu_ahrs_init(acc, mag);
  uimu_ahrs_set_beta(0.2);

  bmp.init();
  bmp.zeroCal(0, 0);
  

  
}


void mag_cal()
{
	for(int i = 0; i < 6; i++){
		hmc.cal[i] = 0;
          }
        Serial.println("Calibrating Mag");
	for(int i = 0; i < 3000; i++)
	{
                Serial.println(i);
		imu::Vector<3> mag = hmc.read_raw();

		if(mag[0] < hmc.cal[0])
		    hmc.cal[0] = mag[0];
		if(mag[0] > hmc.cal[1])
		    hmc.cal[1] = mag[0];

		if(mag[1] < hmc.cal[2])
		    hmc.cal[2] = mag[1];
		if(mag[1] > hmc.cal[3])
		    hmc.cal[3] = mag[1];

		if(mag[2] < hmc.cal[4])
		    hmc.cal[4] = mag[2];
		if(mag[2] > hmc.cal[5])
		    hmc.cal[5] = mag[2];

		delay(20);
	}

	Serial.println("mag_cal:");
	for(int i = 0; i < 6; i++)
	{
                Serial.print(i);
		Serial.print("\t");
		Serial.println(int(hmc.cal[i]));
	}
	Serial.print("\n");
	delay(10000);
}
