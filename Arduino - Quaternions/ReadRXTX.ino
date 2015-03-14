/*
ch1 -392 (left) to 412
ch2 -406 (up) to 413
ch3 0 (off) to -813
ch4 -396 (left) to 393
ch5 962 (up) to 2068
ch6 970 to 2071
*/


volatile unsigned long CalibrateRxTracker, riseCh1, riseCh2, riseCh3, riseCh4, riseCh5, riseCh6; 

// Average Idle Channel Outputs

int zeroRx[6];

boolean calibrate = false;
boolean calibrated = false;


// INTERRUPT FUNCTIONS (must be ISRs that do not accept any parameters)

void startInterrupts() {
  attachInterrupt(ch[0], risingCh1Signal, RISING);
  attachInterrupt(ch[1], risingCh2Signal, RISING);
  attachInterrupt(ch[2], risingCh3Signal, RISING);
  attachInterrupt(ch[3], risingCh4Signal, RISING);
  attachInterrupt(ch[4], risingCh5Signal, RISING);
  attachInterrupt(ch[5], risingCh6Signal, RISING);
  CalibrateRxTracker = millis();
  calibrate = true;
}

void calibrateRxLoop() { 
  static unsigned int cloops;
  static long sumRx[4];
  
  if(calibrate && millis() < CalibrateRxTracker + 1000 && millis() > CalibrateRxTracker + 300) {    
    for(int x=0; x<4; x++) 
      sumRx[x] += volatileRx[x];
    cloops++;
  } 
  else if(calibrate && millis() > CalibrateRxTracker + 400) {
    for(int x=0; x<4; x++) 
      zeroRx[x] = sumRx[x] / cloops;
    calibrate = false;
    calibrated = true;
  }
}

void risingCh1Signal() {
  attachInterrupt(ch[0], fallingCh1Signal, FALLING);
  riseCh1 = micros();
}

void fallingCh1Signal() {
  attachInterrupt(ch[0], risingCh1Signal, RISING);
  volatileRx[0] = micros() - riseCh1;
}
  
void risingCh2Signal() {
  attachInterrupt(ch[1], fallingCh2Signal, FALLING);
  riseCh2 = micros();
}

void fallingCh2Signal() {
  attachInterrupt(ch[1], risingCh2Signal, RISING);
  volatileRx[1] = micros() - riseCh2;
}

void risingCh3Signal() {
  attachInterrupt(ch[2], fallingCh3Signal, FALLING);
  riseCh3 = micros();
}

void fallingCh3Signal() {
  attachInterrupt(ch[2], risingCh3Signal, RISING);
  volatileRx[2] = micros() - riseCh3;
}

void risingCh4Signal() {
  attachInterrupt(ch[3], fallingCh4Signal, FALLING);
  riseCh4 = micros();
}

void fallingCh4Signal() {
  attachInterrupt(ch[3], risingCh4Signal, RISING);
  volatileRx[3] = micros() - riseCh4;
}

void risingCh5Signal() {
  attachInterrupt(ch[4], fallingCh5Signal, FALLING);
  riseCh5 = micros();
}

void fallingCh5Signal() {
  attachInterrupt(ch[4], risingCh5Signal, RISING);
  volatileRx[4] = micros() - riseCh5;
}

void risingCh6Signal() {
  attachInterrupt(ch[5], fallingCh6Signal, FALLING);
  riseCh6 = micros();
}

void fallingCh6Signal() {
  attachInterrupt(ch[5], risingCh6Signal, RISING);
  volatileRx[5] = micros() - riseCh6;
}

void getRxValues() {
  if(calibrated) {
    for(int x=0; x<6; x++) 
      trueRx[x] = volatileRx[x] - zeroRx[x];
    controllerOn = true;
    if(volatileRx[4] > 1750) LeftToggle = true;
    else if(volatileRx[4] < 1250) LeftToggle = false;
    else controllerOn = false;
  }
  //Serial.print("Rx Values Read, Throttle = "); Serial.println(trueRx[2]);
}

// SETUP FUNCTIONS



void setupReceiverInterrupts() {
  pinMode(ch[0], INPUT);
  pinMode(ch[1], INPUT);
  pinMode(ch[2], INPUT);
  pinMode(ch[3], INPUT);
  pinMode(ch[4], INPUT);
  pinMode(ch[5], INPUT);
  attachInterrupt(ch[0], startInterrupts, RISING);
}





