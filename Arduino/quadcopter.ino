// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>
#include <Servo.h> 
#include <PID_v1.h> 
#include <PID_AutoTune_v0.h>

// I2Cdev and L3G4200D must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <L3G.h>
#include <HMC5883L.h>
#include <ADXL345.h>
#include <Kalman.h>

// Set up the Ultrasonic Distance Sensor
#define echoPin 6 // Echo Pin
#define trigPin 7 // Trigger Pin
long duration, distance; // Duration used to calculate distance

// Set up Pitch PID
double SetpointPitch, InputPitch, OutputPitch;
double KpPitch =  0.787;//0.095;//0.115;{0.565,0, 0.372};{0.787, 0.054, 2.885};
double KiPitch = 0.054;//0.025;
double KdPitch = 2.885;//0.8;//0.0013;//0.00285;
PID PitchPID(&InputPitch, &OutputPitch, &SetpointPitch, KpPitch, KiPitch, KdPitch, DIRECT);

//Set up Roll PID
double SetpointRoll, InputRoll, OutputRoll;
double KpRoll = 0.411;
double KiRoll = 0.08;
double KdRoll = 0.528;
PID RollPID(&InputRoll, &OutputRoll, &SetpointRoll, KpRoll, KiRoll, KdRoll, DIRECT);

//Set up Altitude PID
double SetpointAlt, InputAlt, OutputAlt;
double KpAlt = 0;
double KiAlt = 0;
double KdAlt = 0;
double pidTime = 0;
PID AltPID(&InputAlt, &OutputAlt, &SetpointAlt, KpAlt, KiAlt, KdAlt, DIRECT);


//PID Autotune
PID_ATune aTune(&InputRoll, &OutputRoll);
double aTuneStep=40, aTuneNoise=1, aTuneStartValue=0, aTuneControlType = 1;
unsigned int aTuneLookBack= 5; //20;
  
boolean tuning = true;
unsigned long  modelTime;

// default address is 105
// specific I2C address may be passed here
L3G gyro;
ADXL345 accel;
HMC5883L mag;

//Set up kalman instances
Kalman kalmanPitch; 
Kalman kalmanRoll;
double pitchKal, rollKal; 

//Set up Magnometer variables
int16_t magX, magY, magZ;

//Set up accelerometer variables
int16_t accValX, accValY, accValZ;
float accBiasX, accBiasY, accBiasZ;
float accAngleX, accAngleY;
double accPitch, accRoll;

//Set up gyroscope variables
int16_t gyroX, gyroY, gyroZ;
float gyroBiasX, gyroBiasY, gyroBiasZ;
float gyroRateX, gyroRateY, gyroRateZ;
float gyroBias_oldX, gyroBias_oldY, gyroBias_oldZ;
float gyroPitch = 180;
float gyroRoll = -180;
float gyroYaw = 0;
double gyro_sensitivity = 70; //From datasheet, depends on Scale, 2000DPS = 70, 500DPS = 17.5, 250DPS = 8.75. 

float pitchComp=0;
float rollComp = 0;
float yawComp = 0;

//Set up motors
Servo motor1;  // Front (Anti-clockwise)
Servo motor2;  // Back (Clockwise)
Servo motor3;  // Left (Anti-clockwise)
Servo motor4;  // Right (Clockwise)
float throttle = 0;


//Set up a timer Variable
uint32_t timer;
uint32_t timer2;
int count = 1;


//RX and TX Variables

const int ch[6] {2,3,4,5,6,7};
boolean LeftToggle, controllerOn;


volatile int volatileRx[6];
unsigned long MicrosPassed;         // Amount of time taken to run the last loop
unsigned long MicrosTracker;

int trueRx[6];
const float ch_from[6] {-393, -418,    0, -396,  962,  970};
const float ch_to[6]   {412,   402, -813,  393, 2068, 2170};

int tmp_ch5 = 0;
int state_switch = 0;
int curr_pid = 0;

float curr_pid_val[3] = {0.787, 0.054, 2.885};

char curr_pid_tuning;



void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(9600);
  Serial.setTimeout(50);
  
  Serial.println("Initializing...");
  
  Serial.println("Attaching Motors...");
  motor1.attach(11); // Front
  delay(200);
  motor2.attach(10); // Back
  delay(200);
  motor3.attach(9); // Left
  delay(200);
  motor4.attach(8); // Right
  delay(200);
  
  setSpeed(1000, 0, 0);
  Serial.println("Motors Attached");

  
  Serial.println("Calibrating Sensors...");
  calibSensors();
  Serial.println("Sensors Calibrated");
  
  
  Serial.println("Setting up Transmitter...");
  setupReceiverInterrupts();
  Serial.println("Transmitter Set up");
  
  
  /*
  Serial.println("Calibrating Transmitter...");
  calibRxTx();
  Serial.println("Transmitter Calibrated");
  */
  
  
 if(tuning)
  {
    Serial.println("A");
    tuning=false;
    changeAutoTune();
    tuning=true;
  }

  // Start the PIDs
  PitchPID.SetMode(AUTOMATIC);
  RollPID.SetMode(AUTOMATIC);
  AltPID.SetMode(AUTOMATIC);
  PitchPID.SetOutputLimits(-1000, 1000);
  RollPID.SetOutputLimits(-1000, 1000);
  AltPID.SetOutputLimits(-1000, 1000);
  PitchPID.SetSampleTime(1);
  RollPID.SetSampleTime(1);

  timer = micros();
  throttle = 1000;
  
  MicrosTracker = micros(); 
  calibrateRxLoop();
  getRxValues();
  
  Serial.println("Setup done.");
}


void loop() {
  
  
  //////////////////////
  //  Accelerometer   //
  //////////////////////
  

    accel.getAcceleration(&accValX, &accValY, &accValZ);
  
    accPitch = (atan2(-accValX,-accValZ)+PI)*RAD_TO_DEG;
    accRoll = (atan2(accValY,-accValZ)+PI)*RAD_TO_DEG;
  
    if (accPitch <= 360 & accPitch >= 180){
      accPitch = accPitch - 360;
    }
  
    if (accRoll <= 360 & accRoll >= 180){
      accRoll = accRoll - 360;
    }
    



  //////////////////////
  //      GYRO        //
  //////////////////////


  gyro.read();

  // read raw angular velocity measurements from device  
  gyroRateX = ((int)gyro.g.x - gyroBiasX)*.07; //*(.0105);
  gyroRateY = -((int)gyro.g.y - gyroBiasY)*.07; //*(.0105);
  gyroRateZ = ((int)gyro.g.z - gyroBiasZ)*.07; //*(.0105);


  gyroPitch += gyroRateY * ((double)(micros() - timer)/1000000);
  gyroRoll += gyroRateX * ((double)(micros() - timer)/1000000);
  gyroYaw += gyroRateZ * ((double)(micros() - timer)/1000000);


  //////////////////////
  //    MAGNOMETER    //
  //////////////////////
/*
  mag.getHeading(&magX, &magY, &magZ);

  //Heading Estimation
  float magYaw = atan2(magY, magX);
  if(magYaw < 0) 
    magYaw += 2 * M_PI;
  magYaw = (magYaw*180)/M_PI;
*/

  //Complementary Filter
  //InputPitch = .8 *(pitchComp + (gyroRateY * ((double)(micros() - timer)/1000000))) + .2 *accPitch;
  //InputRoll = .8 *(rollComp + (gyroRateX * (double)(micros() - timer)/1000000)) + .2 * accRoll;
  //yawComp = .8*yawComp + .2*gyroYaw;
  

  InputPitch = kalmanPitch.getAngle(accPitch, gyroRateY, (double)(micros()-timer)/1000000);
  InputRoll = kalmanRoll.getAngle(accRoll, gyroRateX, (double)(micros()-timer)/1000000);
  timer = micros();

  
  //InputAlt = getAlt();

  unsigned long now = millis();
  
  MicrosTracker = micros(); 
  calibrateRxLoop();
  getRxValues();
  
  if(tuning)
  {
    //Serial.println("B");
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      curr_pid_val[0] = aTune.GetKp();
      curr_pid_val[1] = aTune.GetKi();
      curr_pid_val[2] = aTune.GetKd();
      RollPID.SetTunings(curr_pid_val[0],curr_pid_val[1],curr_pid_val[2]);
      
      Serial.println("Autotuning Complete");
      Serial.println("Values:");
      Serial.print("P: ");
      Serial.print(RollPID.GetKp(),3);
      Serial.print("\t I: ");
      Serial.print(RollPID.GetKi(),3);
      Serial.print("\t D: ");
      Serial.println(RollPID.GetKd(),3);
    }
  }
  else RollPID.Compute();
  
  //PitchPID.Compute();
  //RollPID.Compute();
  //AltPID.Compute();
  
  //curr_pid_val[0] =  mapfloat(trueRx[5] , ch_from[5], ch_to[5], 0.7 , 1.5);
  //RollPID.SetTunings(curr_pid_val[0],curr_pid_val[1],curr_pid_val[2]);
  
  throttle = map(trueRx[2] , ch_from[2]  , ch_to[2], 1000.00, 1800.00);
  
  //Compute PID
  //map(constrain(trueRx[0], -400, 400) , -400  , 400, -20, 20);;
  
  //SetpointPitch = map(trueRx[1], ch_from[1]  , ch_to[1], -20, 20);
  SetpointPitch = 0;
  SetpointRoll = map(trueRx[0], ch_from[0]  , ch_to[0], -20, 20);
  SetpointAlt = 0;
  
    

  

  
  //setSpeed(throttle, OutputPitch, OutputRoll); 
  //setSpeed(OutputAlt, OutputPitch, OutputRoll); 
  
  if (abs(tmp_ch5 - trueRx[4]) > 100){ //Check if channel 5 has been switched  
    tmp_ch5 = trueRx[4];
    
    
    Serial.print("InputRoll ");
    Serial.print(InputRoll);
    Serial.print(", OutputRoll ");
    Serial.print(OutputRoll);
    Serial.print(", trueRx[0] ");
    Serial.print(trueRx[0]);
    Serial.print(", From ");
    Serial.print(ch_from[0]);
    Serial.print(", To ");
    Serial.print(ch_to[0]);
    Serial.print(", P Value: "); 
    Serial.print(RollPID.GetKp(),3); 
    Serial.print(", I Value: "); 
    Serial.print(RollPID.GetKi(),3); 
    Serial.print(", D Value: "); 
    Serial.println(RollPID.GetKd(),3); 
    
    

    /*
    if (state_switch % 3 == 0){  curr_pid_tuning = 'P'; curr_pid = 0; } 
    else if (state_switch % 3 == 1){ curr_pid_tuning = 'I'; curr_pid = 1; } 
    else if (state_switch % 3 == 2){ curr_pid_tuning = 'D'; curr_pid = 2; }
    
  
    state_switch++;
    */
  } 
  //curr_pid = 0;
  
  //curr_pid_val[curr_pid] =  mapfloat(trueRx[5] , ch_from[5], ch_to[5], 0.000 , 2.000);
  
  
  //PitchPID.SetTunings(curr_pid_val[0], curr_pid_val[1], curr_pid_val[2]);


    Serial.print("InputPitch ");
    Serial.print(InputPitch);
    Serial.print("\t \t InputRoll ");
    Serial.println(InputRoll);
 

   
}



void changeAutoTune()
  {
   if(!tuning)
    {
      //Set the output to the desired starting frequency.
      OutputPitch=aTuneStartValue;
      aTune.SetNoiseBand(aTuneNoise);
      aTune.SetOutputStep(aTuneStep);
      aTune.SetLookbackSec((int)aTuneLookBack);
      aTune.SetControlType(aTuneControlType);
      tuning = true;
    }
    else
    { //cancel autotune
      aTune.Cancel();
      tuning = false;
    }
  }
  
  

