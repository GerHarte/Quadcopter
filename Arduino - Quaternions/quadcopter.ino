// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>
#include <Servo.h> 
#include <PID_v1.h> 
#include <PID_AutoTune_v0.h>

// I2Cdev and L3G4200D must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "/Users/gerardharte/Documents/Arduino/quadcopter/I2Cdev.h"
#include "/Users/gerardharte/Documents/Arduino/quadcopter/L3G.h"
#include "/Users/gerardharte/Documents/Arduino/quadcopter/HMC5883L.h"
#include "/Users/gerardharte/Documents/Arduino/quadcopter/ADXL345.h"
#include "/Users/gerardharte/Documents/Arduino/quadcopter/BMP085.h"
#include "/Users/gerardharte/Documents/Arduino/quadcopter/ahrs.h"
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

double SetpointYaw, InputYaw, OutputYaw;

//PID Autotune
PID_ATune aTune(&InputRoll, &OutputRoll);
double aTuneStep=40, aTuneNoise=1, aTuneStartValue=0, aTuneControlType = 1;
unsigned int aTuneLookBack= 5; //20;
  
boolean tuning = false;
unsigned long  modelTime;

// default address is 105
// specific I2C address may be passed here
BMP085 bmp;
HMC5883L hmc;
ADXL345 adxl(false);
L3G gyro;

unsigned long delta_t = 0;


//Set up gyroscope variables
float gyroBiasX, gyroBiasY, gyroBiasZ;


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


template <class T> class BrownLinearExpo
{
	public:
		BrownLinearExpo() { a = 0.5; }
		BrownLinearExpo(float f, volatile T init_est) { a = f; estimate = init_est; }

		void set_factor(float factor) { a = factor; }
		void step(volatile T measurement)
		{
			single_smoothed = a * measurement + (1 - a) * single_smoothed;
			double_smoothed = a * single_smoothed + (1 - a) * double_smoothed;

			T est_a = (2*single_smoothed - double_smoothed);
			T est_b = (a / (1-a) )*(single_smoothed - double_smoothed);
			estimate = est_a + est_b;
		}
		volatile T get(){ return estimate; }

	private:
		T estimate, double_smoothed, single_smoothed;
		float a;
};


BrownLinearExpo<float> alt_filter(0.1, 0);


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  I2C_BUS.begin();

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
  //mag_cal();
  Serial.println("Sensors Calibrated");
  
  
  Serial.println("Setting up Transmitter...");
  setupReceiverInterrupts();
  Serial.println("Transmitter Set up");
  
  
 if(tuning)
  {
    Serial.println("Tuning");
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
 
  if((millis() - delta_t) < 20)
        return;


  float dt = millis() - delta_t;
  delta_t = millis();
  dt /= 1000.0;

  
   
  
  gyro.read();
  

  /*
  imu::Vector<3> ang_vel;
  ang_vel.x() =  (((int)gyro.g.x - gyroBiasX)*.07);
  ang_vel.y() =  (-((int)gyro.g.y - gyroBiasY)*.07);
  ang_vel.z() =  (((int)gyro.g.z - gyroBiasZ)*.07);
  */    
  
  uimu_ahrs_iterate(gyro.g, adxl.read_acc(), hmc.read_mag());


  imu::Vector<3> euler = uimu_ahrs_get_euler();

   
  //Altitude
  alt_filter.step(bmp.read_altitude()*0.032808399);

  
  /*
    //Temp
    Serial.println(bmp.read_temp()/10.0);
  */
  
  InputPitch = euler.y();
  InputRoll = euler.z();
  InputAlt = alt_filter.get();
  InputYaw = euler.x();
  
 
  Serial.print("InputPitch: ");
  Serial.print(InputPitch);
  Serial.print("\t InputRoll: ");
  Serial.print(InputRoll);
  Serial.print("\t InputYaw: ");
  Serial.println(InputYaw);

  

/*  
  Serial.print("InputPitch: ");
  Serial.print(InputPitch);
  Serial.print("\t InputRoll: ");
  Serial.print(InputRoll);
  Serial.print("\t InputAlt: ");
  Serial.print(InputAlt);
  Serial.print("\t InputYaw: ");
  Serial.println(InputYaw);
*/
  
  //unsigned long now = millis();
  
  
  MicrosTracker = micros(); 
  calibrateRxLoop();
  getRxValues();
  
  
  if(tuning)
  {
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
  else {
    
  //SetpointPitch = map(trueRx[1], ch_from[1]  , ch_to[1], -20, 20);
  SetpointPitch = 0;
  SetpointRoll = map(trueRx[0], ch_from[0]  , ch_to[0], -20, 20);
  SetpointAlt = 0;
  
  //PitchPID.Compute();
  RollPID.Compute();
  //AltPID.Compute();
  
  }
  
  //curr_pid_val[0] =  mapfloat(trueRx[5] , ch_from[5], ch_to[5], 0.7 , 1.5);
  //RollPID.SetTunings(curr_pid_val[0],curr_pid_val[1],curr_pid_val[2]);
  
  throttle = map(trueRx[2] , ch_from[2]  , ch_to[2], 1000.00, 1800.00);
  

    

  
  setSpeed(throttle, OutputPitch, OutputRoll); 

  
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
    
    

  } 

   
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
  
  

