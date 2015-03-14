import cc.arduino.*;
import processing.serial.*;
import oscP5.*;
//import netP5.*;


Arduino arduino;
Serial myPort;
OscP5 oscP5;

float p_Val = 0.0;
float i_Val = 0.0;
float d_Val = 0.0;
float throttle_Val = 0.0f;
int i = 0;
int j = 0;
int i_last = 0;
String last_delim_vals;
String a;
String delim_vals;

void setup(){
  size(320, 480);
  background(0);
  oscP5 = new OscP5(this, 8000); 
  
  String portName = Serial.list()[5];
  myPort = new Serial(this, portName, 115200);
  
  print(Arduino.list());
  
  arduino = new Arduino(this, Arduino.list()[0],57600);
  
  
}


void draw(){
  background((p_Val/3)*255, (i_Val/3)*255, (d_Val/3)*255);
  
  fill(0);
  //red rect
  stroke(255,0,0);
  rect(34,39,67,255);
  fill(50,40,40);
  rect(34,39+255,67,-(p_Val/3)*255);
  
  //green rect
  fill(0);
  stroke(0,255,0);
  rect(124,39,67,255);
  fill(40,50,40);
  rect(124,39+255,67,-(i_Val/3)*255);
  
  //blue rect
  fill(0);
  stroke(0,0,255);
  rect(216,39,67,255);
  fill(40,40,50);
  rect(216,39+255,67,-(d_Val/3)*255);
  
  //write to arduino
  //arduino.analogWrite(11, int(redAmount));
  //arduino.analogWrite(10, int(greenAmount));
  //arduino.analogWrite(9, int(blueAmount));
  
  
  //json = "{\"throttle\":" + str(throttle_Val) + ",  \"p\":" + str(p_Val) + ",  \"i\":" + str(i_Val) + ",  \"d\":" + str(d_Val) + "}\n";
  delim_vals = str(throttle_Val) + "," + str(p_Val) + "," + str(i_Val) + "," + str(d_Val) + "\n";


  
  if(delim_vals.equals(last_delim_vals)){
    
    if (i_last == i){
      i_last = i;
    }else{  
      i = 0;
      myPort.write(delim_vals);
      //print(delim_vals);
      
    }
  } else {
    i = i + 1;
    last_delim_vals = delim_vals;
  }
  
  if (myPort.available() > 0){
    a = myPort.readString();
    println(a);
  }
  
}

void oscEvent(OscMessage theOscMessage){
  String addr = theOscMessage.addrPattern();
  float val = theOscMessage.get(0).floatValue();
  
  if(addr.equals("/1/throttlefader")){ throttle_Val = val;}
  if(addr.equals("/1/pfader")){ p_Val = val;}
  if(addr.equals("/1/ifader")){ i_Val = val;}
  if(addr.equals("/1/dfader")){ d_Val = val;}
  
  
  
}


