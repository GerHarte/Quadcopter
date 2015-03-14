import processing.serial.*;
Serial myPort;  

float pitch = 45.0; 
float roll = 45.0; 
float yaw = 0.0; 

float a = 0.0;
float rSize;  // rectangle size

void setup() {  
  size(640, 360, P3D);
  rSize = width / 6;  
  noStroke();
  fill(204, 204);
  
  
  //println(Serial.list());
  //String portName = Serial.list()[0]; //com3, same as arduino
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.bufferUntil('\n');
}

void draw() {

  if ( myPort.available() > 0)  // If data is available,
  {  
    String inString = myPort.readStringUntil('\n');
    if (inString != null) {
        //trim off any whitespace:
        inString = trim(inString);
        float[] pitchRollYaw = float(split(inString, ','));
        if (pitchRollYaw.length == 3){
          pitch = pitchRollYaw[0];
          roll = pitchRollYaw[1];
          yaw = pitchRollYaw[2];
        }
     }    
     //println(inString);
  }

  
  //a += 1;
  //if(a > 360) { 
  //  a = 0.0; 
  //}
  
  background(255);
  lights();
  ortho(0, width, 0, height); 
  
  translate(width/2, height/2);
  
 
  //Pitch: Front Back
  rotateX(radians(pitch+90));
  
  //Roll: Left Right
  rotateY(radians(roll));
  
  //Yaw: North South East West
  rotateZ(radians(yaw));
  
  drawFrontBack();
  drawLeftRight();
  
  
}

void drawFrontBack()
{
  pushMatrix();
  fill(0, 0, 255, 255);
  stroke(4);
  box(20,200,21);
 
  translate(-100, 0, 0);
  fill(255, 0, 0, 255);
  sphere(15);
  
  translate(200, 0, 0);
  sphere(15);
  
  popMatrix();
}

void drawLeftRight()
{
  pushMatrix();
  fill(255, 0, 0, 255);
  stroke(4);
  box(200,20,20);
  
  translate(0, -100, 0);
  fill(0, 0, 255, 255);
  sphere(15);
  
  translate(0, 200, 0);
  sphere(15);
  
  
  popMatrix();
}

