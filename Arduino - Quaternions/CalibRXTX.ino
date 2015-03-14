/*

void calibRxTx(){

    MicrosTracker = micros(); 
    calibrateRxLoop();
    getRxValues();
   
    tmp_ch5 = trueRx[4];
   
   Serial.print("Values Read - "); Serial.println(tmp_ch5);
   
   //Calibrating Transmitter
   Serial.println("Cycle through full range of channel values to calibrate.");
   while( abs(tmp_ch5 - trueRx[4]) < 100 ){  
      for(int i = 0; i < 6; i++){
       if (trueRx[i] > ch_to[i]){  ch_to[i] = trueRx[i]; } 
       else { ch_from[i] = trueRx[i]; }
          MicrosTracker = micros(); 
          calibrateRxLoop();
          getRxValues();  
      }     
   }
   
   Serial.print("Channel 1: To= "); Serial.print(ch_to[0]); Serial.print(" From="); Serial.println(ch_from[0]);
   Serial.println("");Serial.print("Channel 2: To=");Serial.print(ch_to[1]);Serial.print(" From=");Serial.println(ch_from[1]);
   Serial.println("");Serial.print("Channel 3: To=");Serial.print(ch_to[2]);Serial.print(" From=");Serial.println(ch_from[2]);
   Serial.println("");Serial.print("Channel 4: To=");Serial.print(ch_to[3]);Serial.print(" From=");Serial.println(ch_from[3]);
   Serial.println("");Serial.print("Channel 5: To=");Serial.print(ch_to[4]);Serial.print(" From=");Serial.println(ch_from[4]);
   Serial.println("");Serial.print("Channel 6: To=");Serial.print(ch_to[5]);Serial.print(" From=");Serial.println(ch_from[5]);Serial.println("");
   
   if(abs(ch_to[2] - trueRx[2]) > 100){
     Serial.println("Set Throttle to Zero");
     while( abs(ch_to[2] - trueRx[2]) > 100){
       trueRx[2] = trueRx[2];
     }
   }
   
   Serial.println("Armed");
   
   delay(1000);
   
   tmp_ch5 = trueRx[4];
}
*/
