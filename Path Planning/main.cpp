////******************************************/
 
#include "mbed.h"
#include "MPU9150.h"

#include "sensors.h"
#define SE 1000000

#define DRIFT 2


// ********************************************************************************************************************        

   

int main(){
    myled=1;
    initializeRob();
    while(user_but){
        myled=!myled;
        wait(0.5);
    }
    wait(2); 
    MeanAyGz();
    wait(0.2);
    ena.period(0.05);
    enb.period(0.05);
    cur_error=0.0;
    prev_error=0.0;
    total_error=0.0;
    meangz=0.0;
    meanay=0.0;
    dist=0.0;
    angle=0.0;
    speed_prev=0.0;
    gz_prev=0.0;
    ay_prev=0.0;
    calc.start();
    ti.start();
    while(1) {
        ReadData();
                //Path(gz,calc.read());
        CalcAccGyro(calc.read());
        calc.reset();
        Controller();
         // if(dist
           wait_ms(10);
          if(ti.read()>1){
              pc.printf(" Dist=%f Angle=%f Speed=%f \r\n",cur_error,angle,total_error);
              
               ti.reset();
            }
          
       
           
                
    
        
    }

}
