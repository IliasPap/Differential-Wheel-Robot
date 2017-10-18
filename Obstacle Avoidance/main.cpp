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
    wait(1.0); 
    MeanAyGz();
    wait(0.2);
    
    calc.start();
    ti.start();
    while(1) {
        ReadData();
        CalcAccGyro(calc.read());
        PathObstacles(calc.read());
       
        calc.reset();
       
           
        
    }

}
