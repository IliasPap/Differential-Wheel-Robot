
#include "mbed.h"
#include "MPU9150.h"


#define TH    60
#define VERY_CLOSE_OBST    60
#define NEAR_OBST   60
#define PWM_R 0.5
#define PWM_L 0.5
#define Des_GYRO 0.0
#define CircleROT -6.5
#define SCALE 0.405
#define DEG_CORNER 10


void UltraTrigStart();
void UltraEchoStart();
void UltraEchoEnd();
void CheckObstacle();
void frwd(float r,float l);
void back();
void left();
void right();
void leftfast();
void rightfast();
void stop();
void still();
void UpdateLCD();
void initializeRob();
void Controller();
void Path(float gyroz,float dt); 
float OpenEyewait(float time);
void MeanAyGz();
void ReadData();

DigitalIn       user_but(PC_13);
DigitalOut      ultraTrig(PC_6) ;
InterruptIn     ultraEcho(PC_8) ;
Timer           EchoTime ;
Ticker          UltaTickerTrig ;
float           UltraDistance ;
float           UltraEchoTime ;

//Left wheel
DigitalOut in1(PC_10);
DigitalOut in2(PC_11);
PwmOut ena(PC_2);
//Right wheel
DigitalOut in3(PA_13);
DigitalOut in4(PA_14);
PwmOut enb(PC_3);

DigitalOut led(LED1);
double volatile meangz,meanay;
unsigned int counter;
double volatile angle,gz_now,ay_prev,ay_now,gz_prev,speed_prev,speed,dist;
double p,cur_error,prev_error,total_error;
float pyaw=0;
double volatile Right=PWM_R;
double volatile Left=PWM_L;
float kp=0.00005,kd=0.00001;

typedef enum State {K1,K2,K3,K4,K5,K6,K7,K8 };

State rob_state=K1;

MPU9150 MPU9150;
Timer t,stateTimer,calc,ti;
Serial pc(USBTX, USBRX); // tx, rx
    
    
    
    
void CalcAccGyro(double det){
    angle+=((gz_prev+gz_now)/2.0)*det;
    speed+=(9.81*(ay_prev+ay_now)/2.0)*det;
    dist+=((speed_prev+speed)/2.0)*det;
    /*
    if(fabs(dist)>1.0){
        stop();
        pc.printf("dist >1");
        while(1);
        }
        */
    speed_prev=speed;
    ay_prev=ay_now;
    gz_prev=gz_now;
    
 
    }    
    
void Controller(){
    
    cur_error=0.0-gz;
    //total_error+=cur_error;
   
   
    Right=Right-kp*cur_error;//+kd*(cur_error-prev_error);
    Left=Left+kp*cur_error ;//-kd*(cur_error-prev_error);
     prev_error=cur_error;
    frwd(Right,Left);
    
    }


void UltraTrigStart()
{
//    led=1 ;
    ultraTrig = 1 ;
    wait_us(12) ;
    ultraTrig = 0 ;
//    led=0 ;
}

//**************************************************
void UltraEchoStart()
{
    EchoTime.reset() ;
    EchoTime.start() ;
    //led=1 ;
}


//**************************************************
void UltraEchoEnd()
{
    EchoTime.stop() ;
    UltraEchoTime = EchoTime.read();
    UltraDistance = UltraEchoTime * (340.0*50) ;   // estimation in centimeters
    //led=0 ;
}

void frwd(float r,float l){
    in1=1;
    in2=0;
    in3=1;
    in4=0;
   enb.write(r);
   ena.write(l);
  
    }
    




void left(){
    ena.write(0);
    enb.write(0.25);
    in3=1;
    in4=0;
    
    }


void right(){
    ena.write(0.25);
    in1=1;
    in2=0;
    
    enb.write(0);
    }

void stop(){
     ena.write(0.9);
    in1=1;
    in2=1;
    enb.write(0.9);
    in3=1;
    in4=1;
    }
    
  

void Circle(){
    cur_error=CircleROT-gz;
    p=kp*cur_error;
    Right=Right-p;
    Left=Left+p;
    frwd(Right,Left);
    
    }






void Path(float gyroz,float dt){
   gyroz=gyroz;
   switch(rob_state){
       case(K1):
            Controller();
            
            if(stateTimer.read()>(12.0/(SCALE))){
                rob_state=K2;
            }
            break;
              
        case(K2):
            stateTimer.stop();

            if(pyaw<180-DEG_CORNER){
                left();
                pyaw-=gyroz*dt;
                
                }
            else{
                stop();
                wait(0.5);
                while(user_but){
                myled=!myled;
                wait(0.5);
                }
            wait(2);
            rob_state=K3;
            stateTimer.reset();
            stateTimer.start();
            pyaw=0;
                }
            break;
        case(K3):
            Controller();
            
            if(stateTimer.read()>(12.0/(SCALE)))
                rob_state=K4;
            break;
              
        case(K4):
            
            if(pyaw<90-DEG_CORNER){
                 right();
                pyaw+=gyroz*dt;
                }
            else{
                stop();
                wait(0.5);
                rob_state=K5;
                pyaw=0;
                stateTimer.reset();
                stateTimer.start();
                }
            break;
        case(K5):
            Controller();    
            if(stateTimer.read()>(3.0/(SCALE))){
                rob_state=K6;
             }
             
             
            break;
         case(K6):
             
            stop();
            
            if(pyaw<103-DEG_CORNER){
                right();
                pyaw+=gyroz*dt;
                }
            else{
                stop();
                wait(0.5);
                rob_state=K7;
                pyaw=0;
                stateTimer.reset();
                stateTimer.start();
                }
            break;
   
        case(K7):
            Controller();
  
            if(stateTimer.read()>(12.5/(SCALE))){
                stop();
                wait(0.5);
                while(user_but){
                    myled=!myled;
                    wait(0.5);
                }
                wait(2);
                rob_state=K8;
                pyaw=0;
                Right=PWM_R;
                Left=PWM_L;
                stateTimer.reset();
                stateTimer.start();
             }
            break;
         case(K8):
            Circle();
            if(pyaw<360-DEG_CORNER){
                
                pyaw-=gyroz*dt;
            }
            else {
                stop();
                while(1);
                }
   }
}



void initializeRob(){
    
   
    pc.baud(9600);  
    pc.printf( "Hello three wheel robotic car\r\n" ) ;    
    pc.printf("CPU SystemCoreClock is %d Hz\r\n", SystemCoreClock);   
    
    //Set up I2C in fast mode: 400 kHz   
    i2c.frequency(400000);  
    UltaTickerTrig.attach(&UltraTrigStart, 0.2 );   // every 200ms
    ultraEcho.rise(&UltraEchoStart) ;
    ultraEcho.fall(&UltraEchoEnd) ;

 
    uint8_t whoami = MPU9150.readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150); 
 
    if (whoami == 0x68){  
        pc.printf("MPU9150 WHO_AM_I is 0x%x\n\r", whoami);
        pc.printf("MPU9150 is online...\n\r");
        wait(0.5);
        MPU9150.MPU9150SelfTest(SelfTest);
        wait(0.2);
        MPU9150.resetMPU9150(); // Reset registers to default in preparation for device calibration
        wait(0.2);
        MPU9150.calibrateMPU9150(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
        wait(0.2);
        MPU9150.initMPU9150(); 
        wait(0.2);
        MPU9150.initAK8975A(magCalibration);
        wait(0.2);
    }
    else{
        
        while(1);
    }
    MPU9150.getAres(); // Get accelerometer sensitivity
    MPU9150.getGres(); // Get gyro sensitivity
    }


void ReadData(){
    if(MPU9150.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
        MPU9150.readAccelData(accelCount); 
        ax = (float)accelCount[0]*aRes; 
        ay =(float)accelCount[1]*aRes-meanay;   
        az = (float)accelCount[2]*aRes;  
        MPU9150.readGyroData(gyroCount);  // Read the x/y/z adc values
        gx = (float)gyroCount[0]*gRes;
        gy = (float)gyroCount[1]*gRes;  
        gz = (float)gyroCount[2]*gRes;//-meangz;
        gz_now=gz;
        ay_now=ay;
    
    }
}


void MeanAyGz(){
    unsigned int kl=0;
    for(kl=0;kl<20000;kl++){
        if(MPU9150.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
            MPU9150.readAccelData(accelCount); 
            ax = (float)accelCount[0]*aRes; 
            ay =(float)accelCount[1]*aRes;   
            az = (float)accelCount[2]*aRes;  
            MPU9150.readGyroData(gyroCount);  // Read the x/y/z adc values
            gx = (float)gyroCount[0]*gRes;
            gy = (float)gyroCount[1]*gRes;  
            gz = (float)gyroCount[2]*gRes;
            meanay+=ay;
            meangz+=gz;
            counter++;
                   
        }
    }
    meanay=meanay/counter;
    meangz=meangz/counter;
    pc.printf(" Mean ay=%f Mean gz=%f \r\n",meanay,meangz);
    
    
}
