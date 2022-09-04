#include "M5Atom.h"
#include <Wire.h>          // I2C setting
#include <PCA9685.h>       //for PCA9685
#include <DabbleESP32.h>

//Sonic Sensor
#define echo 33 // Echo Pin
#define trig 23 // Trig Pin
double Duration = 0; //recieved duration
double Distance = 0; //distance

//IMU
float roll, pitch, yaw;
#define IMU_AFS M5.IMU.AFS_2G       // Ascale [g]      (±2,4,8,16)
#define IMU_GFS M5.IMU.GFS_250DPS  // Gscale [deg/s]  (±250,500,1000,200)
int roll_initial, pitch_initial, yaw_initial;
int initial_count = 100;

const float pi = 3.141593;

int angZero[] = {86,97,81,84,86,97,94,100,110,88,96,95,90};
int angHome[] = {0,-3,-12,18,-3,3,0,3,12,-18,3,-3,0};
int ang0[13];
int ang1[13];
int ang_b[13];
char ang_c[13];
float ts=30;  //30msごとに次のステップに移る
float td=10;   //10回で分割

float L1 = 70;
float L2 = 70;
float L0 = 9.7;

float H0 = 80; //Height
float Zc = 31.5; //Half Width
float Nee = 5;
float Ankle = 20;

float FS = 25; //FWDstep
float Ss = 25; //SideMove
float Dis = -5; //Distance
float Adj = 5; //Adjust
float Up = 30; //FootUp
float S1 = 20; //SideStep
float Tn = 10; //Turn
float CenX = 0; //CenterX
float CenZ = 0; //CenterZ

int direction_flag = 0;

// Base Step
int bs_s[27][13]={
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0}};
  
int delection = 0;

//PCA9685のアドレス指定
PCA9685 pwm = PCA9685(0x40);

#define SERVOMIN 104            //Min pulse width (12bit 500μs) 
#define SERVOMAX 512            //Max pulse width (12bit 2500μs) 

void servo_write(int ch, int ang){
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX); //angle（0～180）-> pulse width（150～500）
  pwm.setPWM(ch, 0, ang);
}

void mouth_clear(){
  for(int i=0; i<25; i++){
    M5.dis.drawpix(i, 0x000000); //black
  }
}

void mouth_close(){
  M5.dis.drawpix(11, 0xff0000);  //red
  M5.dis.drawpix(12, 0xff0000);
  M5.dis.drawpix(13, 0xff0000);
  M5.dis.drawpix(15, 0xff0000);
  M5.dis.drawpix(19, 0xff0000);
}

void mouth_open(){
  M5.dis.drawpix(1, 0xff0000);  //red
  M5.dis.drawpix(2, 0xff0000);
  M5.dis.drawpix(3, 0xff0000);
  M5.dis.drawpix(5, 0xff0000);
  M5.dis.drawpix(9, 0xff0000);
  M5.dis.drawpix(10, 0xff0000);
  M5.dis.drawpix(14, 0xff0000);
  M5.dis.drawpix(15, 0xff0000);
  M5.dis.drawpix(19, 0xff0000);
  M5.dis.drawpix(21, 0xff0000);
  M5.dis.drawpix(22, 0xff0000);
  M5.dis.drawpix(23, 0xff0000);
}

void mouth_bero(){
  M5.dis.drawpix(11, 0xff0000);  //red
  M5.dis.drawpix(12, 0xff0000);
  M5.dis.drawpix(13, 0xff0000);
  M5.dis.drawpix(15, 0xff0000);
  M5.dis.drawpix(19, 0xff0000);
  delay(100);
  M5.dis.drawpix(6, 0xff0000);  //red
  M5.dis.drawpix(7, 0xff0000);
  M5.dis.drawpix(8, 0xff0000);
  delay(100);
  M5.dis.drawpix(1, 0xff0000);  //red
  M5.dis.drawpix(2, 0xff0000);
  M5.dis.drawpix(3, 0xff0000);
}

void mouth_bero2(){
  M5.dis.drawpix(11, 0xff0000);  //red
  M5.dis.drawpix(12, 0xff0000);
  M5.dis.drawpix(13, 0xff0000);
  M5.dis.drawpix(15, 0xff0000);
  M5.dis.drawpix(19, 0xff0000);
  M5.dis.drawpix(6, 0xff0000);  //red
  M5.dis.drawpix(7, 0xff0000);
  M5.dis.drawpix(8, 0xff0000);
  M5.dis.drawpix(1, 0xff0000);  //red
  M5.dis.drawpix(2, 0xff0000);
  M5.dis.drawpix(3, 0xff0000);
}

void servo_init_set()
{
  int cn = 5;
  for (int j=0; j <=12; j++){
     servo_write(j+3,angZero[j]+ angHome[j]);
     delay(cn);
  }
}

void base_stay()
{
  float bs_s_init_ik[6]={-CenX, H0, Dis-CenZ, -CenX, H0, Dis+Ss*cos(pi/2)+CenZ};

  ik(&bs_s_init_ik[0], &bs_s_init_ik[1], &bs_s_init_ik[2], &bs_s_init_ik[3], &bs_s_init_ik[4], &bs_s_init_ik[5], &bs_s[0][1], &bs_s[0][2], &bs_s[0][3], &bs_s[0][4], &bs_s[0][5], &bs_s[0][7], &bs_s[0][8], &bs_s[0][9], &bs_s[0][10], &bs_s[0][11]);

  for (int j=0; j <=12 ; j++){
    ang1[j] = angZero[j] + bs_s[0][j];
  }
  servo_set();
}

void base_right_up_step(float Turn, float FS0, float FS1, float S2)
{
  float bs_s_ik[26][6]={
  {-FS0/2*(1-cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi/3)-CenZ, FS0/2*(1+cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi*2/3)+CenZ},
  {-FS0/2*(1-cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi/6)-CenZ, FS0/2*(1+cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi*5/6)+CenZ},
  {-FS0/2*(1-cos(pi))-CenX, H0, Dis+Ss*sin(pi/2)-CenZ, FS0/2*(1+cos(pi))-CenX, H0, Dis+Ss*cos(pi)+CenZ},
  {-FS0/2*(1-cos(pi))-CenX, H0-Adj, Dis+Ss-CenZ, -CenX, H0, Dis+Ss*cos(pi)+CenZ},
  {-FS0*sin(pi/2)*2/3-CenX, H0-Up*sin(pi/6), Dis+Ss-CenZ, -CenX, H0, Dis+Ss*cos(pi)+CenZ},
  {-FS0*sin(pi/2)/3-CenX, H0-Up*sin(pi/3), Dis+Ss-CenZ, -CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {-CenX, H0-Up*sin(pi/2), Dis+Ss-CenZ, -CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {FS1*sin(pi/2)/3-CenX, H0-Up*sin(pi/3)-CenZ, Dis+Ss+S2, -CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {FS1*sin(pi/2)*2/3-CenX, H0-Up*sin(pi/6)-CenZ, Dis+Ss+S2, -CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {FS1*sin(pi/2)-CenX, H0-Adj, Dis+Ss*sin(pi/2)-CenZ, -CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {FS1/2*(1+cos(pi/6))-CenX, H0-Adj, Dis+Ss*cos(pi/6)+S2-CenZ, -FS1/2*(1-cos(pi/6))-CenX, H0, Dis-Ss*sin(pi/3)+CenZ},
  {FS1/2*(1+cos(pi/3))-CenX, H0-Adj, Dis+Ss*cos(pi/3)+S2-CenZ, -FS1/2*(1-cos(pi/3))-CenX, H0, Dis-Ss*sin(pi/6)},
  {FS1/2*(1+cos(pi/2))-CenX, H0, Dis+S2/2-CenZ, -FS1/2*(1-cos(pi/2))-CenX, H0, Dis+S2/2},
  {FS1/2*(1+cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi*2/3)-CenZ, -FS1/2*(1-cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi/3)+S2+CenZ},
  {FS1/2*(1+cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi*5/6)-CenZ, -FS1/2*(1-cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi/6)+S2+CenZ},
  {FS1/2*(1+cos(pi))-CenX, H0, Dis+Ss*cos(pi)-CenZ, -FS1/2*(1-cos(pi))-CenX, H0, Dis+Ss*sin(pi/2)+S2+CenZ},
  {-CenX, H0, Dis+Ss*cos(pi)-CenZ, -FS1/2*(1-cos(pi))-CenX, H0-Adj, Dis+Ss+S2+CenZ},
  {-CenX, H0, Dis+Ss*cos(pi)-CenZ, -FS1*sin(pi/2)*2/3-CenX, H0-Up*sin(pi/6), Dis+Ss+S2+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, -FS1*sin(pi/2)/3-CenX, H0-Up*sin(pi/3), Dis+Ss+S2+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, -CenX, H0-Up*sin(pi/2), Dis+Ss+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, FS1*sin(pi/2)/3-CenX, H0-Up*sin(pi/3), Dis+Ss+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, FS1*sin(pi/2)*2/3-CenX, H0-Up*sin(pi/6), Dis+Ss+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, FS1*sin(pi/2)-CenX, H0-Adj, Dis+Ss*sin(pi/2)+CenZ},
  {-FS1/2*(1-cos(pi/6))-CenX, H0, Dis-Ss*sin(pi/3)-CenZ, FS1/2*(1+cos(pi/6))-CenX, H0-Adj, Dis+Ss*cos(pi/6)+CenZ},
  {-FS1/2*(1-cos(pi/3))-CenX, H0, Dis-Ss*sin(pi/6)-CenZ, FS1/2*(1+cos(pi/3))-CenX, H0-Adj, Dis+Ss*cos(pi/3)+CenZ},
  {-FS1/2*(1-cos(pi/2))-CenX, H0, Dis-CenZ, FS1/2*(1+cos(pi/2))-CenX, H0, Dis+Ss*cos(pi/2)+CenZ}};
  
  for (int i=0; i <=26 ; i++){
    ik(&bs_s_ik[i][0], &bs_s_ik[i][1], &bs_s_ik[i][2], &bs_s_ik[i][3], &bs_s_ik[i][4], &bs_s_ik[i][5], &bs_s[i][1], &bs_s[i][2], &bs_s[i][3], &bs_s[i][4], &bs_s[i][5], &bs_s[i][7], &bs_s[i][8], &bs_s[i][9], &bs_s[i][10], &bs_s[i][11]);
  }

  for (int i=6; i <=19 ; i++){
    bs_s[i][0]=Turn;
    bs_s[i][6]=-Turn;
    bs_s[i][12]=-Turn;
  }
  
  mouth_clear();
  mouth_bero2();
  for (int i=0; i <=25 ; i++){
    for (int j=0; j <=12 ; j++){
      ang1[j] = angZero[j] + bs_s[i][j];
    }
  servo_set();
  }

  for (int i=6; i <=19 ; i++){
    bs_s[i][0]=0;
    bs_s[i][6]=0;  
    bs_s[i][12]=0; 
  }
}

void base_left_up_step(float Turn, float FS0, float FS1, float S2)
{
  float bs_s_ik[26][6]={
  {FS0/2*(1+cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi*2/3)-CenZ, -FS0/2*(1-cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi/3)+CenZ},
  {FS0/2*(1+cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi*5/6)-CenZ -FS0/2*(1-cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi/6)+CenZ},
  {FS0/2*(1+cos(pi))-CenX, H0, Dis+Ss*cos(pi)-CenZ, -FS0/2*(1-cos(pi))-CenX, H0, Dis+Ss*sin(pi/2)+CenZ},
  {-CenX, H0, Dis+Ss*cos(pi)-CenZ, -FS0/2*(1-cos(pi))-CenX, H0-Adj, Dis+Ss+CenZ},
  {-CenX, H0, Dis+Ss*cos(pi)-CenZ, -FS0*sin(pi/2)*2/3-CenX, H0-Up*sin(pi/6), Dis+Ss+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, -FS0*sin(pi/2)/3-CenX, H0-Up*sin(pi/3), Dis+Ss+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ,-CenX, H0-Up*sin(pi/2), Dis+Ss+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, FS1*sin(pi/2)/3-CenX, H0-Up*sin(pi/3), Dis+Ss+S2+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, FS1*sin(pi/2)*2/3-CenX, H0-Up*sin(pi/6), Dis+Ss+S2+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, FS1*sin(pi/2)-CenX, H0-Adj, Dis+Ss*sin(pi/2)+CenZ},
  {-FS1/2*(1-cos(pi/6))-CenX, H0, Dis-Ss*sin(pi/3)-CenZ, FS1/2*(1+cos(pi/6))-CenX, H0-Adj, Dis+Ss*cos(pi/6)+S2+CenZ},
  {-FS1/2*(1-cos(pi/3))-CenX, H0, Dis-Ss*sin(pi/6)-CenZ, FS1/2*(1+cos(pi/3))-CenX, H0-Adj, Dis+Ss*cos(pi/3)+S2+CenZ},
  {-FS1/2*(1-cos(pi/2))-CenX, H0, Dis+S2/2-CenZ, FS1/2*(1+cos(pi/2))-CenX, H0, Dis+S2/2+CenZ},
  {-FS1/2*(1-cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi/3)+S2-CenZ, FS1/2*(1+cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi*2/3)+CenZ},
  {-FS1/2*(1-cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi/6)+S2-CenZ, FS1/2*(1+cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi*5/6)+CenZ},
  {-FS1/2*(1-cos(pi))-CenX, H0, Dis+Ss*sin(pi/2)+S2-CenZ, FS1/2*(1+cos(pi))-CenX, H0, Dis+Ss*cos(pi)+CenZ},
  {-FS1/2*(1-cos(pi))-CenX, H0-Adj, Dis+Ss+S2-CenZ,-CenX, H0, Dis+Ss*cos(pi)+CenZ},
  {-FS1*sin(pi/2)*2/3-CenX, H0-Up*sin(pi/6), Dis+Ss+S2-CenZ,-CenX, H0, Dis+Ss*cos(pi)+CenZ},
  {-FS1*sin(pi/2)/3-CenX, H0-Up*sin(pi/3), Dis+Ss+S2-CenZ,-CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {-CenX, H0-Up*sin(pi/2)-CenX, Dis+Ss-CenZ,-CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {FS1*sin(pi/2)/3-CenX, H0-Up*sin(pi/3), Dis+Ss-CenZ,-CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {FS1*sin(pi/2)*2/3-CenX, H0-Up*sin(pi/6), Dis+Ss-CenZ,-CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {FS1*sin(pi/2)-CenX, H0-Adj, Dis+Ss*sin(pi/2)-CenZ,-CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {FS1/2*(1+cos(pi/6))-CenX, H0-Adj, Dis+Ss*cos(pi/6)-CenZ, -FS1/2*(1-cos(pi/6))-CenX, H0, Dis-Ss*sin(pi/3)+CenZ},
  {FS1/2*(1+cos(pi/3))-CenX, H0-Adj, Dis+Ss*cos(pi/3)-CenZ, -FS1/2*(1-cos(pi/3))-CenX, H0, Dis-Ss*sin(pi/6)+CenZ},
  {FS1/2*(1+cos(pi/2))-CenX, H0, Dis+Ss*cos(pi/2)-CenZ, -FS1/2*(1-cos(pi/2))-CenX, H0, Dis+CenZ}};
  
  for (int i=0; i <=26 ; i++){
    ik(&bs_s_ik[i][0], &bs_s_ik[i][1], &bs_s_ik[i][2], &bs_s_ik[i][3], &bs_s_ik[i][4], &bs_s_ik[i][5], &bs_s[i][1], &bs_s[i][2], &bs_s[i][3], &bs_s[i][4], &bs_s[i][5], &bs_s[i][7], &bs_s[i][8], &bs_s[i][9], &bs_s[i][10], &bs_s[i][11]);
  }

  for (int i=6; i <=19 ; i++){
    bs_s[i][0]=Turn;
    bs_s[i][6]=-Turn;
    bs_s[i][12]=Turn;  
  }
  
  mouth_clear();
  mouth_bero2();
  for (int i=0; i <=25 ; i++){
    for (int j=0; j <=12 ; j++){
      ang1[j] = angZero[j] + bs_s[i][j];
    }
  servo_set();
  }

  for (int i=6; i <=19 ; i++){
    bs_s[i][0]=0;
    bs_s[i][6]=0;
    bs_s[i][12]=0;
    
  }
}

void base_left_up_stop(float FS1, float S2)
{
  float bs_s_ik[13][6]={
  {FS1/2*(1+cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi*2/3)-CenZ, -FS1/2*(1-cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi/3)+S2+CenZ},
  {FS1/2*(1+cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi*5/6)-CenZ, -FS1/2*(1-cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi/6)+S2+CenZ},
  {FS1/2*(1+cos(pi))-CenX, H0, Dis+Ss*cos(pi)-CenZ, -FS1/2*(1-cos(pi))-CenX, H0, Dis+Ss*sin(pi/2)+S2+CenZ},
  {-CenX, H0, Dis+Ss*cos(pi)-CenZ, -FS1/2*(1-cos(pi))-CenX, H0-Adj, Dis+Ss+S2+CenZ},
  {-CenX, H0, Dis+Ss*cos(pi)-CenZ, -FS1*sin(pi/2)*2/3-CenX, H0-Up*sin(pi/6), Dis+Ss+S2+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, -FS1*sin(pi/2)/3-CenX, H0-Up*sin(pi/3), Dis+Ss+S2+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, -CenX, H0-Up*sin(pi/2), Dis+Ss+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, -CenX, H0-Up*sin(pi/3), Dis+Ss+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, -CenX, H0-Up*sin(pi/6), Dis+Ss+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/2)-CenZ, -CenX, H0-Adj, Dis+Ss*sin(pi/2)+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/3)-CenZ, -CenX, H0-Adj, Dis+Ss*cos(pi/6)+CenZ},
  {-CenX, H0, Dis-Ss*sin(pi/6)-CenZ, -CenX, H0-Adj, Dis+Ss*cos(pi/3)+CenZ},
  {-CenX, H0, Dis-CenZ, -CenX, H0, Dis+Ss*cos(pi/2)}};
  
  for (int i=0; i <=12 ; i++){
    ik(&bs_s_ik[i][0], &bs_s_ik[i][1], &bs_s_ik[i][2], &bs_s_ik[i][3], &bs_s_ik[i][4], &bs_s_ik[i][5], &bs_s[i][1], &bs_s[i][2], &bs_s[i][3], &bs_s[i][4], &bs_s[i][5], &bs_s[i][7], &bs_s[i][8], &bs_s[i][9], &bs_s[i][10], &bs_s[i][11]);
  }
  
  for (int i=0; i <=12 ; i++){
    for (int j=0; j <=12 ; j++){
      ang1[j] = angZero[j] + bs_s[i][j];
    }
  servo_set();
  }
  mouth_clear();
  mouth_close();
}

void base_right_up_stop(float FS1, float S2)
{
  float bs_s_ik[13][6]={
  {-FS1/2*(1-cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi/3)+S2-CenZ, FS1/2*(1+cos(pi*2/3))-CenX, H0, Dis+Ss*cos(pi*2/3)+CenZ},
  {-FS1/2*(1-cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi/6)+S2-CenZ, FS1/2*(1+cos(pi*5/6))-CenX, H0, Dis+Ss*cos(pi*5/6)+CenZ},
  {-FS1/2*(1-cos(pi))-CenX, H0, Dis+Ss*sin(pi/2)+S2-CenZ, FS1/2*(1+cos(pi))-CenX, H0, Dis+Ss*cos(pi)+CenZ},
  {-FS1/2*(1-cos(pi))-CenX, H0-Adj, Dis+Ss+S2-CenZ,-CenX, H0, Dis+Ss*cos(pi)+CenZ},
  {-FS1*sin(pi/2)*2/3-CenX, H0-Up*sin(pi/6), Dis+Ss+S2-CenZ,-CenX, H0, Dis+Ss*cos(pi)+CenZ},
  {-FS1*sin(pi/2)/3-CenX, H0-Up*sin(pi/3), Dis+Ss+S2-CenZ,-CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {-CenX, H0-Up*sin(pi/2), Dis+Ss-CenZ,-CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {-CenX, H0-Up*sin(pi/3), Dis+Ss-CenZ,-CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {-CenX, H0-Up*sin(pi/6), Dis+Ss-CenZ,-CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {-CenX, H0-Adj, Dis+Ss*sin(pi/2)-CenZ,-CenX, H0, Dis-Ss*sin(pi/2)+CenZ},
  {-CenX, H0-Adj, Dis+Ss*cos(pi/6)-CenZ, -CenX, H0, Dis-Ss*sin(pi/3)+CenZ},
  {-CenX, H0-Adj, Dis+Ss*cos(pi/3)-CenZ, -CenX, H0, Dis-Ss*sin(pi/6)+CenZ},
  {-CenX, H0, Dis+Ss*cos(pi/2)-CenZ, -CenX, H0, Dis+CenZ}};
  
  for (int i=0; i <=12 ; i++){
    ik(&bs_s_ik[i][0], &bs_s_ik[i][1], &bs_s_ik[i][2], &bs_s_ik[i][3], &bs_s_ik[i][4], &bs_s_ik[i][5], &bs_s[i][1], &bs_s[i][2], &bs_s[i][3], &bs_s[i][4], &bs_s[i][5], &bs_s[i][7], &bs_s[i][8], &bs_s[i][9], &bs_s[i][10], &bs_s[i][11]);
  }

  for (int i=0; i <=12 ; i++){
    for (int j=0; j <=12 ; j++){
      ang1[j] = angZero[j] + bs_s[i][j];
    }
  servo_set();
  }
  mouth_clear();
  mouth_close();
}


void search_direction()
{
  double SDistance[21];
  double SDistanceRight=0.0;
  double SDistanceLeft=0.0;

  for (int i=0; i <=20; i++){
    ang1[12] = angZero[12]+15-i;
    servo_set();
    Distance_get();
    SDistance[i] = Distance;
    delay(150);
  }
  for (int i=12; i <=20; i++){
    SDistanceRight = SDistanceRight + SDistance[i];
  }
  for (int i=0; i <=8; i++){
    SDistanceLeft = SDistanceLeft + SDistance[i];
  }
  if(SDistanceRight >= SDistanceLeft){
    base_right_up_step(Tn, 0.0, 0.0, 0.0);
    base_right_up_step(Tn, 0.0, 0.0, 0.0);
  }
  else{
    base_left_up_step(Tn, 0.0, 0.0, 0.0);
    base_left_up_step(Tn, 0.0, 0.0, 0.0);
  }
}

void servo_set()
{
  int a[13],b[13];

  for (int j=0; j <=12 ; j++){
      a[j] = ang1[j] - ang0[j];
      b[j] = ang0[j];
      ang0[j] = ang1[j];
  }
  
  for (int k=0; k <= td; k++){

      servo_write(3,a[0]*float(k)/td+b[0]);
      servo_write(4,a[1]*float(k)/td+b[1]);
      servo_write(5,a[2]*float(k)/td+b[2]);
      servo_write(6,a[3]*float(k)/td+b[3]);
      servo_write(7,a[4]*float(k)/td+b[4]);
      servo_write(8,a[5]*float(k)/td+b[5]);
      servo_write(9,a[6]*float(k)/td+b[6]);
      servo_write(10,a[7]*float(k)/td+b[7]);
      servo_write(11,a[8]*float(k)/td+b[8]);
      servo_write(12,a[9]*float(k)/td+b[9]);
      servo_write(13,a[10]*float(k)/td+b[10]);
      servo_write(14,a[11]*float(k)/td+b[11]);
      servo_write(15,a[12]*float(k)/td+b[12]);

      delay(ts/td);
  }
}

void ik(float *X2R, float *H2R, float *Z2R, float *X2L, float *H2L, float *Z2L, int *Ang_1R_deg, int *Ang_2R_deg, int *Ang_3R_deg, int *Ang_4R_deg, int *Ang_5R_deg,int *Ang_1L_deg,int *Ang_2L_deg,int *Ang_3L_deg,int *Ang_4L_deg,int *Ang_5L_deg)
{
  //Right
    float Theta3R = atan2(*Z2R,*H2R);
    float LR = *H2R/cos(Theta3R);
    float Y2R = LR - L0;

    float Theta1R = acos((pow(*X2R,2) + pow(Y2R,2) + pow(L1,2) - pow(L2,2))/(2 * L1 * sqrt( pow(*X2R,2) + pow(Y2R,2)))) + atan2(Y2R,*X2R);
    float Theta2R = atan2((Y2R - L1 * sin(Theta1R)),(*X2R - L1 * cos(Theta1R))) - Theta1R;

    *Ang_1R_deg = round(Theta3R/pi*180);
    *Ang_2R_deg = round(180-Theta1R/pi*180-45);
    *Ang_3R_deg = round(-Theta2R/pi*180-90-Nee);
    *Ang_4R_deg = round(-(90-(180-Theta1R/pi*180)-(-Theta2R/pi*180)+45)-Ankle+Nee);
    *Ang_5R_deg = - *Ang_1R_deg;
  //Left
    float Theta3L = atan2(*Z2L,*H2L);
    float LL = *H2L/cos(Theta3L);
    float Y2L = LL - L0;

    float Theta1L = acos((pow(*X2L,2) + pow(Y2L,2) + pow(L1,2) - pow(L2,2))/(2 * L1 * sqrt( pow(*X2L,2) + pow(Y2L,2)))) + atan2(Y2L,*X2L);
    float Theta2L = atan2((Y2L - L1 * sin(Theta1L)),(*X2L - L1 * cos(Theta1L))) - Theta1L;

    *Ang_1L_deg = -round(Theta3L/pi*180);
    *Ang_2L_deg = -round(180-Theta1L/pi*180-45);
    *Ang_3L_deg = -round(-Theta2L/pi*180-90-Nee);
    *Ang_4L_deg = -round(-(90-(180-Theta1L/pi*180)-(-Theta2L/pi*180)+45)-Ankle+Nee);
    *Ang_5L_deg = - *Ang_1L_deg;

}

void Distance_get() 
{
  digitalWrite(trig,HIGH); //超音波を出力
  delayMicroseconds(10); //10μsパルスを出す
  digitalWrite(trig,LOW);
  Duration = pulseIn(echo,HIGH); //センサからパルス間隔を取得
  if (Duration > 0) {
    Distance = Duration/5.77; // パルス間隔から距離を計算 mm
    if (Distance > 500)
    {
      Distance = 500;
    }
  }
}

void IMU_get(void *pvParameters)
{
  while(1){
    M5.IMU.getAhrsData(&pitch, &roll, &yaw);
    pitch = pitch - pitch_initial;
    roll = roll - roll_initial;
    yaw = yaw - yaw_initial;
    
    if(fabsf(roll) <= 20.0){
      CenX = -roll*2.0;
    }
    if(fabsf(pitch) <= 20.0){
      CenZ = pitch*2.0;
    }

    Serial.printf("pitch:%.2f,roll:%.2f,yaw:%.2f,CenX:%.2f,CenZ:%.2f\r\n", pitch, roll, yaw, CenX, CenZ);

    delay(200);
  }
}

void angle_init()
{
  //initail_count 分だけ捨てる
  for (int j = 1; j < initial_count; j++) {
    M5.IMU.getAhrsData(&pitch, &roll, &yaw);
    delay(10);
  }
  
  //initail_count 分だけ初期値として平均値を求める
  for (int i = 1; i <= initial_count; i++) {
    M5.IMU.getAhrsData(&pitch, &roll, &yaw);
    pitch_initial += pitch;
    roll_initial += roll;
    yaw_initial += yaw;
    delay(10);
  }
  pitch_initial /= initial_count;
  roll_initial /= initial_count;
  yaw_initial /= initial_count;
}

void setup() 
{ 
  // void M5Atom::begin(bool SerialEnable , bool I2CenXable , bool DisplayEnable )
  M5.begin(true, false, true);
  //Serial.begin(151200);
  Dabble.begin("NX18_M5Atom");       //set bluetooth name of your device

  pinMode(echo,INPUT);
  pinMode(trig,OUTPUT);
  
  mouth_clear();
  mouth_open();

  Wire.begin(19, 22); //SDA-19, SCL-22
  
  pwm.begin();                   //initial setting (for 0x40) (PCA9685)
  pwm.setPWMFreq(50);            //PWM 50Hz (for 0x40) (PCA9685)

  //initial servo angle
  for (int j=0; j <=12 ; j++){
      ang0[j] = angZero[j] + angHome[j];
  }
  for (int j=0; j <=12 ; j++){
      ang1[j] = angZero[j] + angHome[j];
  }
  servo_init_set();
  
  mouth_clear();
  mouth_close();
  
  M5.IMU.Init();
  M5.IMU.SetGyroFsr(IMU_GFS);
  M5.IMU.SetAccelFsr(IMU_AFS);

  angle_init();

  xTaskCreatePinnedToCore(IMU_get, "IMU_get", 4096, NULL, 1, NULL, 0);

  base_stay();
} 

void loop() 
{   
  Dabble.processInput();             //this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.

  int ang_game = GamePad.getAngle();

  int rad = GamePad.getRadius();

  if ((rad > 2)&&((ang_game >= 60) && (ang_game < 120)))
  {
    Serial.println("FW BaseLeftUp");
    if(direction_flag == 0){
      mouth_clear();
      mouth_bero();
      base_left_up_step(0.0, 0.0, FS, 0.0);
      direction_flag = 1;
    }
    else if(((direction_flag == 4) || (direction_flag == 8))) {
      base_right_up_stop(FS, 0.0);
      direction_flag = 0;
    }
    else{
      base_left_up_step(0.0, FS, FS, 0.0);
      direction_flag = 1;
    }
  }

  if ((rad > 2)&&((ang_game >= 120) && (ang_game < 150)))
  {
    Serial.println("FW_Left BaseLeftUp");
    if(direction_flag == 0){
      mouth_clear();
      mouth_bero();
      base_left_up_step(0.0, 0.0, FS, S1);
      direction_flag = 2;
    }
    else if(((direction_flag == 4) || (direction_flag == 8))) {
      base_right_up_stop(FS, 0.0);
      direction_flag = 0;
    }
    else{
      base_left_up_step(0.0, FS, FS, S1);
      direction_flag = 2;
    }
  }

  if ((rad > 2)&&((ang_game >= 150) && (ang_game < 210)))
  {
    Serial.println("Left Base LeftUp");
    base_left_up_step(0.0, 0.0, 0.0, S1);
    mouth_clear();
    mouth_close();
    direction_flag = 0;
  }
  
  if ((rad > 2)&&((ang_game >= 210) && (ang_game < 240)))
  {
    Serial.println("Back_Left BaseLeftUp");
    if(direction_flag == 0){
      mouth_clear();
      mouth_bero();
      base_left_up_step(0.0, 0.0, -FS, S1);
      direction_flag = 4;
    }
    else if((((direction_flag == 1) || (direction_flag == 2)) || (direction_flag == 5)) || (direction_flag == 6)) {
      base_right_up_stop(-FS, 0.0);
      direction_flag = 0;
    }
    else{
      base_left_up_step(0.0, -FS, -FS, S1);
      direction_flag = 4;
    }
  }
  
  if ((rad > 2)&&((ang_game >= 240) && (ang_game < 300)))
  {
    Serial.println("Back BaseRightUp");
    if(direction_flag == 0){
      mouth_clear();
      mouth_bero();
      base_right_up_step(0.0, 0.0, -FS, 0.0);
      direction_flag = 5;
    }
    else if(((direction_flag == 4) || (direction_flag == 8))) {
      base_left_up_stop(-FS, 0.0);
      direction_flag = 0;
    }
    else{
      base_right_up_step(0.0, -FS, -FS, 0.0);
      direction_flag = 5;
    }
  }

  if ((rad > 2)&&((ang_game >= 300) && (ang_game < 330)))
  {
    Serial.println("Back_Right BaseRightUp");
    if(direction_flag == 0){
      mouth_clear();
      mouth_bero();
      base_right_up_step(0.0, 0.0, -FS, S1);
      direction_flag = 6;
    }
    else if(((direction_flag == 4) || (direction_flag == 8))) {
      base_left_up_stop(-FS, 0.0);
      direction_flag = 0;
    }
    else{
      base_right_up_step(0.0, -FS, -FS, S1);
      direction_flag = 6;
    }
  }

  if ((rad > 2)&&((ang_game >= 330) || (ang_game < 30)))
  {
    Serial.println("Right BaseRightUp");
    base_right_up_step(0.0, 0.0, 0.0, S1);
    mouth_clear();
    mouth_close();
    direction_flag = 0;
  }

  if ((rad > 2)&&((ang_game >= 30) && (ang_game < 60)))
  {
    Serial.println("FW_Right BaseRightUp");
    if(direction_flag == 0){
      mouth_clear();
      mouth_bero();
      base_right_up_step(0.0, 0.0, FS, S1);
      direction_flag = 8;
    }
    else if((((direction_flag == 1) || (direction_flag == 2)) || (direction_flag == 5)) || (direction_flag == 6)) {
      base_left_up_stop(FS, 0.0);
      direction_flag = 0;
    }
    else{
      base_right_up_step(0.0, FS, FS, S1);
      direction_flag = 8;
    }
  }

  if (rad == 0)
  {
    if(direction_flag == 1){
      Serial.println("FW BaseLeftUp_Stop");
      base_left_up_stop(FS, 0.0);
      mouth_clear();
      mouth_close();
    }
    if(direction_flag == 2){
      Serial.println("FW_LeftBaseLeftUp_Stop");
      base_left_up_stop(FS, 0.0);
      mouth_clear();
      mouth_close();
    }
    if(direction_flag == 4){
      Serial.println("Back_LeftBaseLeftUp_Stop");
      base_left_up_stop(-FS, 0.0);
      mouth_clear();
      mouth_close();
    }
    if(direction_flag == 5){
      Serial.println("Back BaseRightUp_Stop");
      base_right_up_stop(-FS, 0.0);
      mouth_clear();
      mouth_close();
    }
    if(direction_flag == 6){
      Serial.println("Back_RightBaseRightUp_Stop");
      base_right_up_stop(-FS, 0.0);
      mouth_clear();
      mouth_close();
    }
    if(direction_flag == 8){
      Serial.println("FW_Right BaseRightUp_Stop");
      base_right_up_stop(FS, 0.0);
      mouth_clear();
      mouth_close();
    }
    direction_flag = 0;
  }
  
  if (GamePad.isSquarePressed())
  {
    Serial.println("LEFT STEP");
    base_left_up_step(Tn, 0.0, 0.0, 0.0);
    mouth_clear();
    mouth_close();
  }

  if (GamePad.isCirclePressed())
  {
    Serial.println("RIGHT STEP");
    base_right_up_step(Tn, 0.0, 0.0, 0.0);
    mouth_clear();
    mouth_close();
  }

  if (GamePad.isCrossPressed())
  {
    Serial.print("Hight Down");
    H0 = H0 - 5.0;
    base_stay();
  }

  if (GamePad.isTrianglePressed())
  {
    Serial.print("Hight Up");
    H0 = H0 + 5.0;
    base_stay();
  }

  if (GamePad.isUpPressed())
  {
    Serial.println("Sonic_Direction");
    while(1){
      Distance_get();
      Serial.println(Distance);
      if((direction_flag == 0) && (Distance >160)){
        mouth_clear();
        mouth_bero();
        base_left_up_step(0.0, 0.0, FS, 0.0);
        direction_flag = 1;
      }
      else if((direction_flag == 1) && (Distance <= 160)) {
        base_left_up_stop(FS, 0.0);
        base_right_up_step(0.0, 0.0, -FS, 0.0);
        base_right_up_stop(-FS, 0.0);
        direction_flag = 0;
        mouth_clear();
        mouth_close();
        search_direction();
      }
      else if((direction_flag == 0) && (Distance <=160)){
        base_right_up_step(0.0, 0.0, -FS, 0.0);
        base_right_up_stop(-FS, 0.0);
        mouth_clear();
        mouth_open();
        search_direction();
      }
      else{
        base_left_up_step(0.0, FS, FS, 0.0);
        direction_flag = 1;
      }
    }
  }

  if (GamePad.isDownPressed())
  {
    Serial.println("Sonic_Distrance");
    while(1){
      Distance_get();
      Serial.println(Distance);
      if((direction_flag == 0) && (Distance >200)){
        mouth_clear();
        mouth_bero();
        base_left_up_step(0.0, 0.0, FS, 0.0);
        direction_flag = 1;
      }
      else if((direction_flag == 1) && (Distance <= 200)) {
        base_left_up_stop(FS, 0.0);
        base_right_up_step(0.0, 0.0, -FS, 0.0);
        direction_flag = 5;
        mouth_clear();
        mouth_close();
      }
      else if((direction_flag == 5) && (Distance <=200)){
        base_right_up_step(0.0, -FS, -FS, 0.0);
        direction_flag = 5;
      }
      else if((direction_flag == 5) && (Distance >200)){
        base_right_up_stop(-FS, 0.0);
        mouth_clear();
        mouth_bero();
        direction_flag = 0;
      }
      else{
        base_left_up_step(0.0, FS, FS, 0.0);
        direction_flag = 1;
      }
    }
  }
  base_stay();
}
