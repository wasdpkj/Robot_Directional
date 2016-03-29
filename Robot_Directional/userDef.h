#include"arduino.h"

#define _DEBUG  //DEBUG调试

#define SIN(X,Y) (float)(X/sqrt(pow(X,2)+pow(Y,2)))
#define COS(X,Y) (float)(Y/sqrt(pow(X,2)+pow(Y,2)))
#define V(X,Y)  (float)sqrt(pow(X,2) + pow(Y,2))

#define MOTOR_EN  4
int MOTOR_DIR[3] = {A0, A1, A2};
int MOTOR_STEP[3] = {5, 6, 7};

#define BLE_SPEED 9600  //蓝牙接口速度

#define SAFE_TIME_OUT 250   //失控保护时间

#define MAX_THROTTLE 1024 //最大油门
#define MAX_STEERING 512 //最大转向
#define MAX_ACCEL 128 //最大ACCEL 4 36

#define CHANNEL_THROTTLE  2 //油门通道
#define CHANNEL_STEERING  1 //转向通道
#define CHANNEL_ROTATE  3 //转向通道

