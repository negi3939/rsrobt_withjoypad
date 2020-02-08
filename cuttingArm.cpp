#include <iostream>
#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <termio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/LU>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <linux/joystick.h>

#include "mymath.h"
#include "solvenu.h"
#include "inversekinematics.h"
#include "inversedynamics.h"
#include "serialsetting.h"

#define BAUDRATE B115200
#define ROBODEV "/dev/ttyUSB0"
#define JOYDEVNAME "/dev/input/js0"
#define _POSIX_SOURCE 1 //POSIX
#define BYTE unsigned char
/*
class Rsmotor{
    protected:
        int fd;
    public:
        Rsmotor(int ff);
        short RSGetAngle(int fd, int ID);
};*/

short RSGetAngle(int fd, int ID){
    unsigned char sendbuf[32];
    unsigned char readbuf[128];
    unsigned char sum;
    int i;
    int ret;
    unsigned long len, readlen;
    short angle;
    if (!fd){return -1;}
    memset(sendbuf, 0x00, sizeof(sendbuf));
    // パケット作成
    sendbuf[0] = (unsigned char)0xFA; // ヘッダー1
    sendbuf[1] = (unsigned char)0xAF; // ヘッダー2
    sendbuf[2] = (unsigned char)ID;   // サーボID
    sendbuf[3] = (unsigned char)0x09; // フラグ(0x01 | 0x04<<1)
    sendbuf[4] = (unsigned char)0x00; // アドレス(0x00)
    sendbuf[5] = (unsigned char)0x00; // 長さ(0byte)
    sendbuf[6] = (unsigned char)0x01; // 個数
    // チェックサムの計算
    sum = sendbuf[2];
    for (i = 3; i < 7; i++){
        sum = (unsigned char)(sum ^ sendbuf[i]);
    }
    sendbuf[7] = sum; // チェックサム
    tcflush(fd, TCIFLUSH);
    ret = write(fd, &sendbuf, 8);
    memset(readbuf, 0x00, sizeof(readbuf));
    readlen = 27;
    len = 0;
    ret = read(fd, readbuf, readlen);
    sum = 0;
    sum = readbuf[2];
    for (i = 3; i < 26; i++){
        sum = sum ^ readbuf[i];
    }
    if (sum){
        return -3;
    }
    angle = ((readbuf[8] << 8) & 0x0000FF00) | (readbuf[7] & 0x000000FF);
    return angle;
}

short RSGetCurrent(int fd, int ID){
    unsigned char sendbuf[32];
    unsigned char readbuf[128];
    unsigned char sum;
    int i;
    int ret;
    unsigned long len, readlen;
    short current;
    if (!fd){
        return -1;
    }
    memset(sendbuf, 0x00, sizeof(sendbuf));
    sendbuf[0] = (unsigned char)0xFA; // ヘッダー1
    sendbuf[1] = (unsigned char)0xAF; // ヘッダー2
    sendbuf[2] = (unsigned char)ID;   // サーボID
    sendbuf[3] = (unsigned char)0x09; // フラグ(0x01 | 0x04<<1)
    sendbuf[4] = (unsigned char)0x00; // アドレス(0x00)
    sendbuf[5] = (unsigned char)0x00; // 長さ(0byte)
    sendbuf[6] = (unsigned char)0x01; // 個数
    sum = sendbuf[2];
    for (i = 3; i < 7; i++)
    {
        sum = (unsigned char)(sum ^ sendbuf[i]);
    }
    sendbuf[7] = sum;
    tcflush(fd, TCIFLUSH);
    ret = write(fd, &sendbuf, 8);
    memset(readbuf, 0x00, sizeof(readbuf));
    readlen = 27;
    len = 0;
    ret = read(fd, readbuf, readlen);
    sum = 0;
    sum = readbuf[2];
    for (i = 3; i < 26; i++){
        sum = sum ^ readbuf[i];
    }
    if (sum){
        return -3;
    }
    current = ((readbuf[13] << 8) & 0x0000FF00) | (readbuf[12] & 0x000000FF);
    return current;
}

int RSTorqueOnOff(int SERVO_ID, int fd, short sMode){
    unsigned char sendbuf[28];
    unsigned char sum;
    int i;
    int ret;
    if (!fd){
        return -1;
    }
    memset(sendbuf, 0x00, sizeof(sendbuf));
    sendbuf[0] = (unsigned char)0xFA;             // ヘッダー1
    sendbuf[1] = (unsigned char)0xAF;             // ヘッダー2
    sendbuf[2] = (unsigned char)SERVO_ID;         // サーボID
    sendbuf[3] = (unsigned char)0x00;             // フラグ
    sendbuf[4] = (unsigned char)0x24;             // アドレス(0x24=36)
    sendbuf[5] = (unsigned char)0x01;             // 長さ(4byte)
    sendbuf[6] = (unsigned char)0x01;             // 個数
    sendbuf[7] = (unsigned char)(sMode & 0x00FF); // ON/OFFフラグ
    sum = sendbuf[2];
    for (i = 3; i < 8; i++){
        sum = (unsigned char)(sum ^ sendbuf[i]);
    }
    sendbuf[8] = sum; // チェックサム
    tcflush(fd, TCIFLUSH);
    ret = write(fd, &sendbuf, 9);
    return ret;
}

int RSMove(int SERVO_ID, int fd, short sPos, unsigned short sTime){
    unsigned char sendbuf[28];
    unsigned char sum;
    int i;
    int ret;
    if (!fd){return -1;}
    memset(sendbuf, 0x00, sizeof(sendbuf));
    sendbuf[0] = (unsigned char)0xFA;                     // ヘッダー1
    sendbuf[1] = (unsigned char)0xAF;                     // ヘッダー2
    sendbuf[2] = (unsigned char)SERVO_ID;                 // サーボID
    sendbuf[3] = (unsigned char)0x00;                     // フラグ
    sendbuf[4] = (unsigned char)0x1E;                     // アドレス(0x1E=30)
    sendbuf[5] = (unsigned char)0x04;                     // 長さ(4byte)
    sendbuf[6] = (unsigned char)0x01;                     // 個数
    sendbuf[7] = (unsigned char)(sPos & 0x00FF);          // 位置
    sendbuf[8] = (unsigned char)((sPos & 0xFF00) >> 8);   // 位置
    sendbuf[9] = (unsigned char)(sTime & 0x00FF);         // 時間
    sendbuf[10] = (unsigned char)((sTime & 0xFF00) >> 8); // 時間
    sum = sendbuf[2];
    for (i = 3; i < 11; i++){
        sum = (unsigned char)(sum ^ sendbuf[i]);
    }
    sendbuf[11] = sum; // チェックサム
    tcflush(fd, TCIFLUSH);
    ret = write(fd, &sendbuf, 12);
    return ret;
}

int main(){
    int fdarm,fdjoy;
    fdarm = open(ROBODEV, O_RDWR);
    fdjoy = open(JOYDEVNAME, O_RDWR);
    serial_init(fdarm);
    serial_init(fdjoy);
    for(int ii=1;ii<4;ii++){
        RSTorqueOnOff(ii,fdarm,1);
    }
    while(1){
        RSMove(1,fdarm, 900, 100);
        RSMove(2,fdarm, 0, 100);
        RSMove(3,fdarm, 0, 100);
        std::cout << " angle 1:" << RSGetAngle(fdarm, 1) << " angle 2:" << RSGetAngle(fdarm, 2) << " angle 3:" << RSGetAngle(fdarm, 3) << " currenr 1:" << RSGetCurrent(fdarm, 1) << " currenr 2:" << RSGetCurrent(fdarm, 2)<< " currenr 3:" << RSGetCurrent(fdarm, 3)  << std::endl;;
        if(kbhit()){break;}
    }
    for(ii=1;ii<4;ii++){
        RSTorqueOnOff(ii,fdarm,0);
    }
    
}