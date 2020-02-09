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
#include <vector>
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

class Rsmotor{
    protected:
        int fd;
        int jointnum;
        int datanum;
        long vlength;
        std::vector<std::vector<double> > dataseries;
        std::string f_n;
		std::fstream fs;
        struct timeval start_time, end_time;
        double *data;
        VectorXd angle;
        VectorXd targetx;
        invkSolvenu *invk;
    public:
        Rsmotor(int ff,int jn);
        std::stringstream filename;
		std::string filenames;
        void setdhparameter(int ii,double thoff,double aal,double dis,double alp);
        void observe();
        void senddatasireis();
        void setangle(VectorXd ang);
        short getangle(int ID);
        short getspeed(int ID);
        short getcurrent(int ID);
        void getall(int ID,short &ang,short &speed,short &current);
        int torqueonoff(int SERVO_ID,short sMode);
        int settorque();
        int unsettorque();
        int move(int SERVO_ID,short sPos, unsigned short sTime);
        void move(Vector3d x);
        void fileout();
        ~Rsmotor();
};

Rsmotor::Rsmotor(int ff,int jn){
    fd = ff;
    jointnum = jn;
    datanum = 3*jn+1;
    vlength = 0;
    data = new double[datanum];
    angle.resize(jointnum);
    invk = new invkSolvenu(jn);
    targetx = VectorXd::Zero(7);
    invk->setcountlimit(1000);
}

void Rsmotor::setdhparameter(int ii,double thoff,double aal,double dis,double alp){
    invk->setdhparameter(ii,thoff,aal,dis,alp);
}

void Rsmotor::observe(){
    double databuf[datanum];
    short ang[jointnum],speed[jointnum],curr[jointnum];
    short bufang,bufspeed,bufcurr;
    gettimeofday(&end_time, NULL);
	long seconds = end_time.tv_sec - start_time.tv_sec; //seconds
   	long useconds = end_time.tv_usec - start_time.tv_usec; //milliseconds
	double time = (double)((seconds) + useconds/1000000.0);
    databuf[0] = time;
    for(int ii=0;ii<jointnum;ii++){
        getall(ii+1,bufang,bufspeed,bufcurr);
        ang[ii] = bufang;
        speed[ii] = bufspeed;
        curr[ii] = bufcurr;
    }
    for(ii=0;ii<jointnum;ii++){
        databuf[ii+1] = (double)ang[ii]/10.0d;
    }
    for(ii=0;ii<jointnum;ii++){
        databuf[ii+1+jointnum] = (double)speed[ii];
    }
    for(ii=0;ii<jointnum;ii++){
        databuf[ii+2+jointnum] = (double)curr[ii]/1000.0d;;
    }
    for(ii=0;ii<datanum;ii++){
        data[ii] = databuf[ii];
    }
    senddatasireis();
}

void Rsmotor::senddatasireis(){
	int ii,jj;
	std::vector<double> da(datanum);
	for(ii=0;ii<datanum;ii++){
		da.at(ii) = data[ii];
	}
	vlength++;
	dataseries.push_back(da);
}

void Rsmotor::setangle(VectorXd ang){
    angle = ang;
    double bufang;
    for(int ii=0;ii<jointnum;ii++){
        bufang = ang(ii)*180.0d/M_PI*10.0d;
        move(ii+1,(short)bufang,10);
    }
}

short Rsmotor::getangle(int ID){
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

short Rsmotor::getspeed(int ID){
    unsigned char sendbuf[32];
    unsigned char readbuf[128];
    unsigned char sum;
    int i;
    int ret;
    unsigned long len, readlen;
    short speed;
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
    speed = ((readbuf[12] << 8) & 0x0000FF00) | (readbuf[11] & 0x000000FF);
    return speed;
}

short Rsmotor::getcurrent(int ID){
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
    current = ((readbuf[14] << 8) & 0x0000FF00) | (readbuf[13] & 0x000000FF);
    return current;
}

void Rsmotor::getall(int ID,short &ang,short &speed,short &current){
    unsigned char sendbuf[32];
    unsigned char readbuf[128];
    unsigned char sum;
    int i;
    int ret;
    unsigned long len, readlen;
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
    ang = ((readbuf[8] << 8) & 0x0000FF00) | (readbuf[7] & 0x000000FF);
    speed = ((readbuf[12] << 8) & 0x0000FF00) | (readbuf[11] & 0x000000FF);
    current = ((readbuf[14] << 8) & 0x0000FF00) | (readbuf[13] & 0x000000FF);
}

int Rsmotor::torqueonoff(int SERVO_ID,short sMode){
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

int  Rsmotor::settorque(){
    for(int ii=1;ii<jointnum+1;ii++){
        torqueonoff(ii,1);
    }
    gettimeofday(&start_time, NULL);
}

int  Rsmotor::unsettorque(){
    for(int ii=1;ii<jointnum+1;ii++){
        torqueonoff(ii,0);
    }
}

int Rsmotor::move(int SERVO_ID,short sPos, unsigned short sTime){
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

void Rsmotor::move(Vector3d x){
    Vector4d qua;
    targetx.block(0,0,3,1) = x;
    Matrix4d mattheta = Matrix4d::Identity(4,4);
    mattheta(0,0) = cos(0.0d);
    mattheta(0,1) = sin(0.0d);
    mattheta(1,0) = -sin(0.0d);
    mattheta(1,1) = cos(0.0d);
    qua = invk->matrixtoquatanion(mattheta);
    targetx.block(3,0,3,1) = qua.block(0,0,3,1);
    targetx(6) = 0.0d*sign(qua(3));
    double bufang;
    invk->settargetfx(targetx);
    angle = invk->getangle(angle);
    std::cout << "x:" << 0.093d*cos(angle(0)-M_PI/2.0d) + 0.093d*cos(angle(0)+angle(1)-M_PI/2.0d) + 0.2d*cos(angle(0)+angle(1)+angle(2)-M_PI/2.0d) << std::endl;
    std::cout << "y:" << 0.093d*sin(angle(0)-M_PI/2.0d) + 0.093d*sin(angle(0)+angle(1)-M_PI/2.0d) + 0.2d*sin(angle(0)+angle(1)+angle(2)-M_PI/2.0d) << std::endl;
    for(int ii=0;ii<jointnum;ii++){
        bufang = angle(ii)*180.0d/M_PI*10.0d;
        move(ii+1,(short)bufang,10);
    }
}

void Rsmotor::fileout(){
    int ii,jj;
	f_n = filename.str();
	fs.open(f_n.c_str(),std::ios::out);
	if(! fs.is_open()) {
		std::cout << "=======cannot open file=========="<<std::endl;
		exit(1);
	}
	filename >> filenames;
	for(ii=0;ii<vlength;ii++){
		for(jj=0;jj<datanum;jj++){
			fs << std::fixed << std::setprecision(6) << dataseries.at(ii).at(jj) << " ";
		}
		fs << std::endl;
	}
	std::cout << "===========   file out:" << filenames <<" with vlenfth :" << vlength <<"  ==========="<<std::endl;
}

Rsmotor::~Rsmotor(){
    fileout();
}

int main(){
    int fdarm,fdjoy;
    fdarm = open(ROBODEV, O_RDWR);
    //fdjoy = open(JOYDEVNAME, O_RDWR);
    serial_init(fdarm);
    //serial_init(fdjoy);
    Vector3d ang;
    Vector3d targx;
    targx << 0.3d,0.1d,0.0d; 
    ang << 0.0d,0.0d,0.0d;
    Rsmotor rsm(fdarm,3);
    rsm.filename << "data/hogeangcur.dat";
    rsm.setdhparameter(0,-M_PI/2.0d,0.093d,0.0d,0.0d);
    rsm.setdhparameter(1,0.0d,0.093d,0.0d,0.0d);
    rsm.setdhparameter(2,0.0d,0.2d,0.0d,0.0d);
    rsm.settorque();
    rsm.setangle(ang);
    while(1){
        rsm.move(targx);
        rsm.observe();
        std::cout << " angle 1:" << rsm.getangle(1) << " angle 2:" <<rsm.getangle(2)<< " angle 3:" << rsm.getangle(3) << " currenr 1:" << rsm.getcurrent(1) << " currenr 2:" << rsm.getcurrent(2) << " currenr 3:" << rsm.getcurrent(3) << std::endl;;
        if(kbhit()){break;}   
    }
    rsm.unsettorque();
}