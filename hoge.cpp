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
#define JOYDEVNAME "/dev/input/js0"


class Joypadxy{
    private:
    static void* launchread(void *pParam) {
        	reinterpret_cast<Joypadxy*>(pParam)->joypadread();
        	pthread_exit(NULL);
    }
    pthread_t readthread;
    pthread_mutex_t readtex,breaktex;
    protected:
        int fd;
        int endflag;
        double cc,xp,yp;
        void joypadread();
    public:
        Joypadxy(int ff);
        void setunitcom(double cc);
        void setxp(double x);
        void setyp(double y);
        double getunitcom();
        double getxp();
        double getyp();
        int getendflag();
};

Joypadxy::Joypadxy(int ff){
    fd = ff;
    cc = 0.01;
    xp = 0.0d;
    yp = 0.0d;
    endflag = 0;
    pthread_mutex_init(&readtex,NULL);
    pthread_mutex_init(&breaktex,NULL);
    pthread_create(&readthread,NULL,Joypadxy::launchread,this);
}

void Joypadxy::setunitcom(double c){cc = c;}
void Joypadxy::setxp(double x){xp = x;}
void Joypadxy::setyp(double y){yp = y;}
double Joypadxy::getunitcom(){return cc;}
double Joypadxy::getxp(){return xp;}
double Joypadxy::getyp(){return yp;}

void Joypadxy::joypadread(){
    int ret;
    struct js_event js;
    double xl,yl;
    pthread_mutex_lock(&readtex);
    xl = xp;
    yl = yp;
    pthread_mutex_unlock(&readtex);
    while(1){
        ret = read(fd, &js, sizeof(js));
        if (ret != sizeof(js)){
            std::cout << "ba-ka size is not same" << std::endl;
        }
        if(js.number == 1){ //vertical
            if(js.value == -32767){
                yl = yl + cc;
                //std::cout << "up" << std::endl; 
            }else if(js.value == 32767){
                yl = yl - cc;
                //std::cout << "down" << std::endl;
            }
        }else if(js.number == 0){ //horizontal
            if(js.value == 32767){
                xl = xl + cc;
                //std::cout << "right" << std::endl;
            }else if(js.value == -32767){
                xl = xl - cc;
                //std::cout << "left" << std::endl;
            }else if(js.value ==1){break;}
        }
        pthread_mutex_lock(&readtex);
        xp = xl;
        yp = yl;
        pthread_mutex_unlock(&readtex);
    }
    pthread_mutex_lock(&breaktex);
    endflag = 1;
    pthread_mutex_unlock(&breaktex);
}

int Joypadxy::getendflag(){
    int flagl;
    pthread_mutex_lock(&breaktex);
    flagl = endflag;
    pthread_mutex_unlock(&breaktex);
    return flagl;
}

int main(){
    int ret;
    struct js_event js;
    int fdjoy = open(JOYDEVNAME, O_RDWR);
    serial_init(fdjoy);
    Vector3d targx;
    targx << 0.3d,0.02d,0.0d; 
    Joypadxy jsxy(fdjoy);
    jsxy.setxp(targx(0));
    jsxy.setyp(targx(1));
    while(1){
        targx(0) = jsxy.getxp();
        targx(1) = jsxy.getyp();
        std::cout << "x: " << targx(0) << " y:" <<targx(1) << std::endl;
        usleep(10000);
        if(jsxy.getendflag()){break;}
    }
}