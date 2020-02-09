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

#include "Joypadxy.h"

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