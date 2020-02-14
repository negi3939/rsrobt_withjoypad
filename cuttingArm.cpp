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
#include "Joypadxy.h"
#include "Rsmotor.h"
#include "serialsetting.h"

#define BAUDRATE B115200
#define ROBODEV "/dev/ttyUSB0"
#define JOYDEVNAME "/dev/input/js0"
#define _POSIX_SOURCE 1 //POSIX
#define BYTE unsigned char


int main(int ac,char *av[]){
    std::vector<double> timef,xf,yf,thetaf;
    if(ac>1){
        std::ifstream ifs(av[1]);
        if (!ifs){
            std::cout << "cannot read file" << std::endl; //エラー
        }
        std::string line;
        for( int row = 0;getline(ifs, line);++row ) { // 1行読んで
            std::istringstream stream(line);
            double data;
            for(int col = 0; stream >> data; ++col) { // 1個ずつ切り分ける
                switch (col){
                    case 0:
                        timef.push_back(data);
                        break; 
                    case 4:
                        xf.push_back(data);
                        break;
                    case 5:
                        yf.push_back(data);
                        break;
                    case 6:
                        thetaf.push_back(data);
                        break;
                    default:
                        break;
                }
            }
        }
        ifs.close();
    }
    
    int fdarm,fdjoy;
    int ret,numstep=0;
    fdarm = open(ROBODEV, O_RDWR);
    fdjoy = open(JOYDEVNAME, O_RDWR);
    serial_init(fdarm);
    serial_init(fdjoy);
    Joypadxy jsxy(fdjoy);
    Vector3d ang;
    Vector3d targx;
    double theta = 0.0d;
    targx << 0.3d,0.02d,0.0d;
    jsxy.setxp(targx(0));
    jsxy.setyp(targx(1));
    jsxy.setthp(theta);
    ang << M_PI/2.0d,0.0d,0.0d;
    Rsmotor rsm(fdarm,3);
    rsm.filename << "data/hogeangcur.dat";
    rsm.setdhparameter(0,-M_PI/2.0d,0.093d,-0.0095d,0.0d);
    rsm.setdhparameter(1,0.0d,0.093d,-0.0095d,0.0d);
    rsm.setdhparameter(2,0.0d,0.2d,-0.0095d,0.0d);
    rsm.settorque();
    if(ac>1){
        targx(0) = xf[numstep];
        targx(1) = yf[numstep];
        theta = thetaf[numstep];
        rsm.move(targx,theta);
        sleep(3);
    }
    while(0){rsm.setangle(ang);}
    while(1){
        if(ac>1){
            targx(0) = xf[numstep];
            targx(1) = yf[numstep];
            theta = thetaf[numstep];
            sleep(1);
            numstep++;
        }else{
            targx(0) = jsxy.getxp();
            targx(1) = jsxy.getyp();
            theta = jsxy.getthp();
        }
        rsm.move(targx,theta);
        rsm.observe();
        //std::cout << " angle 1:" << rsm.getangle(1) << " angle 2:" <<rsm.getangle(2)<< " angle 3:" << rsm.getangle(3) << " currenr 1:" << rsm.getcurrent(1) << " currenr 2:" << rsm.getcurrent(2) << " currenr 3:" << rsm.getcurrent(3) << std::endl;;
        if(kbhit()){break;}
        if(jsxy.getendflag()){break;}   
    }
    rsm.unsettorque();
    close(fdarm);
}