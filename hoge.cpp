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
//#define JOYDEVNAME "/dev/input/js0"

int main(){
    //int fdjoy = open(JOYDEVNAME, O_RDWR);
    invkSolvenu invk(3);
    invk.setdhparameter(0,-M_PI/2.0d,0.093d,0.0d,0.0d);
    invk.setdhparameter(1,0.0d,0.093d,0.0d,0.0d);
    invk.setdhparameter(2,0.0d,0.2d,0.0d,0.0d);
    MatrixXd mattheta;
    mattheta = MatrixXd::Zero(4,4);
    VectorXd qua,targetx,angle;
    angle = VectorXd::Zero(3);
    Vector3d x;
    x << 0.3d,0.01d,0.0d;
    targetx = VectorXd::Zero(7);
    targetx.block(0,0,3,1) = x;
    mattheta(0,0) = cos(0.0d);
    mattheta(0,1) = sin(0.0d);
    mattheta(1,0) = -sin(0.0d);
    mattheta(1,1) = cos(0.0d);
    qua = invk.matrixtoquatanion(mattheta);
    targetx.block(3,0,3,1) = qua.block(0,0,3,1);
    targetx(6) = 0.0d*sign(qua(3));
    invk.settargetfx(targetx);
    angle = invk.getangle(angle);
    PRINT_MAT(angle);
    std::cout << "x:" << 0.093d*cos(angle(0)-M_PI/2.0d) + 0.093d*cos(angle(0)+angle(1)-M_PI/2.0d) + 0.2d*cos(angle(0)+angle(1)+angle(2)-M_PI/2.0d) << std::endl;
    std::cout << "y:" << 0.093d*sin(angle(0)-M_PI/2.0d) + 0.093d*sin(angle(0)+angle(1)-M_PI/2.0d) + 0.2d*sin(angle(0)+angle(1)+angle(2)-M_PI/2.0d) << std::endl;
}