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
#include "serialsetting.h"

#define JOYDEVNAME "/dev/input/js0"

int main(){
    int ret;
    int fdjoy = open(JOYDEVNAME, O_RDWR);
    serial_init(fdjoy);
    /*
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
    */

   struct js_event js;
   while(1){
       ret = read(fdjoy, &js, sizeof(js));
        if (ret != sizeof(js)){
            std::cout << "ba-ka size is not same" << std::endl;
        }
        std::cout << " jsnumber: " << (int) js.number << " jsval: " << js.value << std::endl;
   }
}