#ifndef RSMOTOR_H
#define RSMOTOR_H

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
        invdSolvenu *invd;
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

#endif