#ifndef JPYPADXY_H
#define JPYPADXY_H

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
        double cc,xp,yp,thp;
        void joypadread();
    public:
        Joypadxy(int ff);
        void setunitcom(double cc);
        void setxp(double x);
        void setyp(double y);
        void setthp(double theta);
        double getunitcom();
        double getxp();
        double getyp();
        double getthp();
        int getendflag();
};

#endif