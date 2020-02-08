#ifndef SOLVENU_H
#define SOLVENU_H

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;

class Solvenu : public Funcvec{
    protected:
        long countlimit;
        VectorXd targetfx;
        VectorXd x;
    public:
        Solvenu();
        void setcountlimit(long a);
        void settargetfx(VectorXd tfx);
        VectorXd gettargetfx();
        VectorXd function(VectorXd x) override;
        virtual VectorXd funcorg(VectorXd x);
        VectorXd functionerror(VectorXd x);
        VectorXd solve(VectorXd intx);
        ~Solvenu();
};

#endif