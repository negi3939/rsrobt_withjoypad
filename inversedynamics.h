#ifndef INVDYN_H
#define INVDYN_H
#include "solvenu.h"
#include "inversekinematics.h"

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;

class invdSolvenu : public invkSolvenu {
  protected:
    Matrix4d *aTt;
    Vector3d *rra;
    Vector3d *ppa;
    Vector3d *zz;
    MatrixXd *jacobi;
  public:
    invdSolvenu();
    invdSolvenu(int num);
    VectorXd funcorg(VectorXd x) override;
    void calcaTt();
    void calcjacobi();
    MatrixXd getjacobi();
    VectorXd gettau(VectorXd f,VectorXd mom);
    void calcforce(VectorXd tau,Vector3d &f,Vector3d &mom);
    VectorXd getvel(VectorXd x);
    ~invdSolvenu();
};

#endif