#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/times.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <bitset>
#include <termios.h>
#include <pthread.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/LU>
#include "mymath.h"
#include "solvenu.h"
#include "inversekinematics.h"
#include "inversedynamics.h"

invdSolvenu::invdSolvenu():invkSolvenu(){
    aTt = new Matrix4d[jointnum];
    rra = new Vector3d[jointnum];
    ppa = new Vector3d[jointnum];
    zz = new Vector3d[jointnum];
    jacobi = new MatrixXd;
    *jacobi= MatrixXd::Zero(6,jointnum);
    rra[0] = Vector3d::Zero(3,1);
    zz[0] << 0.0d,0.0d,1.0d;
}
invdSolvenu::invdSolvenu(int num):invkSolvenu(num){
    aTt = new Matrix4d[jointnum];
    rra = new Vector3d[jointnum];
    ppa = new Vector3d[jointnum];
    zz = new Vector3d[jointnum];
    jacobi = new MatrixXd;
    *jacobi = MatrixXd::Zero(6,jointnum);
    rra[0] = Vector3d::Zero(3,1);
    zz[0] << 0.0d,0.0d,1.0d;
}
invdSolvenu::~invdSolvenu(){delete jacobi;delete[] ppa;delete[] rra;delete[] aTt;delete[] zz;}

VectorXd invdSolvenu::funcorg(VectorXd x){
    calcaTt();
    calcjacobi();
    return (*jacobi)*x;
}

void invdSolvenu::calcaTt(){
    Matrix4d buff = aA[0];
    aTt[0] = buff;
    for(int ii=1;ii<jointnum;ii++){
        buff = buff*aA[ii];
        aTt[ii] = buff;
    }
}
void invdSolvenu::calcjacobi(){
    Vector3d rrend;
    rrend = aTt[jointnum-1].block(0,3,3,1);
    ppa[0] = rrend;
    jacobi->block(0,0,3,1) = zz[0].cross(ppa[0]);
    jacobi->block(3,0,3,1) = zz[0];
    for(int ii=1;ii<jointnum;ii++){
        rra[ii] = aTt[ii-1].block(0,3,3,1);
        zz[ii] = aTt[ii-1].block(0,2,3,1);
        ppa[ii] = rrend - rra[ii];
        jacobi->block(0,ii,3,1) = zz[ii].cross(ppa[ii]);
        jacobi->block(3,ii,3,1) = zz[ii];
    }
}

MatrixXd invdSolvenu::getjacobi(){
    calcaTt();
    calcjacobi();
    return (*jacobi);//動作未確認
}

VectorXd invdSolvenu::gettau(VectorXd f,VectorXd mom){
    calcaTt();
    calcjacobi();
    VectorXd fm,tau;
    fm.resize(f.size()+mom.size(),1);
    fm.block(0,0,f.size(),1) = f;
    fm.block(f.size(),0,mom.size(),1) = mom;
    tau = jacobi->transpose()*fm;
    return tau;//動作未確認
}

void invdSolvenu::calcforce(VectorXd tau,Vector3d &f,Vector3d &mom){
    calcaTt();
    calcjacobi();
    VectorXd fmom(6);
    fmom.block(0,0,3,1) = f;
    fmom.block(3,0,3,1) = mom;
    JacobiSVD<MatrixXd> svd(jacobi->transpose(),ComputeThinU|ComputeThinV);
    fmom = svd.solve(tau);
    f = fmom.block(0,0,3,1);
    mom = fmom.block(3,0,3,1);//動作未確認
}

VectorXd invdSolvenu::getvel(VectorXd x){
    VectorXd vel = solve(x);
    return vel;
}