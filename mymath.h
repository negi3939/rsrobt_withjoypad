#ifndef MYMATH_H
#define MYMATH_H

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

using namespace Eigen;

namespace Mymath{
	class Funcvec{
		public:
			Funcvec();
			virtual VectorXd function(VectorXd x);
			virtual ~Funcvec();
	};
	
	double sign(double A);
	/*
	void eig(MatrixXd aA,MatrixXd bB,MatrixXd &eigenV,MatrixXd &eigenD);
	void eig(MatrixXd aA,MatrixXd &eigenV,MatrixXd &eigenD);
	*/
	MatrixXd inv(MatrixXd aA);
	MatrixXd absmat(MatrixXd aA);
	MatrixXd diffvec(VectorXd x,Funcvec *func);
}

#endif