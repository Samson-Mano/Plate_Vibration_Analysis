#pragma once
#include "modal_analysis_solver.h"


class triCKZ_element
{
public:
	triCKZ_element();
	~triCKZ_element();

	Eigen::MatrixXd get_triCKZ_element_stiffness_matrix(
		const double& x1, const double& y1,
		const double& x2, const double& y2,
		const double& x3, const double& y3, 
		const double& thickness,
		const double& materialdensity,
		const double& youngsmodulus,
		const double& poissonsratio);


private:
	const int nip = 4; // number of integration point always for isotropic CKZ linear triangular plate element

	Eigen::MatrixXd elasticity_matrix; // 3 x 3 matrix saves the elasticity matrix based on youngsmodulus and poissons ratio
	Eigen::MatrixXd jacobianMatrix; // 2 x 3 matrix stores the jacobian J (dL/dx, dL/dy, etc)
	Eigen::MatrixXd jacobianProducts; // 3 x 6 matrix store the jacobian products J^2 ((dL/dx)^2, (dL/dy)^2, (dL/dx dL/dy), etc )
	Eigen::VectorXd shapeFunction; // 9 x 1 stores the shape function N
	Eigen::MatrixXd shapefunction_secondDerivativeMatrix; // 6 x 9 stores the d^2N second derivatives of the shape function
	Eigen::MatrixXd strainDisplacementMatrix; // 3 x 9 matrix stores the B strain Displacment matrix


	void computeElasticityMatrix(const double& youngsmodulus,
		const double& poissonsratio);


	void computeTriangleIntegrationPoints();



	void computeLocalCoordinateSystem(
		const Eigen::Vector3d& p,  // Point P
		const Eigen::Vector3d& q,  // Point Q
		const Eigen::Vector3d& r,  // Point R
		double& sin_angle,
		double& cos_angle,
		double& dpq,
		double& dpr, Eigen::Matrix3d& coordinateSystemE);

	

	void computeJacobianCoefficients(
		const double& x1, const double& y1,
		const double& x2, const double& y2,
		const double& x3, const double& y3);



	void computeShapeFunctions(
		const double& b1, const double& b2, const double& b3,
		const double& c1, const double& c2, const double& c3,
		const double& L1, const double& L2, const double& L3);


};