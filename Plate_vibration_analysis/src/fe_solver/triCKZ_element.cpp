
#include "triCKZ_element.h"

triCKZ_element::triCKZ_element()
{
	// Empty constructor
}

triCKZ_element::~triCKZ_element()
{
	// Empty destructor
}


Eigen::MatrixXd triCKZ_element::get_triCKZ_element_stiffness_matrix(const double& x1, const double& y1, 
	const double& x2, const double& y2, 
	const double& x3, const double& y3, 
	const double& thickness, const double& materialdensity, 
	const double& youngsmodulus, const double& poissonsratio)
{




	return Eigen::MatrixXd();
}


void triCKZ_element::computeElasticityMatrix(const double& youngsmodulus, const double& poissonsratio)
{
}


void triCKZ_element::computeTriangleIntegrationPoints()
{
}


void triCKZ_element::computeJacobianCoefficients(const double& x1, const double& y1, 
	const double& x2, const double& y2, 
	const double& x3, const double& y3)
{


}

void triCKZ_element::computeShapeFunctions(const double& b1, const double& b2, const double& b3, 
	const double& c1, const double& c2, const double& c3, 
	const double& L1, const double& L2, const double& L3)
{




}

