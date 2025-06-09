
#include "triCKZ_element.h"

triCKZ_element::triCKZ_element()
{
	// Empty constructor
}

triCKZ_element::~triCKZ_element()
{
	// Empty destructor
}

void triCKZ_element::init(const double& youngsmodulus, const double& poissonsratio)
{
	// Intialize the CKZ triangle element module
		// Step 0A: Set the elasticity matrix (from youngs modulus, poissons ratio)
	computeElasticityMatrix(youngsmodulus, poissonsratio);

	// Step 0B: Set the Triangle Numerical Integration Point
	computeTriangleIntegrationPoints();

}


void triCKZ_element::computeElasticityMatrix(const double& youngsmodulus, const double& poissonsratio)
{

	// compute the elasticity matrix
	elasticity_matrix.resize(3, 3);
	elasticity_matrix.setZero();

	double k_const = youngsmodulus / (1.0 - (poissonsratio * poissonsratio));

	// Row 1
	elasticity_matrix.coeffRef(0, 0) = 1.0;
	elasticity_matrix.coeffRef(0, 1) = poissonsratio;
	elasticity_matrix.coeffRef(0, 2) = 0.0;

	// Row 2
	elasticity_matrix.coeffRef(1, 0) = poissonsratio;
	elasticity_matrix.coeffRef(1, 1) = 1.0;
	elasticity_matrix.coeffRef(1, 2) = 0.0;

	// Row 3
	elasticity_matrix.coeffRef(2, 0) = 0.0;
	elasticity_matrix.coeffRef(2, 1) = 0.0;
	elasticity_matrix.coeffRef(2, 2) = (1.0 - poissonsratio) * 0.5;


	elasticity_matrix = k_const * elasticity_matrix;


}


void triCKZ_element::computeTriangleIntegrationPoints()
{

	// compute the Triangle Area coordinates Integration Points
	integration_points.resize(4, 4);
	integration_points.setZero();

	// Row 1 coordinate 1
	integration_points.coeffRef(0, 0) = 0.6;
	integration_points.coeffRef(0, 1) = 0.2;
	integration_points.coeffRef(0, 2) = 0.2;
	integration_points.coeffRef(0, 3) = 25.0 / 48.0; // weight 1

	// Row 2 coordinate 2
	integration_points.coeffRef(1, 0) = 0.2;
	integration_points.coeffRef(1, 1) = 0.6;
	integration_points.coeffRef(1, 2) = 0.2;
	integration_points.coeffRef(1, 3) = 25.0 / 48.0;  // weight 2

	// Row 3 coordinate 3
	integration_points.coeffRef(2, 0) = 0.2;
	integration_points.coeffRef(2, 1) = 0.2;
	integration_points.coeffRef(2, 2) = 0.6;
	integration_points.coeffRef(2, 3) = 25.0 / 48.0;  // weight 3

	// Row 4 centroid
	integration_points.coeffRef(3, 0) = 1.0 / 3.0;
	integration_points.coeffRef(3, 1) = 1.0 / 3.0;
	integration_points.coeffRef(3, 2) = 1.0 / 3.0;
	integration_points.coeffRef(3, 3) = -27.0 / 48.0;  // weight 4


}


Eigen::MatrixXd triCKZ_element::get_triCKZ_element_stiffness_matrix(const double& x1, const double& y1, const double& z1,
	const double& x2, const double& y2, const double& z2,
	const double& x3, const double& y3, const double& z3,
	const double& thickness, const double& materialdensity)
{

	// Step 1: Set the local co-ordinate system for the triangle
	computeLocalCoordinateSystem(x1, y1, z1, x2, y2, z2, x3, y3, z3);

	// Step 2: Set the Jacobian Co-efficients
	computeJacobianCoefficients();

	// Parameters for calculating Consistent mass matrix
	double elem_volume = this->triangle_area * thickness;
	double element_mass_scaling = (elem_volume * materialdensity) / 12.0;


	// Initialize the element consistent mass matrix and element stiffness matrix
	this->element_consistentmassMatrix = Eigen::MatrixXd::Zero(9, 9);
	this->element_stiffness_matrix = Eigen::MatrixXd::Zero(9, 9);
	

	for (int i = 0; i < 4; i++)
	{
		double L1 = integration_points.coeffRef(0, 0) = 0.6;
		double L2 = integration_points.coeffRef(0, 1) = 0.2;
		double L3 = integration_points.coeffRef(0, 2) = 0.2;
		double int_wt = integration_points.coeffRef(0, 3) = 25.0 / 48.0; // weight 1
		double hxy = this->triangle_area * int_wt;

		double t_eff = (std::pow(thickness, 3)) / 12.0;
		double factor = t_eff * hxy;

		//___________________________________________________________________________________________________________________
		// Calculate the consistent mass matrix
		// Compute outer product N^T * N 9 x 1 * 1 x 9
		Eigen::MatrixXd mass_matrix_ip = this->shapeFunction * this->shapeFunction.transpose(); // 9x9

		// Scale and accumulate Mass
		this->element_consistentmassMatrix = this->element_consistentmassMatrix + (element_mass_scaling * int_wt * mass_matrix_ip);


		//___________________________________________________________________________________________________________________
		// Strain Displacement Matrix at Integration Points
		Eigen::MatrixXd  strainDisplacement_Bmatrix = computeStrainDisplacementMatrix();  // B is 3x9 matrix

		// Compute E = B^T * D * B
		Eigen::MatrixXd bending_stiffness_matrix_IP = strainDisplacement_Bmatrix.transpose() * this->elasticity_matrix * strainDisplacement_Bmatrix;

		// Apply integration weight factor
		bending_stiffness_matrix_IP = factor * bending_stiffness_matrix_IP;


		this->element_stiffness_matrix = this->element_stiffness_matrix + bending_stiffness_matrix_IP;


	}



	return Eigen::MatrixXd();
}


void triCKZ_element::computeLocalCoordinateSystem(const double& x1, const double& y1, const double& z1,
	const double& x2, const double& y2, const double& z2,
	const double& x3, const double& y3, const double& z3)
{

	Eigen::Vector3d p(x1, y1, z1);  // Point P
	Eigen::Vector3d q(x2, y2, z2);  // Point Q
	Eigen::Vector3d r(x3, y3, z3);  // Point R


	// Vectors PQ and PR
	Eigen::Vector3d v_pq = q - p;
	Eigen::Vector3d v_pr = r - p;

	double dpq = v_pq.norm();
	double dpr = v_pr.norm();

	// Normal vector to the triangle (Z-axis)
	Eigen::Vector3d v_z = v_pq.cross(v_pr);
	double dz = v_z.norm();

	// Sine and Cosine of angle between PQ and PR
	double sin_angle = dz / (dpq * dpr);
	double cos_angle = v_pq.dot(v_pr) / (dpq * dpr);

	// Unit vectors for local coordinate system
	Eigen::Vector3d x_local = v_pq.normalized();           // Local x-axis
	Eigen::Vector3d z_local = v_z.normalized();            // Local z-axis
	Eigen::Vector3d y_local = z_local.cross(x_local);      // Local y-axis

	// Construct transformation matrix E (columns: X, Y, Z)
	Eigen::Matrix3d coordinateSystemE;

	coordinateSystemE.col(0) = x_local;
	coordinateSystemE.col(1) = y_local;
	coordinateSystemE.col(2) = z_local;

	//_______________________________________________________________________
	// Set the local variables
	this->x1 = 0.0;
	this->y1 = 0.0;
	this->x2 = dpq;
	this->y2 = 0.0;
	this->x3 = dpr * cos_angle;
	this->y3 = dpr * sin_angle;

	this->triangle_area = 0.5 * x2 * y3;

}



void triCKZ_element::computeJacobianCoefficients()
{
	// Reciprocal of 2 × triangle area (used as a scaling factor)
	double invTwoArea = 1.0 / (2.0 * this->triangle_area);

	// Differences in y-coordinates for B vector components
	double b1 = this->y2 - this->y3;
	double b2 = this->y3 - this->y1;
	double b3 = this->y1 - this->y2;

	// Differences in x-coordinates for C vector components
	double c1 = this->x3 - this->x2;
	double c2 = this->x1 - this->x3;
	double c3 = this->x2 - this->x1;

	// Fill first Jacobian matrix (2x3): derivatives of area co-ordinates L1, L2, L3
	// 
	//  | dL1/dx  dL2/dx  dL3/dx |
	//  | dL1/dy  dL2/dy  dL3/dy |
	//

	this->jacobianMatrix.resize(2, 3);
	this->jacobianMatrix.setZero();

	this->jacobianMatrix(0, 0) = b1 * invTwoArea;
	this->jacobianMatrix(0, 1) = b2 * invTwoArea;
	this->jacobianMatrix(0, 2) = b3 * invTwoArea;

	this->jacobianMatrix(1, 0) = c1 * invTwoArea;
	this->jacobianMatrix(1, 1) = c2 * invTwoArea;
	this->jacobianMatrix(1, 2) = c3 * invTwoArea;

	// Fill second matrix (3x6): products of derivatives of the area co-ordinates
	//
	// |    (dL1/dx)^2       (dL2/dx)^2       (dL3/dx)^2          2(dL1/dx dL2/dx)                 2(dL2/dx dL3/dx)                 2(dL2/dx dL3/dx)           |
	// |    (dL1/dy)^2       (dL2/dy)^2       (dL3/dy)^2          2(dL1/dy dL2/dy)                 2(dL2/dy dL3/dy)                 2(dL2/dy dL3/dy)           |
	// | (dL1/dx dL1/dy)  (dL2/dx dL2/dy)  (dL3/dx dL3/dy)  (dL1/dx dL2/dy)+(dL1/dy dL2/dx)  (dL2/dx dL3/dy)+(dL2/dy dL3/dx)  (dL3/dx dL1/dy)+(dL3/dy dL1/dx)  |
	//

	this->jacobianProducts.resize(3, 6);
	this->jacobianProducts.setZero();

	for (int j = 0; j < 3; ++j)
	{
		double bj = this->jacobianMatrix(0, j);
		double cj = this->jacobianMatrix(1, j);

		this->jacobianProducts(0, j) = bj * bj; // (dL1/dx)^2
		this->jacobianProducts(1, j) = cj * cj; // (dL1/dy)^2
		this->jacobianProducts(2, j) = bj * cj; // (dL1/dx dL1/dy)

		int m = j + 3;
		int next = (j + 1) % 3; // Wrap-around index for pairs

		double bjNext = jacobianMatrix(0, next);
		double cjNext = jacobianMatrix(1, next);

		this->jacobianProducts(0, m) = 2.0 * bj * bjNext; // 2(dL1/dx dL2/dx) 
		this->jacobianProducts(1, m) = 2.0 * cj * cjNext; // 2(dL1/dy dL2/dy) 
		this->jacobianProducts(2, m) = bj * cjNext + bjNext * cj; // (dL1/dx dL2/dy)+(dL1/dy dL2/dx)
	}

}




void triCKZ_element::computeShapeFunctions(const double& L1, const double& L2, const double& L3)
{
	// Differences in y-coordinates for B vector components
	double b1 = this->y2 - this->y3;
	double b2 = this->y3 - this->y1;
	double b3 = this->y1 - this->y2;

	// Differences in x-coordinates for C vector components
	double c1 = this->x3 - this->x2;
	double c2 = this->x1 - this->x3;
	double c3 = this->x2 - this->x1;


	this->shapeFunction.resize(9);
	shapeFunction.setZero();

	Eigen::MatrixXd shapeGradient(3, 9);
	shapeGradient.setZero(3, 9);

	this->shapefunction_secondDerivativeMatrix.resize(6, 9);
	this->shapefunction_secondDerivativeMatrix.setZero();

	//__________________________________________________________________________________________________________________
	// Shape functions AN0
	this->shapeFunction(0) = L1 + (L1 * L1 * L2) + (L1 * L1 * L3) - (L1 * L2 * L2) - (L1 * L3 * L3);
	this->shapeFunction(1) = -b3 * ((L1 * L1 * L2) + (0.5 * L1 * L2 * L3)) + b2 * ((L3 * L1 * L1) + (0.5 * L1 * L2 * L3));
	this->shapeFunction(2) = -c3 * ((L1 * L1 * L2) + (0.5 * L1 * L2 * L3)) + c2 * ((L3 * L1 * L1) + (0.5 * L1 * L2 * L3));
	this->shapeFunction(3) = L2 + (L2 * L2 * L3) + (L2 * L2 * L1) - (L2 * L3 * L3) - (L2 * L1 * L1);
	this->shapeFunction(4) = -b1 * ((L2 * L2 * L3) + (0.5 * L1 * L2 * L3)) + b3 * ((L1 * L2 * L2) + (0.5 * L1 * L2 * L3));
	this->shapeFunction(5) = -c1 * ((L2 * L2 * L3) + (0.5 * L1 * L2 * L3)) + c3 * ((L1 * L2 * L2) + (0.5 * L1 * L2 * L3));
	this->shapeFunction(6) = L3 + (L3 * L3 * L1) + (L3 * L3 * L2) - (L3 * L1 * L1) - (L3 * L2 * L2);
	this->shapeFunction(7) = -b2 * ((L3 * L3 * L1) + (0.5 * L1 * L2 * L3)) + b1 * ((L2 * L3 * L3) + (0.5 * L1 * L2 * L3));
	this->shapeFunction(8) = -c2 * ((L3 * L3 * L1) + (0.5 * L1 * L2 * L3)) + c1 * ((L2 * L3 * L3) + (0.5 * L1 * L2 * L3));


	//__________________________________________________________________________________________________________________
	// First derivative AN1 (3 x 9 Matrix)
	//      0       1   2     3      4   5     6      7   8 
	//  0| dN1/dL1  ()  ()  dN2/dL1  ()  ()  dN3/dL1  ()  () |
	//  1| dN1/dL2  ()  ()  dN2/dL2  ()  ()  dN3/dL2  ()  () |
	//  2| dN1/dL3  ()  ()  dN2/dL3  ()  ()  dN3/dL3  ()  () |
	//  
	// Derivatives with respect to L1, L2, L3
	shapeGradient(0, 0) = 1.0 + 2.0 * L1 * L2 + 2.0 * L1 * L3 - L2 * L2 - L3 * L3; // dN1/dL1
	shapeGradient(1, 3) = 1.0 + 2.0 * L2 * L3 + 2.0 * L2 * L1 - L3 * L3 - L1 * L1; // dN2/dL2
	shapeGradient(2, 6) = 1.0 + 2.0 * L3 * L1 + 2.0 * L3 * L2 - L1 * L1 - L2 * L2; // dN3/dL3

	shapeGradient(1, 0) = L1 * L1 - 2.0 * L1 * L2; // dN1/dL2
	shapeGradient(2, 3) = L2 * L2 - 2.0 * L2 * L3; // dN2/dL3
	shapeGradient(0, 6) = L3 * L3 - 2.0 * L3 * L1; // dN3/dL1

	shapeGradient(2, 0) = L1 * L1 - 2.0 * L1 * L3; // dN1/dL3
	shapeGradient(0, 3) = L2 * L2 - 2.0 * L2 * L1; // dN2/dL1
	shapeGradient(1, 6) = L3 * L3 - 2.0 * L3 * L2; // dN3/dL2

	// B terms
	shapeGradient(0, 1) = -b3 * (2.0 * L1 * L2 + 0.5 * L2 * L3) + b2 * (2.0 * L3 * L1 + 0.5 * L2 * L3); // dN'1/dL1
	shapeGradient(1, 4) = -b1 * (2.0 * L2 * L3 + 0.5 * L3 * L1) + b3 * (2.0 * L1 * L2 + 0.5 * L3 * L1); // dN'2/dL2
	shapeGradient(2, 7) = -b2 * (2.0 * L3 * L1 + 0.5 * L1 * L2) + b1 * (2.0 * L2 * L3 + 0.5 * L1 * L2); // dN'3/dL3

	shapeGradient(1, 1) = -b3 * (L1 * L1 + 0.5 * L1 * L3) + b2 * (0.5 * L1 * L3); // dN'1/dL2
	shapeGradient(2, 4) = -b1 * (L2 * L2 + 0.5 * L2 * L1) + b3 * (0.5 * L2 * L1); // dN'2/dL3
	shapeGradient(0, 7) = -b2 * (L3 * L3 + 0.5 * L3 * L2) + b1 * (0.5 * L3 * L2); // dN'3/dL1

	shapeGradient(2, 1) = -b3 * (0.5 * L1 * L2) + b2 * (L1 * L1 + 0.5 * L1 * L2); // dN'1/dL3
	shapeGradient(0, 4) = -b1 * (0.5 * L2 * L3) + b3 * (L2 * L2 + 0.5 * L2 * L3); // dN'2/dL1
	shapeGradient(1, 7) = -b2 * (0.5 * L3 * L1) + b1 * (L3 * L3 + 0.5 * L3 * L1); // dN'3/dL2

	// C terms
	shapeGradient(0, 2) = -c3 * (2.0 * L1 * L2 + 0.5 * L2 * L3) + c2 * (2.0 * L3 * L1 + 0.5 * L2 * L3); // dN''1/dL1
	shapeGradient(1, 5) = -c1 * (2.0 * L2 * L3 + 0.5 * L3 * L1) + c3 * (2.0 * L1 * L2 + 0.5 * L3 * L1); // dN''2/dL2
	shapeGradient(2, 8) = -c2 * (2.0 * L3 * L1 + 0.5 * L1 * L2) + c1 * (2.0 * L2 * L3 + 0.5 * L1 * L2); // dN''3/dL3

	shapeGradient(1, 2) = -c3 * (L1 * L1 + 0.5 * L1 * L3) + c2 * (0.5 * L1 * L3); // dN''1/dL2
	shapeGradient(2, 5) = -c1 * (L2 * L2 + 0.5 * L2 * L1) + c3 * (0.5 * L2 * L1); // dN''2/dL3
	shapeGradient(0, 8) = -c2 * (L3 * L3 + 0.5 * L3 * L2) + c1 * (0.5 * L3 * L2); // dN''3/dL1

	shapeGradient(2, 2) = -c3 * (0.5 * L1 * L2) + c2 * (L1 * L1 + 0.5 * L1 * L2); // dN''1/dL3
	shapeGradient(0, 5) = -c1 * (0.5 * L2 * L3) + c3 * (L2 * L2 + 0.5 * L2 * L3); // dN''2/dL1
	shapeGradient(1, 8) = -c2 * (0.5 * L3 * L1) + c1 * (L3 * L3 + 0.5 * L3 * L1); // dN''3/dL2


	//__________________________________________________________________________________________________________________
	// Second derivatives AN2  (6 x 9 Matrix)
	//      0              1   2     3            4   5        6          7   8 
	//  0| d^2N1/dL1^2    ()  ()  d^2N2/dL1^2    ()  ()    d^2N3/dL1^2   ()  () |
	//  1| d^2N1/dL2^2    ()  ()  d^2N2/dL2^2    ()  ()    d^2N3/dL2^2   ()  () |
	//  2| d^2N1/dL3^2    ()  ()  d^2N2/dL3^2    ()  ()    d^2N3/dL3^2   ()  () |
	//  3| d^2N1/dL1dL2   ()  ()  d^2N2/dL1dL2   ()  ()    d^2N3/dL1dL2  ()  () |
	//  4| d^2N1/dL2dL3   ()  ()  d^2N2/dL2dL3   ()  ()    d^2N3/dL2dL3  ()  () |
	//  5| d^2N1/dL3dL1   ()  ()  d^2N2/dL3dL1   ()  ()    d^2N3/dL3dL1  ()  () |
	// 
	// Second Derivatives with respect to L1, L2, L3


	this->shapefunction_secondDerivativeMatrix(0, 0) = 2.0 * (L2 + L3); // d^2N1/dL1^2
	this->shapefunction_secondDerivativeMatrix(1, 3) = 2.0 * (L3 + L1); // d^2N2/dL2^2
	this->shapefunction_secondDerivativeMatrix(2, 6) = 2.0 * (L1 + L2); // d^2N3/dL3^2
	this->shapefunction_secondDerivativeMatrix(1, 0) = -2.0 * L1; // d^2N1/dL2^2
	this->shapefunction_secondDerivativeMatrix(2, 3) = -2.0 * L2; // d^2N2/dL3^2
	this->shapefunction_secondDerivativeMatrix(0, 6) = -2.0 * L3; // d^2N3/dL1^2
	this->shapefunction_secondDerivativeMatrix(2, 0) = -2.0 * L1; // d^2N1/dL3^2 
	this->shapefunction_secondDerivativeMatrix(0, 3) = -2.0 * L2; // d^2N2/dL1^2 
	this->shapefunction_secondDerivativeMatrix(1, 6) = -2.0 * L3; // d^2N3/dL2^2

	this->shapefunction_secondDerivativeMatrix(0, 1) = (-2.0 * b3 * L2) + (2.0 * b2 * L3); // d^2N'1/dL1^2
	this->shapefunction_secondDerivativeMatrix(1, 4) = (-2.0 * b1 * L3) + (2.0 * b3 * L1); // d^2N'2/dL2^2
	this->shapefunction_secondDerivativeMatrix(2, 7) = (-2.0 * b2 * L1) + (2.0 * b1 * L2); // d^2N'3/dL3^2

	this->shapefunction_secondDerivativeMatrix(0, 2) = (-2.0 * c3 * L2) + (2.0 * c2 * L3); // d^2N''1/dL1^2
	this->shapefunction_secondDerivativeMatrix(1, 5) = (-2.0 * c1 * L3) + (2.0 * c3 * L1); // d^2N''2/dL2^2
	this->shapefunction_secondDerivativeMatrix(2, 8) = (-2.0 * c2 * L1) + (2.0 * c1 * L2); // d^2N''3/dL3^2

	// Additional terms... after 3rd row
	// N
	this->shapefunction_secondDerivativeMatrix(3, 0) = +2.0 * L1 - 2.0 * L2; // d^2N1/dL1dL2
	this->shapefunction_secondDerivativeMatrix(4, 3) = +2.0 * L2 - 2.0 * L3; // d^2N2/dL2dL3
	this->shapefunction_secondDerivativeMatrix(5, 6) = +2.0 * L3 - 2.0 * L1; // d^2N3/dL3dL1

	this->shapefunction_secondDerivativeMatrix(5, 0) = +2.0 * L1 - 2.0 * L3; // d^2N1/dL3dL1
	this->shapefunction_secondDerivativeMatrix(3, 3) = +2.0 * L2 - 2.0 * L1; // d^2N2/dL1dL2
	this->shapefunction_secondDerivativeMatrix(4, 6) = +2.0 * L3 - 2.0 * L2; // d^2N3/dL2dL3

	// N'
	this->shapefunction_secondDerivativeMatrix(3, 1) = -b3 * (2.0 * L1 + 0.5 * L3) + b2 * (0.5 * L3); // d^2N'1/dL1dL2
	this->shapefunction_secondDerivativeMatrix(4, 4) = -b1 * (2.0 * L2 + 0.5 * L1) + b3 * (0.5 * L1); // d^2N'2/dL2dL3
	this->shapefunction_secondDerivativeMatrix(5, 7) = -b2 * (2.0 * L3 + 0.5 * L2) + b1 * (0.5 * L2); // d^2N'3/dL3dL1

	this->shapefunction_secondDerivativeMatrix(4, 1) = -b3 * (L1 * 0.5) + b2 * (L1 * 0.5); // d^2N'1/dL2dL3
	this->shapefunction_secondDerivativeMatrix(5, 4) = -b1 * (L2 * 0.5) + b3 * (L2 * 0.5); // d^2N'2/dL3dL1
	this->shapefunction_secondDerivativeMatrix(3, 7) = -b2 * (L3 * 0.5) + b1 * (L3 * 0.5); // d^2N'3/dL1dL2

	this->shapefunction_secondDerivativeMatrix(5, 1) = -b3 * (L2 * 0.5) + b2 * (2.0 * L1 + L2 * 0.5); // d^2N'1/dL3dL1
	this->shapefunction_secondDerivativeMatrix(3, 4) = -b1 * (L3 * 0.5) + b3 * (2.0 * L2 + L3 * 0.5); // d^2N'2/dL1dL2
	this->shapefunction_secondDerivativeMatrix(4, 7) = -b2 * (L1 * 0.5) + b1 * (2.0 * L3 + L1 * 0.5); // d^2N'3/dL2dL3

	// N''
	this->shapefunction_secondDerivativeMatrix(3, 2) = -c3 * (2.0 * L1 + L3 * 0.5) + c2 * (L3 * 0.5); // d^2N''1/dL1dL2
	this->shapefunction_secondDerivativeMatrix(4, 5) = -c1 * (2.0 * L2 + L1 * 0.5) + c3 * (L1 * 0.5); // d^2N''2/dL2dL3
	this->shapefunction_secondDerivativeMatrix(5, 8) = -c2 * (2.0 * L3 + L2 * 0.5) + c1 * (L2 * 0.5); // d^2N''3/dL3dL1

	this->shapefunction_secondDerivativeMatrix(4, 2) = -c3 * (L1 * 0.5) + c2 * (L1 * 0.5); // d^2N''1/dL2dL3
	this->shapefunction_secondDerivativeMatrix(5, 5) = -c1 * (L2 * 0.5) + c3 * (L2 * 0.5); // d^2N''2/dL3dL1
	this->shapefunction_secondDerivativeMatrix(3, 8) = -c2 * (L3 * 0.5) + c1 * (L3 * 0.5); // d^2N''3/dL1dL2

	this->shapefunction_secondDerivativeMatrix(5, 2) = -c3 * (L2 * 0.5) + c2 * (2.0 * L1 + L2 * 0.5); // d^2N''1/dL3dL1
	this->shapefunction_secondDerivativeMatrix(3, 5) = -c1 * (L3 * 0.5) + c3 * (2.0 * L2 + L3 * 0.5); // d^2N''2/dL1dL2
	this->shapefunction_secondDerivativeMatrix(4, 8) = -c2 * (L1 * 0.5) + c1 * (2.0 * L3 + L1 * 0.5); // d^2N''3/dL2dL3


}



Eigen::MatrixXd triCKZ_element::computeStrainDisplacementMatrix()
{
	// Jacobian Products is a 3x6 matrix, Second derivative shape function is a 6x9 matrix
	Eigen::MatrixXd strainDisplacement_Bmatrix = this->jacobianProducts * this->shapefunction_secondDerivativeMatrix;  // Result is 3x9

	// Adjust signs 
	strainDisplacement_Bmatrix.row(0) = -1.0 * strainDisplacement_Bmatrix.row(0);
	strainDisplacement_Bmatrix.row(1) = -1.0 * strainDisplacement_Bmatrix.row(1);
	strainDisplacement_Bmatrix.row(2) = -2.0 * strainDisplacement_Bmatrix.row(2);

	return strainDisplacement_Bmatrix;  // B is 3x9 matrix

}






