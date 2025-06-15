#pragma once
#include <Eigen/Dense>


class triCKZ_element
{
public:
	triCKZ_element();
	~triCKZ_element();

	void init();

	void set_triCKZ_element_stiffness_matrix(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
		const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
		const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
		const double& thickness,
		const double& materialdensity,
		const double& youngsmodulus, 
		const double& poissonsratio);

	Eigen::MatrixXd get_element_stiffness_matrix();

	Eigen::MatrixXd get_element_mass_matrix();

private:
	double x1 = 0.0;
	double y1 = 0.0;
	double x2 = 0.0;
	double y2 = 0.0;
	double x3 = 0.0;
	double y3 = 0.0;

	double triangle_area = 0.0;

	
	Eigen::MatrixXd transformation_matrix_phi = Eigen::MatrixXd::Zero(18, 18); // 18x18 transformation matrices

	Eigen::MatrixXd elasticity_matrix = Eigen::MatrixXd::Zero(3, 3); // 3 x 3 matrix saves the elasticity matrix based on youngsmodulus and poissons ratio
	Eigen::MatrixXd integration_points = Eigen::MatrixXd::Zero(4, 4); // 4 x 4 Triangle stiffness integration points
	Eigen::MatrixXd bendingstress_integration_points = Eigen::MatrixXd::Zero(3, 3); // 3 x 3 Triangle bending stress integration points
	Eigen::MatrixXd jacobianMatrix = Eigen::MatrixXd::Zero(2, 3); // 2 x 3 matrix stores the jacobian J (dL/dx, dL/dy, etc)
	Eigen::MatrixXd jacobianProducts = Eigen::MatrixXd::Zero(3, 6); // 3 x 6 matrix store the jacobian products J^2 ((dL/dx)^2, (dL/dy)^2, (dL/dx dL/dy), etc )
	Eigen::VectorXd shapeFunction = Eigen::VectorXd::Zero(9); // 9 x 1 stores the shape function N
	Eigen::MatrixXd shapefunction_secondDerivativeMatrix = Eigen::MatrixXd::Zero(6, 9); // 6 x 9 stores the d^2N second derivatives of the shape function
	Eigen::MatrixXd strainDisplacementMatrix = Eigen::MatrixXd::Zero(3, 9); // 3 x 9 matrix stores the B strain Displacment matrix
	// Eigen::MatrixXd element_consistentmassMatrix = Eigen::MatrixXd::Zero(9, 9); // 9 x 9 matrix Element consistent mass matrix

	Eigen::MatrixXd element_MembraneStiffnessMatrix = Eigen::MatrixXd::Zero(18, 18); // 18 x 18 matrix Element membrane stiffness matrix
	Eigen::MatrixXd element_BendingStiffnessMatrix = Eigen::MatrixXd::Zero(18, 18); // 18 x 18 matrix Element bending stiffness matrix

	Eigen::MatrixXd element_StiffnessMatrix = Eigen::MatrixXd::Zero(18, 18); // 18 x 18 matrix Element combined bending and membrane stiffness matrix

	Eigen::MatrixXd element_LumpedMassMatrix = Eigen::MatrixXd::Zero(18, 18); // 18 x 18 matrix Element lumped mass matrix

	// Eigen::MatrixXd element_MembraneStressMatrix = Eigen::MatrixXd::Zero(9, 9); // 9 x 9 matrix Element membrane stress matrix
	// Eigen::MatrixXd element_BendingStressMatrix = Eigen::MatrixXd::Zero(9, 9); // 9 x 9 matrix Element bending stress matrix


	void computeElasticityMatrix(const double& youngsmodulus,
		const double& poissonsratio);


	void computeTriangleIntegrationPoints();



	void computeLocalCoordinateSystem(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
		const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
		const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord);


	void computeMembraneStiffnessMatrix(const double& thickness);

	void computeBendingStiffnessMatrix(const double& thickness);


	void computeLumpedMassMatrix(const double& thickness, const double& materialdensity);


	void computeJacobianCoefficients();
		

	void computeShapeFunctions(const double& L1, const double& L2, const double& L3);



	Eigen::MatrixXd computeStrainDisplacementMatrix(const double& L1, const double& L2, const double& L3);

	void coupleStressMatrices();


	void matrixToString(const Eigen::MatrixXd& mat);


	void vectorToString(const Eigen::VectorXd& vec);


};