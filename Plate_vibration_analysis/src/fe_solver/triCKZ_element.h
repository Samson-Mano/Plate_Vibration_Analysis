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


	Eigen::MatrixXd elasticity_matrix = Eigen::MatrixXd::Zero(3, 3); // 3 x 3 matrix saves the elasticity matrix based on youngsmodulus and poissons ratio
	Eigen::MatrixXd integration_points = Eigen::MatrixXd::Zero(4, 4); // 4 x 4 Triangle stiffness integration points
	Eigen::MatrixXd bendingstress_integration_points = Eigen::MatrixXd::Zero(3, 3); // 3 x 3 Triangle bending stress integration points
	


	Eigen::MatrixXd element_StiffnessMatrix = Eigen::MatrixXd::Zero(18, 18); // 18 x 18 matrix Element combined bending and membrane stiffness matrix
	Eigen::MatrixXd element_LumpedMassMatrix = Eigen::MatrixXd::Zero(18, 18); // 18 x 18 matrix Element lumped mass matrix
	// Eigen::MatrixXd element_ConsistentMassMatrix = Eigen::MatrixXd::Zero(18, 18); // 18 x 18 matrix Element consitent mass matrix


	void computeTriangleIntegrationPoints();


	void computeElasticityMatrix(const double& youngsmodulus,
		const double& poissonsratio);



	void computeLocalCoordinateSystem(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
		const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
		const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
		Eigen::MatrixXd& transformation_matrix_phi);


	void computeMembraneStiffnessMatrix(const double& thickness, Eigen::MatrixXd& element_MembraneStiffnessMatrix);


	void computeBendingStiffnessMatrix(const double& thickness, Eigen::MatrixXd& element_BendingStiffnessMatrix);


	void computeLumpedMassMatrix(const double& thickness, const double& materialdensity,
		Eigen::MatrixXd& elem_LumpedMassMatrix);


	void computeConsistentMassMatrix(const double& thickness, const double& materialdensity,
		Eigen::MatrixXd& elem_ConsistentMassMatrix);


	void computeJacobianCoefficients(Eigen::MatrixXd& jacobianProducts);
		

	void computeShapeFunctions(const double& L1, const double& L2, const double& L3, 
		Eigen::VectorXd& shapeFunction, Eigen::MatrixXd& shapefunction_secondDerivativeMatrix);


	Eigen::MatrixXd computeStrainDisplacementMatrix(const double& L1, const double& L2, const double& L3, 
		const Eigen::MatrixXd& jacobianProducts);


	void coupleStressMatrices();


	void matrixToString(const Eigen::MatrixXd& mat);


	void vectorToString(const Eigen::VectorXd& vec);


};