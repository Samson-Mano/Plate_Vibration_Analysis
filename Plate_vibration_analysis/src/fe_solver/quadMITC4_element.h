#pragma once
#include <Eigen/Dense>



class quadMITC4_element
{
public:
	quadMITC4_element();
	~quadMITC4_element();

	void init();

	void set_quadMITC4_element_stiffness_matrix(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
		const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
		const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
		const double& x4_g_coord, const double& y4_g_coord, const double& z4_g_coord,
		const double& thickness,
		const double& materialdensity,
		const double& youngsmodulus,
		const double& poissonsratio);

	Eigen::MatrixXd get_element_stiffness_matrix();

	Eigen::MatrixXd get_element_mass_matrix();

private:
	const double m_pi = 3.1415926535897932384626433;

	std::array<Eigen::Matrix3d, 4> p_matrix_global;
	std::array<Eigen::Vector3d, 4> quad_local_coords;
	Eigen::Vector3d ref_vector; // Reference vector in element local coordinate system
	std::array<Eigen::Matrix3d, 4> p_matrix_local;


	// Eigen::MatrixXd transformation_matrix_phi = Eigen::MatrixXd::Zero(24, 24); // 24x24 transformation matrices

	Eigen::MatrixXd elasticity_matrix = Eigen::MatrixXd::Zero(5, 5); // 5 x 5 matrix saves the elasticity matrix based on youngsmodulus and poissons ratio
	Eigen::MatrixXd integration_points = Eigen::MatrixXd::Zero(2, 2); // 2 x 2 integration points
	Eigen::Vector4d nat_xi = Eigen::Vector4d(-1.0, +1.0, +1.0, -1.0); // Natural xi co-ordinate
	Eigen::Vector4d nat_yi = Eigen::Vector4d(-1.0, -1.0, +1.0, +1.0); // Natural yi co-ordinate
	Eigen::Vector4d nat_zi = Eigen::Vector4d(+1.0, +1.0, +1.0, +1.0); // Natural zi co-ordinate
	// Eigen::VectorXd shapeFunction = Eigen::VectorXd::Zero(4); // 4 x 1 stores the shape function N
	// Eigen::MatrixXd shapefunction_firstDerivativeMatrix = Eigen::MatrixXd::Zero(2, 4); // 2 x 4 stores the dN first derivatives of the shape function
	// Eigen::MatrixXd jacobianMatrix = Eigen::MatrixXd(3, 3); // 3 x 3 store the Jacobian Coefficients

	Eigen::MatrixXd element_StiffnessMatrix = Eigen::MatrixXd::Zero(24, 24); // 24 x 24 matrix Element combined bending and membrane stiffness matrix

	Eigen::MatrixXd element_LumpedMassMatrix = Eigen::MatrixXd::Zero(24, 24); // 24 x 24 matrix Element lumped mass matrix


	void setIntegrationPoints();

	void computeElasticityMatrix(const double& youngsmodulus,
		const double& poissonsratio); 



	void computeTriadPMatrix(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
		const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
		const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
		const double& x4_g_coord, const double& y4_g_coord, const double& z4_g_coord);


	void checkquadgeometry(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
		const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
		const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
		const double& x4_g_coord, const double& y4_g_coord, const double& z4_g_coord);



	void computeLocalCoordinateSystem(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
		const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
		const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
		const double& x4_g_coord, const double& y4_g_coord, const double& z4_g_coord);


	void computeBMatrixMainShapeFunction(const double& thickness, Eigen::MatrixXd& StrainDisplacementMatrixMainShapeFunction);


	Eigen::MatrixXd computeTransverseShearStrainMatrix(const double& thickness);


	void computeMainStrainDisplacementMatrix(const double& integration_ptx, const double& integration_pty, const double& integration_ptz,
		const double& thickness, const Eigen::MatrixXd& TransverseShearStrainMatrix, const Eigen::Matrix3d& initial_transformation_matrix,
		Eigen::MatrixXd& StrainDisplacementMatrix, Eigen::MatrixXd& transformation_matrix_phi);


	void computeBMatrixExtraShapeFunction(const double& thickness, Eigen::MatrixXd& bic_matrix);


	void computeExtraStrainDisplacementMatrix(const double& integration_ptx, const double& integration_pty, const double& integration_ptz,
		const double& thickness, const Eigen::Matrix3d& initial_transformation_matrix,
		Eigen::MatrixXd& StrainDisplacementMatrix, Eigen::MatrixXd& transformation_matrix_phi);


	void computeShapeFunctonNJacobianMatrix(const double& xp, const double& yp, const double& zp, const double& thickness,
		Eigen::Vector4d& shapeFunction, Eigen::MatrixXd& shapefunction_firstDerivativeMatrix, Eigen::Matrix3d& jacobianMatrix);


	Eigen::Matrix3d computeInitialTransformationMatrix(const Eigen::Matrix3d& jacobianMatrix);


	Eigen::Matrix3d computeTransformationMatrixFromReference(const Eigen::Matrix3d& jacobianMatrix,
		const Eigen::Vector3d& ref_vector);


	Eigen::MatrixXd computePhiMatrix(const Eigen::Matrix3d& inverse_jacobian,
		const Eigen::Matrix3d& transformation_matrix);


	void compute_B_matrix(
		int ID,
		const Eigen::Vector4d& shape_function_values,                // N(1..4)
		const Eigen::MatrixXd& shapefunction_firstDerivativeMatrix,  // 2x4
		const Eigen::Matrix3d& jacobianMatrix,                       // 3x3
		const std::array<Eigen::Matrix3d, 4>& p_matrix_local,        // P(:,:,i) 4 (3x3)
		const Eigen::MatrixXd& jacobianMatrixbb2,                    // 4x24 (bb2)
		const Eigen::Matrix3d& transformation_matrix,                // TIC
		const double& xp, const double& yp, const double& zp, const double& zp0,
		const double& thickness,
		Eigen::MatrixXd& B_matrix                                    // 6x28 (output)
	);



};


