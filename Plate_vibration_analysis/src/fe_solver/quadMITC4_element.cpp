#include "quadMITC4_element.h"


quadMITC4_element::quadMITC4_element()
{
	// Empty constructor
}


quadMITC4_element::~quadMITC4_element()
{
	// Empty destructor
}


void quadMITC4_element::init()
{
	// Intialize the MITC4 quadrilateral element module

	setIntegrationPoints();


}


void quadMITC4_element::setIntegrationPoints()
{
	// Set the integration points
	integration_points.setZero();

	// Row 1 (integration point, weights)
	integration_points.coeffRef(0, 0) = -1.0 / std::sqrt(3.0);
	integration_points.coeffRef(0, 1) = 1.0;

	// Row 2 (integration point, weights)
	integration_points.coeffRef(1, 0) = +1.0 / std::sqrt(3.0);
	integration_points.coeffRef(1, 1) = 1.0;

}


void quadMITC4_element::set_quadMITC4_element_stiffness_matrix(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
	const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
	const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
	const double& x4_g_coord, const double& y4_g_coord, const double& z4_g_coord,
	const double& thickness, const double& materialdensity,
	const double& youngsmodulus, const double& poissonsratio)
{



}



Eigen::MatrixXd quadMITC4_element::get_element_stiffness_matrix()
{

	// Return the element stiffness matrix
	return this->element_StiffnessMatrix;
}


Eigen::MatrixXd quadMITC4_element::get_element_mass_matrix()
{

	// Return the element mass matrix
	return this->element_LumpedMassMatrix;
}





void quadMITC4_element::computeElasticityMatrix(const double& youngsmodulus, const double& poissonsratio)
{
	// compute the elasticity matrix
	elasticity_matrix.setZero();

	double k_const = youngsmodulus / (1.0 - (poissonsratio * poissonsratio));

	// Row 1
	elasticity_matrix.coeffRef(0, 0) = 1.0;
	elasticity_matrix.coeffRef(0, 1) = poissonsratio;
	elasticity_matrix.coeffRef(0, 2) = 0.0;
	elasticity_matrix.coeffRef(0, 3) = 0.0;
	elasticity_matrix.coeffRef(0, 4) = 0.0;

	// Row 2
	elasticity_matrix.coeffRef(1, 0) = poissonsratio;
	elasticity_matrix.coeffRef(1, 1) = 1.0;
	elasticity_matrix.coeffRef(1, 2) = 0.0;
	elasticity_matrix.coeffRef(1, 3) = 0.0;
	elasticity_matrix.coeffRef(1, 4) = 0.0;

	// Row 3
	elasticity_matrix.coeffRef(2, 0) = 0.0;
	elasticity_matrix.coeffRef(2, 1) = 0.0;
	elasticity_matrix.coeffRef(2, 2) = (1.0 - poissonsratio) * 0.5;
	elasticity_matrix.coeffRef(2, 3) = 0.0;
	elasticity_matrix.coeffRef(2, 4) = 0.0;

	// Row 4
	elasticity_matrix.coeffRef(3, 0) = 0.0;
	elasticity_matrix.coeffRef(3, 1) = 0.0;
	elasticity_matrix.coeffRef(3, 2) = 0.0;
	elasticity_matrix.coeffRef(3, 3) = (1.0 - poissonsratio) * (5.0 / 12.0);
	elasticity_matrix.coeffRef(3, 4) = 0.0;

	// Row 5
	elasticity_matrix.coeffRef(4, 0) = 0.0;
	elasticity_matrix.coeffRef(4, 1) = 0.0;
	elasticity_matrix.coeffRef(4, 2) = 0.0;
	elasticity_matrix.coeffRef(4, 3) = 0.0;
	elasticity_matrix.coeffRef(4, 4) = (1.0 - poissonsratio) * (5.0 / 12.0);


	elasticity_matrix = k_const * elasticity_matrix;

}



void quadMITC4_element::computeTriadPMatrix(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
	const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
	const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
	const double& x4_g_coord, const double& y4_g_coord, const double& z4_g_coord)
{
	// Differences between coordinates
	Eigen::Vector3d v21 = { x2_g_coord - x1_g_coord, y2_g_coord - y1_g_coord, z2_g_coord - z1_g_coord };
	Eigen::Vector3d v41 = { x4_g_coord - x1_g_coord, y4_g_coord - y1_g_coord, z4_g_coord - z1_g_coord };
	Eigen::Vector3d v32 = { x3_g_coord - x2_g_coord, y3_g_coord - y2_g_coord, z3_g_coord - z2_g_coord };
	Eigen::Vector3d v34 = { x3_g_coord - x4_g_coord, y3_g_coord - y4_g_coord, z3_g_coord - z4_g_coord };

	double al21 = v21.norm();
	double al41 = v41.norm();
	double al32 = v32.norm();
	double al34 = v34.norm();

	// Initialize the P Matrix
	this->p_matrix_global = { Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
		Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero() }; // p_matrix global


	// Compute the third column of P (unit direction vectors of cross products)
	this->p_matrix_global[0].col(2) = v21.cross(v41) / (al21 * al41);
	this->p_matrix_global[1].col(2) = v21.cross(v32) / (al21 * al32);
	this->p_matrix_global[2].col(2) = v34.cross(v32) / (al34 * al32);
	this->p_matrix_global[3].col(2) = v34.cross(v41) / (al34 * al41);

	const double tolerance_1 = m_pi / 180.0;

	for (int i = 0; i < 4; ++i)
	{
		Eigen::Vector3d z_axis = this->p_matrix_global[i].col(2);

		double xuli = z_axis(0);
		double yuli = z_axis(1);
		double zuli = z_axis(2);

		double av3i = z_axis.norm();
		double av1i = std::sqrt(yuli * yuli + zuli * zuli);
		double av2i = std::sqrt(std::pow(yuli * yuli + zuli * zuli, 2) + xuli * xuli * (yuli * yuli + zuli * zuli));

		if ((av1i / av3i) >= tolerance_1)
		{
			// Case 1
			this->p_matrix_global[i].col(0) = Eigen::Vector3d(0.0, -zuli / av1i, yuli / av1i);
			this->p_matrix_global[i].col(1) = Eigen::Vector3d(
				(av1i * av1i) / av2i,
				-xuli * yuli / av2i,
				-xuli * zuli / av2i
			);
		}
		else
		{
			// Case 2
			av1i = std::sqrt(xuli * xuli + zuli * zuli);
			av2i = std::sqrt(std::pow(xuli * xuli + zuli * zuli, 2) + yuli * yuli * (xuli * xuli + zuli * zuli));

			this->p_matrix_global[i].col(0) = Eigen::Vector3d(zuli / av1i, 0.0, -xuli / av1i);
			this->p_matrix_global[i].col(1) = Eigen::Vector3d(
				-yuli * xuli / av2i,
				(av1i * av1i) / av2i,
				-yuli * zuli / av2i
			);
		}

		// Normalize Z-axis again for consistency
		this->p_matrix_global[i].col(2) = z_axis.normalized();
	}

}



void quadMITC4_element::checkquadgeometry(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
	const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
	const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
	const double& x4_g_coord, const double& y4_g_coord, const double& z4_g_coord)
{
	// Check the quadrilateral element distortion

	// Check the aspect ratio of quadrilateral elements
	const double aspect_ratio_tolerance = 700;


	// Check the corner angles
	const double angle_minimum = 0.1;
	const double angle_maximum = 179.9;


}



void quadMITC4_element::computeLocalCoordinateSystem(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
	const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
	const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
	const double& x4_g_coord, const double& y4_g_coord, const double& z4_g_coord)
{

	Eigen::Vector3d p(x1_g_coord, y1_g_coord, z1_g_coord);  // Point P
	Eigen::Vector3d q(x2_g_coord, y2_g_coord, z2_g_coord);  // Point Q
	Eigen::Vector3d r(x3_g_coord, y3_g_coord, z3_g_coord);  // Point R
	Eigen::Vector3d s(x4_g_coord, y4_g_coord, z4_g_coord);  // Point S


	// Step 1: A = R - P, B = S - Q
	Eigen::Vector3d A = r - p;
	Eigen::Vector3d B = s - q;

	// Step 2: C = A x B (cross product), unit z axis = C / ||C||
	Eigen::Vector3d C = A.cross(B);
	double D = C.norm();
	Eigen::Vector3d ez = C / D;

	// Step 3: A = Q - P
	A = q - p;

	// Step 4: Project A onto ez, then remove projection
	double XL = A.dot(ez);
	A = A - (XL * ez);
	D = A.norm();

	// Unit x axis
	Eigen::Vector3d ex = A / D;

	// Step 5: Compute local y axis as C x A (still right-handed)
	B = C.cross(A);
	D = B.norm();

	// Unit y axis
	Eigen::Vector3d ey = B / D;

	// Step 6: Assemble transformation matrix E = [ex ey ez]
	Eigen::Matrix3d local_coordinate_matrix = Eigen::Matrix3d::Zero(); // 3 x 3 local co-ordinate matrix

	local_coordinate_matrix.col(0) = ex;
	local_coordinate_matrix.col(1) = ey;
	local_coordinate_matrix.col(2) = ez;


	std::array<Eigen::Vector3d, 4> quad_global_coords;
	quad_global_coords[0] = Eigen::Vector3d(x1_g_coord, y1_g_coord, z1_g_coord);
	quad_global_coords[1] = Eigen::Vector3d(x2_g_coord, y2_g_coord, z2_g_coord);
	quad_global_coords[2] = Eigen::Vector3d(x3_g_coord, y3_g_coord, z3_g_coord);
	quad_global_coords[3] = Eigen::Vector3d(x4_g_coord, y4_g_coord, z4_g_coord);


	this->quad_local_coords = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
		Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() }; // local coords


	// Step 1: Transform global coordinates to local
	for (int i = 0; i < 4; i++)
	{
		this->quad_local_coords[i] = local_coordinate_matrix.transpose() * quad_global_coords[i];

	}


	// Step 2: Transfrom node 1 to origin
	Eigen::Vector3d ref_origin = this->quad_local_coords[0];
	for (int i = 1; i < 4; i++)
	{
		this->quad_local_coords[i] -= ref_origin;

	}
	this->quad_local_coords[0].setZero();



	// Step 3: Transform P_matrix (global) to P_matrix (local)
	this->p_matrix_local = { Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
		Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero() }; // p_matrix local


	for (int i = 0; i < 4; i++)
	{
		this->p_matrix_local[i] = local_coordinate_matrix.transpose() * this->p_matrix_global[i];

	}


	// Step 4: Form reference vector in element local coordinate system
	this->ref_vector.setZero();

	// Compute vector from node 1 to node 2 in local coordinates
	this->ref_vector = this->quad_local_coords[1] - this->quad_local_coords[0];


	// Project VR onto local z-axis (p_matrix_local[0].col(2) * ref_vector)
	double vrn = this->p_matrix_local[0].col(2).dot(ref_vector);

	this->ref_vector = ref_vector - (vrn * this->p_matrix_local[0].col(2));

	this->ref_vector.normalize();// Normalize


}

void quadMITC4_element::computInlStrainDisplacementMatrix(const double& integration_ptx, const double& integration_pty, const double& integration_ptz,
	const int& integrationpt_sum, const double& thickness,
	Eigen::MatrixXd StrainDisplacementMatrix, Eigen::MatrixXd& transformation_matrix_phi)
{
	// Computes the strain displacement matrix (B) and
	// Transformation matric phi
	StrainDisplacementMatrix = Eigen::MatrixXd::Zero(6, 28);
	transformation_matrix_phi = Eigen::MatrixXd::Zero(5, 6);


	Eigen::Vector4d shapeFunction = Eigen::Vector4d::Zero(); // Shape function
	Eigen::MatrixXd shapefunction_firstDerivativeMatrix = Eigen::MatrixXd::Zero(2, 4); // Shape function First derivative
	Eigen::Matrix3d jacobianMatrix = Eigen::Matrix3d::Zero(); // Jacobian Matrix

	if (integrationpt_sum == 1)
	{
		// At zero point
		computeShapeFunctonNJacobianMatrix(0, 0, 0, thickness, 
			shapeFunction, shapefunction_firstDerivativeMatrix, jacobianMatrix);


		Eigen::Matrix3d initial_transformation_matrix = computeInitialTransformationMatrix(jacobianMatrix);

	}


	// Compute the shape function, derivative shape function and jacobian matrix at integration points
	computeShapeFunctonNJacobianMatrix(integration_ptx, integration_pty, integration_ptz, thickness,
		shapeFunction, shapefunction_firstDerivativeMatrix, jacobianMatrix);


	// Calculate the inverse of jacobian
	Eigen::Matrix3d invjacobianMatrix = jacobianMatrix.inverse();
	
	// Calculate the transformation matrix at the integration point
	Eigen::Matrix3d transformation_matrix = computeTransformationMatrixFromReference(jacobianMatrix, this->ref_vector);


	// Calculate the phi transformation matrix
	transformation_matrix_phi = computePhiMatrix(invjacobianMatrix, transformation_matrix);

	// Extra shape functions (always calculated)
	double GT11 = jacobianMatrix.row(0).dot(transformation_matrix.col(0));
	double GT12 = jacobianMatrix.row(0).dot(transformation_matrix.col(1));
	double GT21 = jacobianMatrix.row(1).dot(transformation_matrix.col(0));
	double GT22 = jacobianMatrix.row(1).dot(transformation_matrix.col(1));

	StrainDisplacementMatrix(0, 24) = -2.0 * integration_ptx * GT11;
	StrainDisplacementMatrix(0, 25) = -2.0 * integration_ptx * GT12;
	StrainDisplacementMatrix(1, 26) = -2.0 * integration_pty * GT21;
	StrainDisplacementMatrix(1, 27) = -2.0 * integration_pty * GT22;
	StrainDisplacementMatrix(3, 24) = -integration_ptx * GT21;
	StrainDisplacementMatrix(3, 25) = -integration_ptx * GT22;
	StrainDisplacementMatrix(3, 26) = -integration_pty * GT11;
	StrainDisplacementMatrix(3, 27) = -integration_pty * GT12;

}




void quadMITC4_element::computeShapeFunctonNJacobianMatrix(const double& xp, const double& yp, const double& zp, const double& thickness,
	Eigen::Vector4d& shapeFunction, Eigen::MatrixXd& shapefunction_firstDerivativeMatrix, Eigen::Matrix3d& jacobianMatrix)
{
	// Step 1: Shape functions and their derivatives
	shapeFunction = Eigen::VectorXd::Zero(4); // 4 x 1 stores the shape function N
	shapefunction_firstDerivativeMatrix = Eigen::MatrixXd::Zero(2, 4); // 2 x 4 stores the dN first derivatives of the shape function

	shapefunction_firstDerivativeMatrix.setZero();

	for (int i = 0; i < 4; ++i)
	{
		const double xi = nat_xi(i);
		const double yi = nat_yi(i);

		const double fxi = 1.0 + (xp * xi);
		const double fyi = 1.0 + (yp * yi);

		// Shape functions
		shapeFunction.coeffRef(i) = 0.25 * fxi * fyi;

		// First derivative shape function
		shapefunction_firstDerivativeMatrix.coeffRef(0, i) = 0.25 * fyi * xi; // dN(1, I)
		shapefunction_firstDerivativeMatrix.coeffRef(1, i) = 0.25 * fxi * yi; // dN(2, I)
	}


	// Step 2: Initialize Jacobian Coefficient Matrix to zero
	jacobianMatrix = Eigen::Matrix3d::Zero();

	// Step 3: Compute the Jacobian matrix
	for (int i = 0; i < 4; i++)
	{

		const Eigen::MatrixXd& p_local = p_matrix_local[i];

		// Eigen::VectorXd p_ul2 = 0.5 * thickness * p_local.col(2);


		double xul2 = 0.5 * thickness * p_local(0, 2); // P(1,3,i)
		double yul2 = 0.5 * thickness * p_local(1, 2); // P(2,3,i)
		double zul2 = 0.5 * thickness * p_local(2, 2); // P(3,3,i)

		const Eigen::Vector3d& coord = quad_local_coords[i];

		for (int j = 0; j < 2; j++)
		{
			jacobianMatrix.coeffRef(j, 0) +=
				shapefunction_firstDerivativeMatrix(j, i) * (coord.x() + (zp * xul2));
			jacobianMatrix.coeffRef(j, 1) +=
				shapefunction_firstDerivativeMatrix(j, i) * (coord.y() + (zp * yul2));
			jacobianMatrix.coeffRef(j, 2) +=
				shapefunction_firstDerivativeMatrix(j, i) * (coord.z() + (zp * zul2));
		}

		jacobianMatrix.coeffRef(2, 0) += shapeFunction(i) * xul2;
		jacobianMatrix.coeffRef(2, 1) += shapeFunction(i) * yul2;
		jacobianMatrix.coeffRef(2, 2) += shapeFunction(i) * zul2;
	}

}



Eigen::Matrix3d quadMITC4_element::computeInitialTransformationMatrix(const Eigen::Matrix3d& jacobianMatrix)
{
	// The transformation matrix is built from three orthonormal vectors :
	// -v1 : tangent vector(in - plane)
	// -v2 : second tangent vector(in - plane, orthogonal to v1)
	// -v3 : normal vector(out - of - plane)


	Eigen::Matrix3d initial_transformation_matrix = Eigen::Matrix3d::Zero();

	// Step 1: Compute V3 = cross product of row 0 and row 1 of JC
	Eigen::Vector3d v3(
		jacobianMatrix(0, 1) * jacobianMatrix(1, 2) - jacobianMatrix(0, 2) * jacobianMatrix(1, 1),
		jacobianMatrix(0, 2) * jacobianMatrix(1, 0) - jacobianMatrix(0, 0) * jacobianMatrix(1, 2),
		jacobianMatrix(0, 0) * jacobianMatrix(1, 1) - jacobianMatrix(0, 1) * jacobianMatrix(1, 0)
	);
	v3.normalize();

	// Third column of TIC
	initial_transformation_matrix.col(2) = v3;

	// Step 2: Compute V1 = cross product of row 1 of JC and V3
	Eigen::Vector3d v1(
		jacobianMatrix(1, 1) * v3.z() - jacobianMatrix(1, 2) * v3.y(),
		jacobianMatrix(1, 2) * v3.x() - jacobianMatrix(1, 0) * v3.z(),
		jacobianMatrix(1, 0) * v3.y() - jacobianMatrix(1, 1) * v3.x()
	);
	v1.normalize();

	// First column of Transformation matrix
	initial_transformation_matrix.col(0) = v1;

	// Step 3: Compute V2 = cross product of V3 and V1
	Eigen::Vector3d v2 = v3.cross(v1);
	v2.normalize();

	// Second column of Transformation matrix
	initial_transformation_matrix.col(1) = v2;


	return initial_transformation_matrix;


}



Eigen::Matrix3d quadMITC4_element::computeTransformationMatrixFromReference(
	const Eigen::Matrix3d& jacobianMatrix,
	const Eigen::Vector3d& ref_vector)
{

	Eigen::Matrix3d transformation_matrix = Eigen::Matrix3d::Zero();

	// Step 1: Compute v3 = normal vector (cross product of Jacobian row 0 and 1)
	Eigen::Vector3d v3(
		jacobianMatrix(0, 1) * jacobianMatrix(1, 2) - jacobianMatrix(0, 2) * jacobianMatrix(1, 1),
		jacobianMatrix(0, 2) * jacobianMatrix(1, 0) - jacobianMatrix(0, 0) * jacobianMatrix(1, 2),
		jacobianMatrix(0, 0) * jacobianMatrix(1, 1) - jacobianMatrix(0, 1) * jacobianMatrix(1, 0)
	);
	v3.normalize();
	transformation_matrix.col(2) = v3;  // Set third column of T

	// Step 2: Compute v2 = cross product of v3 and ref_vector
	Eigen::Vector3d v2(
		v3.y() * ref_vector.z() - v3.z() * ref_vector.y(),
		v3.z() * ref_vector.x() - v3.x() * ref_vector.z(),
		v3.x() * ref_vector.y() - v3.y() * ref_vector.x()
	);
	v2.normalize();
	transformation_matrix.col(1) = v2;  // Set second column of T

	// Step 3: Compute v1 = cross product of v2 and v3
	Eigen::Vector3d v1 = v2.cross(v3);
	v1.normalize();
	transformation_matrix.col(0) = v1;  // Set first column of T

	return transformation_matrix;

}



Eigen::MatrixXd quadMITC4_element::computePhiMatrix(const Eigen::Matrix3d& inverse_jacobian,
	const Eigen::Matrix3d& transformation_matrix)
{


	Eigen::MatrixXd phi_matrix = Eigen::MatrixXd::Zero(5,6);
	
	// Step 1: Compute XL, XM, XN components for each T column
	Eigen::Vector3d col1 = transformation_matrix.col(0);
	Eigen::Vector3d col2 = transformation_matrix.col(1);
	Eigen::Vector3d col3 = transformation_matrix.col(2);


	double xl1 = inverse_jacobian(0, 0) * col1.x() + inverse_jacobian(1, 0) * col1.y() + inverse_jacobian(2, 0) * col1.z();
	double xm1 = inverse_jacobian(0, 1) * col1.x() + inverse_jacobian(1, 1) * col1.y() + inverse_jacobian(2, 1) * col1.z();
	double xn1 = inverse_jacobian(0, 2) * col1.x() + inverse_jacobian(1, 2) * col1.y() + inverse_jacobian(2, 2) * col1.z();

	double xl2 = inverse_jacobian(0, 0) * col2.x() + inverse_jacobian(1, 0) * col2.y() + inverse_jacobian(2, 0) * col2.z();
	double xm2 = inverse_jacobian(0, 1) * col2.x() + inverse_jacobian(1, 1) * col2.y() + inverse_jacobian(2, 1) * col2.z();
	double xn2 = inverse_jacobian(0, 2) * col2.x() + inverse_jacobian(1, 2) * col2.y() + inverse_jacobian(2, 2) * col2.z();

	double xl3 = inverse_jacobian(0, 0) * col3.x() + inverse_jacobian(1, 0) * col3.y() + inverse_jacobian(2, 0) * col3.z();
	double xm3 = inverse_jacobian(0, 1) * col3.x() + inverse_jacobian(1, 1) * col3.y() + inverse_jacobian(2, 1) * col3.z();
	double xn3 = inverse_jacobian(0, 2) * col3.x() + inverse_jacobian(1, 2) * col3.y() + inverse_jacobian(2, 2) * col3.z();

	// Step 2: Fill phi matrix
	// Row 1
	phi_matrix.coeffRef(0, 0) = xl1 * xl1;
	phi_matrix.coeffRef(0, 1) = xm1 * xm1;
	phi_matrix.coeffRef(0, 2) = xn1 * xn1;
	phi_matrix.coeffRef(0, 3) = 2.0 * xl1 * xm1;
	phi_matrix.coeffRef(0, 4) = 2.0 * xm1 * xn1;
	phi_matrix.coeffRef(0, 5) = 2.0 * xn1 * xl1;

	// Row 2
	phi_matrix.coeffRef(1, 0) = xl2 * xl2;
	phi_matrix.coeffRef(1, 1) = xm2 * xm2;
	phi_matrix.coeffRef(1, 2) = xn2 * xn2;
	phi_matrix.coeffRef(1, 3) = 2.0 * xl2 * xm2;
	phi_matrix.coeffRef(1, 4) = 2.0 * xm2 * xn2;
	phi_matrix.coeffRef(1, 5) = 2.0 * xn2 * xl2;

	// Row 3
	phi_matrix.coeffRef(2, 0) = 2.0 * xl1 * xl2;
	phi_matrix.coeffRef(2, 1) = 2.0 * xm1 * xm2;
	phi_matrix.coeffRef(2, 2) = 2.0 * xn1 * xn2;
	phi_matrix.coeffRef(2, 3) = 2.0 * (xl1 * xm2 + xl2 * xm1);
	phi_matrix.coeffRef(2, 4) = 2.0 * (xm1 * xn2 + xm2 * xn1);
	phi_matrix.coeffRef(2, 5) = 2.0 * (xn1 * xl2 + xn2 * xl1);

	// Row 4
	phi_matrix.coeffRef(3, 0) = 2.0 * xl2 * xl3;
	phi_matrix.coeffRef(3, 1) = 2.0 * xm2 * xm3;
	phi_matrix.coeffRef(3, 2) = 2.0 * xn2 * xn3;
	phi_matrix.coeffRef(3, 3) = 2.0 * (xl2 * xm3 + xl3 * xm2);
	phi_matrix.coeffRef(3, 4) = 2.0 * (xm2 * xn3 + xm3 * xn2);
	phi_matrix.coeffRef(3, 5) = 2.0 * (xn2 * xl3 + xn3 * xl2);

	// Row 5
	phi_matrix.coeffRef(4, 0) = 2.0 * xl3 * xl1;
	phi_matrix.coeffRef(4, 1) = 2.0 * xm3 * xm1;
	phi_matrix.coeffRef(4, 2) = 2.0 * xn3 * xn1;
	phi_matrix.coeffRef(4, 3) = 2.0 * (xl3 * xm1 + xl1 * xm3);
	phi_matrix.coeffRef(4, 4) = 2.0 * (xm3 * xn1 + xm1 * xn3);
	phi_matrix.coeffRef(4, 5) = 2.0 * (xn3 * xl1 + xn1 * xl3);

	return phi_matrix;


}



void quadMITC4_element::compute_B_matrix(
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
)
{
	B_matrix = Eigen::MatrixXd::Zero(6, 28);

	if (ID == 1) 
	{
		// Only extra shape functions part
		double GT11 = (jacobianMatrix.row(0).dot(transformation_matrix.col(0)));
		double GT12 = (jacobianMatrix.row(0).dot(transformation_matrix.col(1)));
		double GT21 = (jacobianMatrix.row(1).dot(transformation_matrix.col(0)));
		double GT22 = (jacobianMatrix.row(1).dot(transformation_matrix.col(1)));

		B_matrix(0, 24) = -2.0 * xp * GT11;
		B_matrix(0, 25) = -2.0 * xp * GT12;
		B_matrix(1, 26) = -2.0 * yp * GT21;
		B_matrix(1, 27) = -2.0 * yp * GT22;
		B_matrix(3, 24) = -xp * GT21;
		B_matrix(3, 25) = -xp * GT22;
		B_matrix(3, 26) = -yp * GT11;
		B_matrix(3, 27) = -yp * GT12;
		return;
	}

	for (int i = 0; i < 4; i++) 
	{
		int ii = 6 * i;
		const Eigen::Matrix3d& P = p_matrix_local[i];

		double dn1 = shapefunction_firstDerivativeMatrix(0, i);
		double dn2 = shapefunction_firstDerivativeMatrix(1, i);
		double N = shape_function_values(i);

		// Row 1
		B_matrix(0, ii + 0) = dn1 * jacobianMatrix(0, 0);
		B_matrix(0, ii + 1) = dn1 * jacobianMatrix(0, 1);
		B_matrix(0, ii + 2) = dn1 * jacobianMatrix(0, 2);
		double GV22 = jacobianMatrix.row(0).dot(P.col(1));
		B_matrix(0, ii + 3) = -(zp - zp0) * dn1 * GV22 * thickness / 2.0;
		double GV21 = jacobianMatrix.row(0).dot(P.col(0));
		B_matrix(0, ii + 4) = (zp - zp0) * dn1 * GV21 * thickness / 2.0;

		// Row 2
		B_matrix(1, ii + 0) = dn2 * jacobianMatrix(1, 0);
		B_matrix(1, ii + 1) = dn2 * jacobianMatrix(1, 1);
		B_matrix(1, ii + 2) = dn2 * jacobianMatrix(1, 2);
		GV22 = jacobianMatrix.row(1).dot(P.col(1));
		B_matrix(1, ii + 3) = -(zp - zp0) * dn2 * GV22 * thickness / 2.0;
		GV21 = jacobianMatrix.row(1).dot(P.col(0));
		B_matrix(1, ii + 4) = (zp - zp0) * dn2 * GV21 * thickness / 2.0;

		// Row 3
		GV22 = jacobianMatrix.row(2).dot(P.col(1));
		GV21 = jacobianMatrix.row(2).dot(P.col(0));
		B_matrix(2, ii + 3) = -N * GV22 * thickness / 2.0;
		B_matrix(2, ii + 4) = N * GV21 * thickness / 2.0;

		// Row 4
		B_matrix(3, ii + 0) = 0.5 * (dn1 * jacobianMatrix(1, 0) + dn2 * jacobianMatrix(0, 0));
		B_matrix(3, ii + 1) = 0.5 * (dn1 * jacobianMatrix(1, 1) + dn2 * jacobianMatrix(0, 1));
		B_matrix(3, ii + 2) = 0.5 * (dn1 * jacobianMatrix(1, 2) + dn2 * jacobianMatrix(0, 2));
		double GV21_alt = jacobianMatrix.row(0).dot(P.col(1));
		double GV22_alt = jacobianMatrix.row(1).dot(P.col(1));
		B_matrix(3, ii + 3) = -(zp - zp0) * (dn2 * GV21_alt + dn1 * GV22_alt) * thickness / 4.0;

		double GV11 = jacobianMatrix.row(0).dot(P.col(0));
		double GV12 = jacobianMatrix.row(1).dot(P.col(0));
		B_matrix(3, ii + 4) = (zp - zp0) * (dn2 * GV11 + dn1 * GV12) * thickness / 4.0;
	}

	// Rows 5 and 6 (using jacobianMatrix2 / BB)
	for (int j = 0; j < 24; j++) 
	{
		B_matrix(4, j) = 0.5 * ((1.0 - xp) * jacobianMatrixbb2(0, j) + (1.0 + xp) * jacobianMatrixbb2(1, j));
		B_matrix(5, j) = 0.5 * ((1.0 - yp) * jacobianMatrixbb2(2, j) + (1.0 + yp) * jacobianMatrixbb2(3, j));
	}

	// Extra shape functions (always calculated)
	double GT11 = jacobianMatrix.row(0).dot(transformation_matrix.col(0));
	double GT12 = jacobianMatrix.row(0).dot(transformation_matrix.col(1));
	double GT21 = jacobianMatrix.row(1).dot(transformation_matrix.col(0));
	double GT22 = jacobianMatrix.row(1).dot(transformation_matrix.col(1));

	B_matrix(0, 24) = -2.0 * xp * GT11;
	B_matrix(0, 25) = -2.0 * xp * GT12;
	B_matrix(1, 26) = -2.0 * yp * GT21;
	B_matrix(1, 27) = -2.0 * yp * GT22;
	B_matrix(3, 24) = -xp * GT21;
	B_matrix(3, 25) = -xp * GT22;
	B_matrix(3, 26) = -yp * GT11;
	B_matrix(3, 27) = -yp * GT12;


}










