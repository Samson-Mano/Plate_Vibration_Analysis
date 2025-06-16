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

	const double TOL1 = m_pi / 180.0;

	for (int i = 0; i < 4; ++i)
	{
		Eigen::Vector3d z_axis = this->p_matrix_global[i].col(2);

		double xuli = z_axis(0);
		double yuli = z_axis(1);
		double zuli = z_axis(2);

		double av3i = z_axis.norm();
		double av1i = std::sqrt(yuli * yuli + zuli * zuli);
		double av2i = std::sqrt(std::pow(yuli * yuli + zuli * zuli, 2) + xuli * xuli * (yuli * yuli + zuli * zuli));

		if (av1i / av3i >= TOL1)
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
	A = A - XL * ez;
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
	Eigen::Vector3d ref_vector;

	// Compute vector from node 1 to node 2 in local coordinates
	ref_vector = this->quad_local_coords[1] - this->quad_local_coords[0];


	// Project VR onto local z-axis (p_matrix_local[0].col(2) * ref_vector)
	double vrn = this->p_matrix_local[0].col(2).dot(ref_vector);

	ref_vector = ref_vector - (vrn * this->p_matrix_local[0].col(2));

	ref_vector.normalize();// Normalize


}




