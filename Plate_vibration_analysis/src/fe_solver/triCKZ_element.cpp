
#include "triCKZ_element.h"
#include "quadMITC4_element.h"

triCKZ_element::triCKZ_element()
{
	// Empty constructor
}

triCKZ_element::~triCKZ_element()
{
	// Empty destructor
}

void triCKZ_element::init()
{
	// Intialize the CKZ triangle element module

	// Step 0A: Set the Triangle Numerical Integration Point
	computeTriangleIntegrationPoints();

}


void triCKZ_element::computeTriangleIntegrationPoints()
{

	// Set the Triangle Area coordinates Integration Points
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


	// Set the bending stress integration points
	bendingstress_integration_points.setZero();

	// Row 1 coordinate 1
	bendingstress_integration_points.coeffRef(0, 0) = 1.0;
	bendingstress_integration_points.coeffRef(0, 1) = 0.0;
	bendingstress_integration_points.coeffRef(0, 2) = 0.0;

	// Row 2 coordinate 2
	bendingstress_integration_points.coeffRef(1, 0) = 0.0;
	bendingstress_integration_points.coeffRef(1, 1) = 1.0;
	bendingstress_integration_points.coeffRef(1, 2) = 0.0;

	// Row 3 coordinate 3
	bendingstress_integration_points.coeffRef(2, 0) = 0.0;
	bendingstress_integration_points.coeffRef(2, 1) = 0.0;
	bendingstress_integration_points.coeffRef(2, 2) = 1.0;

}


void triCKZ_element::set_triCKZ_element_stiffness_matrix(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
	const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
	const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
	const double& thickness, const double& materialdensity,
	const double& youngsmodulus, const double& poissonsratio)
{
	// Step 0: Compute the elasticity matrix
	computeElasticityMatrix(youngsmodulus, poissonsratio);

	// Step 1: Set the local co-ordinate system for the triangle
	Eigen::MatrixXd transformation_matrix_phi = Eigen::MatrixXd::Zero(18, 18); // 18x18 transformation matrices

	computeLocalCoordinateSystem(x1_g_coord, y1_g_coord, z1_g_coord,
		x2_g_coord, y2_g_coord, z2_g_coord, x3_g_coord, y3_g_coord, z3_g_coord,
		transformation_matrix_phi);


	// Membrane stiffness and stress matrix
	Eigen::MatrixXd element_MembraneStiffnessMatrix = Eigen::MatrixXd::Zero(18, 18); // 18 x 18 matrix Element membrane stiffness matrix

	computeMembraneStiffnessMatrix(thickness, element_MembraneStiffnessMatrix);


	// Bending stiffness and stress matrix
	Eigen::MatrixXd element_BendingStiffnessMatrix = Eigen::MatrixXd::Zero(18, 18); // 18 x 18 matrix Element bending stiffness matrix

	computeBendingStiffnessMatrix(thickness, element_BendingStiffnessMatrix);


	// Create the total stiffness (Membrane and Bending stiffness)
	Eigen::MatrixXd elem_StiffnessMatrix = Eigen::MatrixXd::Zero(18, 18);

	elem_StiffnessMatrix = element_MembraneStiffnessMatrix + element_BendingStiffnessMatrix;


	// Transform the element matrices to global coordinates
	this->element_StiffnessMatrix =
		transformation_matrix_phi.transpose() * elem_StiffnessMatrix * transformation_matrix_phi;

	// Compute the lumped mass matrix
	Eigen::MatrixXd elem_LumpedMassMatrix = Eigen::MatrixXd::Zero(18, 18);

	computeLumpedMassMatrix(thickness, materialdensity, elem_LumpedMassMatrix);

	this->element_LumpedMassMatrix = elem_LumpedMassMatrix;


	// Compute the consistent mass matrix


}


Eigen::MatrixXd triCKZ_element::get_element_stiffness_matrix()
{

	// Return the element stiffness matrix
	return this->element_StiffnessMatrix;
}


Eigen::MatrixXd triCKZ_element::get_element_mass_matrix()
{

	// Return the element mass matrix
	return this->element_LumpedMassMatrix;
}


void triCKZ_element::computeElasticityMatrix(const double& youngsmodulus, const double& poissonsratio)
{

	// compute the elasticity matrix
	this->elasticity_matrix.setZero();

	double k_const = youngsmodulus / (1.0 - (poissonsratio * poissonsratio));

	// Row 1
	this->elasticity_matrix.coeffRef(0, 0) = 1.0;
	this->elasticity_matrix.coeffRef(0, 1) = poissonsratio;
	this->elasticity_matrix.coeffRef(0, 2) = 0.0;

	// Row 2
	this->elasticity_matrix.coeffRef(1, 0) = poissonsratio;
	this->elasticity_matrix.coeffRef(1, 1) = 1.0;
	this->elasticity_matrix.coeffRef(1, 2) = 0.0;

	// Row 3
	this->elasticity_matrix.coeffRef(2, 0) = 0.0;
	this->elasticity_matrix.coeffRef(2, 1) = 0.0;
	this->elasticity_matrix.coeffRef(2, 2) = (1.0 - poissonsratio) * 0.5;


	this->elasticity_matrix = k_const * elasticity_matrix;

}



void triCKZ_element::computeLocalCoordinateSystem(const double& x1_g_coord, const double& y1_g_coord, const double& z1_g_coord,
	const double& x2_g_coord, const double& y2_g_coord, const double& z2_g_coord,
	const double& x3_g_coord, const double& y3_g_coord, const double& z3_g_coord,
	Eigen::MatrixXd& transformation_matrix_phi)
{

	Eigen::Vector3d p(x1_g_coord, y1_g_coord, z1_g_coord);  // Point P
	Eigen::Vector3d q(x2_g_coord, y2_g_coord, z2_g_coord);  // Point Q
	Eigen::Vector3d r(x3_g_coord, y3_g_coord, z3_g_coord);  // Point R


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

	this->triangle_area = 0.5 * this->x2 * this->y3;


	// 18x18 transformation matrices
	// It fills the 18 � 18 matrices PHI in 3�3 diagonal blocks with the local transformation matrix E0 (3�3)
	// So for blocks at positions [0:3, 0:3], [3:6, 3:6], ..., [15:18, 15:18], inserting E0

	// Fill 3x3 diagonal blocks with coordinateSystemE
	for (int i = 0; i < 18; i += 3)
	{
		transformation_matrix_phi.block<3, 3>(i, i) = coordinateSystemE.transpose();

	}


}


void triCKZ_element::computeMembraneStiffnessMatrix(const double& thickness, Eigen::MatrixXd& element_MembraneStiffnessMatrix)
{
	// Calculate element stiffness matrix and stress matrices for  
	// membrane deformations based on the free formulation.  

	// Reference: P.G. Bergan and C.A. Felippa,  
	// "A Triangular Membrane Element with Rotational Degrees of Freedom",  
	// CMAME, Vol. 50, pp. 25�69 (1985).

	double alpha = 1.50;
	double beta = 0.50;

	// Reciprocal of Sqrt(triangle area) 
	double invSqrtArea = 1.0 / std::sqrt(this->triangle_area);

	// Triangle vertex coordinates
	Eigen::Vector2d p1(this->x1, this->y1);
	Eigen::Vector2d p2(this->x2, this->y2);
	Eigen::Vector2d p3(this->x3, this->y3);


	// Helper structure for edge parameters
	struct EdgeParams
	{
		double a1, a2, a3;
		double b1, b2, b3;
		double cos_c, sin_s;
	};


	auto computeEdgeParams = [](const Eigen::Vector2d& pi, const Eigen::Vector2d& pj, const Eigen::Vector2d& pk) -> EdgeParams
		{
			Eigen::Vector2d mid = 0.5 * (pj + pk);
			Eigen::Vector2d vec = mid - pi;
			double xl = vec.norm();
			double cos_c = vec.x() / xl;
			double sin_s = vec.y() / xl;

			return
			{
				-sin_s * cos_c * cos_c * 0.5, // a1
				cos_c * cos_c * cos_c, // a2
				sin_s * ((0.5 * sin_s * sin_s) + (cos_c * cos_c)), // a3
				-cos_c * ((sin_s * sin_s) + (0.5 * cos_c * cos_c)), // b1
				-sin_s * sin_s * sin_s, // b2
				0.5 * sin_s * sin_s * cos_c, // b3
				cos_c, sin_s // cos & sin
			};
		};

	// Compute edge parameters
	EdgeParams edge1 = computeEdgeParams(p1, p2, p3);
	EdgeParams edge2 = computeEdgeParams(p2, p3, p1);
	EdgeParams edge3 = computeEdgeParams(p3, p1, p2);

	// Compute centroid
	Eigen::Vector2d centroid = (p1 + p2 + p3) / 3.0;

	// G matrix
	Eigen::MatrixXd g_matrix = Eigen::MatrixXd::Zero(9, 9);

	// Helper lambda to populate g_matrix rows for each node
	auto fillNodeContrib = [&](int baseRow, const Eigen::Vector2d& p, const EdgeParams& e1, const EdgeParams& e2, const EdgeParams& e3)
		{
			double xp = invSqrtArea * (p.x() - centroid.x());
			double yp = invSqrtArea * (p.y() - centroid.y());

			g_matrix.coeffRef(baseRow + 0, 0) = 1.0;
			g_matrix.coeffRef(baseRow + 1, 1) = 1.0;
			g_matrix.coeffRef(baseRow + 0, 2) = -yp;
			g_matrix.coeffRef(baseRow + 1, 2) = xp;
			g_matrix.coeffRef(baseRow + 2, 2) = invSqrtArea;
			g_matrix.coeffRef(baseRow + 0, 3) = xp;
			g_matrix.coeffRef(baseRow + 1, 4) = yp;
			g_matrix.coeffRef(baseRow + 0, 5) = yp;
			g_matrix.coeffRef(baseRow + 1, 5) = xp;

			auto fillEdgeTerms = [&](int col, const EdgeParams& edge)
				{
					double xpxp = xp * xp;
					double xpyp = xp * yp;
					double ypyp = yp * yp;

					g_matrix.coeffRef(baseRow + 0, col) = (edge.a1 * xpxp) + (edge.a2 * xpyp) + (edge.a3 * ypyp);
					g_matrix.coeffRef(baseRow + 1, col) = (edge.b1 * xpxp) + (edge.b2 * xpyp) + (edge.b3 * ypyp);
					g_matrix.coeffRef(baseRow + 2, col) = -invSqrtArea * ((edge.cos_c * xp) + (edge.sin_s * yp));
				};

			fillEdgeTerms(6, e1);
			fillEdgeTerms(7, e2);
			fillEdgeTerms(8, e3);
		};



	// Fill contributions for all 3 nodes
	fillNodeContrib(0, p1, edge1, edge2, edge3);
	fillNodeContrib(3, p2, edge1, edge2, edge3);
	fillNodeContrib(6, p3, edge1, edge2, edge3);


	// Inverse of G Matrix
	Eigen::MatrixXd g_matrix_inverse = g_matrix.inverse();


	//_____________________________________________________________________________________________
	// Formulate lumping matrix Q

	Eigen::MatrixXd q_matrix = Eigen::MatrixXd::Zero(9, 3);

	// Lambda to help fill q_matrix compactly
	auto fill_q_block = [&](int row_offset,
		double x1, double y1,
		double x2, double y2,
		double x3, double y3)
		{
			double yki = y2 - y3;
			double xik = x3 - x2;
			double yji = y1 - y3;
			double ykj = y2 - y1;
			double xij = x3 - x1;
			double xjk = x1 - x2;

			q_matrix.coeffRef(row_offset + 0, 0) = 0.5 * yki;
			q_matrix.coeffRef(row_offset + 0, 2) = 0.5 * xik;
			q_matrix.coeffRef(row_offset + 1, 1) = 0.5 * xik;
			q_matrix.coeffRef(row_offset + 1, 2) = 0.5 * yki;
			q_matrix.coeffRef(row_offset + 2, 0) = (alpha / 12.0) * ((yji * yji) - (ykj * ykj));
			q_matrix.coeffRef(row_offset + 2, 1) = (alpha / 12.0) * ((xij * xij) - (xjk * xjk));
			q_matrix.coeffRef(row_offset + 2, 2) = (alpha / 6.0) * ((xij * yji) - (xjk * ykj));
		};

	// First set (Node 1)
	fill_q_block(0, x1, y1, x2, y2, x3, y3);
	// Second set (Node 2)
	fill_q_block(3, x2, y2, x3, y3, x1, y1);
	// Third set (Node 3)
	fill_q_block(6, x3, y3, x1, y1, x2, y2);


	//_____________________________________________________________________________________________
	// Assemble basic membrane stiffness matrix

	std::vector<int> iadm = { 0, 1, 5, 6, 7, 11, 12, 13, 17 };  // DOF mapping 
	Eigen::MatrixXd basic_membrane_stiffness_matrix = Eigen::MatrixXd::Zero(18, 18);

	// Step 1: Compute SMM = DM * Q^T (manually, but could also use Eigen)
	Eigen::MatrixXd smm_matrix = this->elasticity_matrix * q_matrix.transpose();  // 3x9

	// Step 2: Assemble stiffness matrix (upper triangle only)
	for (int i = 0; i < 9; i++)
	{
		int ii = iadm[i];

		for (int j = i; j < 9; j++)
		{
			int jj = iadm[j];
			double contribution = q_matrix.row(i).dot(smm_matrix.col(j));  // Efficient dot product
			double value = contribution * (thickness / this->triangle_area);

			basic_membrane_stiffness_matrix.coeffRef(ii, jj) = basic_membrane_stiffness_matrix.coeff(ii, jj) + value;

			// Fill symmetric part
			if (ii != jj)
			{
				basic_membrane_stiffness_matrix.coeffRef(jj, ii) = basic_membrane_stiffness_matrix.coeff(jj, ii) + value;
			}

		}
	}


	//_____________________________________________________________________________________________
	// Calculate higher order stiffness matrix

	Eigen::Matrix3d eh_stiffness_matrix = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d b1_matrix = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d b2_matrix = Eigen::Matrix3d::Zero();

	// Step 1: Build B1 and B2 matrices
	b1_matrix.coeffRef(0, 0) = 2.0 * edge1.a1;
	b1_matrix.coeffRef(0, 1) = 2.0 * edge2.a1;
	b1_matrix.coeffRef(0, 2) = 2.0 * edge3.a1;
	b1_matrix.coeffRef(1, 0) = edge1.b2;
	b1_matrix.coeffRef(1, 1) = edge2.b2;
	b1_matrix.coeffRef(1, 2) = edge3.b2;
	b1_matrix.coeffRef(2, 0) = -4.0 * edge1.b3;
	b1_matrix.coeffRef(2, 1) = -4.0 * edge2.b3;
	b1_matrix.coeffRef(2, 2) = -4.0 * edge3.b3;

	b2_matrix.coeffRef(0, 0) = edge1.a2;
	b2_matrix.coeffRef(0, 1) = edge2.a2;
	b2_matrix.coeffRef(0, 2) = edge3.a2;
	b2_matrix.coeffRef(1, 0) = 2.0 * edge1.b3;
	b2_matrix.coeffRef(1, 1) = 2.0 * edge2.b3;
	b2_matrix.coeffRef(1, 2) = 2.0 * edge3.b3;
	b2_matrix.coeffRef(2, 0) = -4.0 * edge1.a1;
	b2_matrix.coeffRef(2, 1) = -4.0 * edge2.a1;
	b2_matrix.coeffRef(2, 2) = -4.0 * edge3.a1;


	Eigen::Vector2d xy_p1 = invSqrtArea * (p1 - centroid);
	Eigen::Vector2d xy_p2 = invSqrtArea * (p2 - centroid);
	Eigen::Vector2d xy_p3 = invSqrtArea * (p3 - centroid);

	// Step 2: Compute x_j, y_j, xy_j term
	double x_j = -((xy_p1.x() * xy_p2.x()) + (xy_p2.x() * xy_p3.x()) + (xy_p3.x() * xy_p1.x())) / 6.0;
	double y_j = -((xy_p1.y() * xy_p2.y()) + (xy_p2.y() * xy_p3.y()) + (xy_p3.y() * xy_p1.y())) / 6.0;
	double xy_j = ((xy_p1.x() * xy_p1.y()) + (xy_p2.x() * xy_p2.y()) + (xy_p3.x() * xy_p3.y())) / 12.0;

	// Step 3: Comput the higher order stiffness
	// EH += XJ * (B1^T * DM * B1)
	eh_stiffness_matrix = eh_stiffness_matrix + x_j * (b1_matrix.transpose() * this->elasticity_matrix * b1_matrix);

	// EH += YJ * (B2^T * DM * B2)
	eh_stiffness_matrix = eh_stiffness_matrix + y_j * (b2_matrix.transpose() * this->elasticity_matrix * b2_matrix);

	// EH += XYJ * (B1^T * DM * B2 + B2^T * DM * B1)
	Eigen::Matrix3d term1 = b1_matrix.transpose() * (this->elasticity_matrix * b2_matrix);
	Eigen::Matrix3d term2 = b2_matrix.transpose() * (this->elasticity_matrix * b1_matrix);

	eh_stiffness_matrix = eh_stiffness_matrix + xy_j * (term1 + term2);


	//Reinitialize the smm_matrix
	Eigen::MatrixXd higher_order_stiffness_matrix = Eigen::MatrixXd::Zero(18, 18);
	smm_matrix.setZero(); // 3 x 9

	// Step 1: Compute smm = EH * G_sub, where G_sub is G(6:8, :)
	for (int i = 0; i < 3; i++) // i = 0 to 2 for rows of EH
	{
		for (int j = 0; j < 9; j++) // j = 0 to 8
		{
			double sum = 0.0;
			for (int k = 0; k < 3; k++) // k = 0 to 2 (corresponding to EH col and G row)
			{
				sum = sum + eh_stiffness_matrix.coeff(i, k) * g_matrix_inverse.coeff(6 + k, j);
			}
			smm_matrix.coeffRef(i, j) = sum;
		}
	}

	// Step 2: Accumulate into higher_order_stiffness_matrix_18 using DOF mapping
	for (int i = 0; i < 9; i++)
	{
		int ii = iadm[i];
		for (int j = i; j < 9; j++) // j = i to 8 for upper triangle
		{
			int jj = iadm[j];
			double sum = 0.0;
			for (int k = 0; k < 3; k++)
			{
				sum = sum + g_matrix_inverse.coeff(6 + k, i) * smm_matrix.coeff(k, j);
			}

			higher_order_stiffness_matrix.coeffRef(ii, jj) = higher_order_stiffness_matrix.coeff(ii, jj) + (beta * sum * thickness);

			// Fill symmetric part
			if (ii != jj)
			{
				higher_order_stiffness_matrix.coeffRef(jj, ii) = 
					higher_order_stiffness_matrix.coeff(jj, ii) + higher_order_stiffness_matrix.coeff(ii, jj);
			}

		}
	}

	
	// Assign the total membrane stiffness matrix
	element_MembraneStiffnessMatrix = basic_membrane_stiffness_matrix + higher_order_stiffness_matrix;


/*
	//__________________________________________________________________________________________________________________________________

	this->element_MembraneStressMatrix.setZero();  // 9x9 final stress matrix

	double sqrtBeta = std::sqrt(beta);


	// Lambda to help fill stress matrix compactly
	auto computeStressContribution = [&](const Eigen::Vector2d& xy_p,        // X and Y coordinate of the point
		int smm_row_offset)                 // Offset for placing result in smm3 (0, 3, or 6)
		{
			// Step 1: Build BB (3x6)
			Eigen::MatrixXd bb = Eigen::MatrixXd::Zero(3, 6);
			bb.coeffRef(0, 0) = 1.0;
			bb.coeffRef(1, 1) = 1.0;
			bb.coeffRef(2, 2) = 2.0;

			// Update BB(0:2, 3:5)
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					bb.coeffRef(i, 3 + j) = ((b1_matrix.coeff(i, j) * xy_p.x()) + (b2_matrix.coeff(i, j) * xy_p.y())) * sqrtBeta;
				}
			}

			// Step 2: Compute BG = BB * G_lower (3x9)
			Eigen::MatrixXd bg = Eigen::MatrixXd::Zero(3, 9);
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 9; j++)
				{
					for (int k = 0; k < 6; k++)
					{
						bg.coeffRef(i, j) = bg.coeff(i, j) + (bb.coeff(i, k) * g_matrix_inverse.coeff(3 + k, j)); // G_inv(3+K, J)
					}
				}
			}

			// Step 3: Compute stress = DM * BG * XLAM
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 9; j++)
				{
					double zz = 0.0;
					for (int k = 0; k < 3; k++)
					{
						zz = zz + this->elasticity_matrix.coeff(i, k) * bg(k, j);
					}
					this->element_MembraneStressMatrix.coeffRef(smm_row_offset + i, j) =
						this->element_MembraneStressMatrix.coeff(smm_row_offset + i, j) + (zz * invSqrtArea);
				}
			}

		};


	computeStressContribution(xy_p1, 0);   // First corner
	computeStressContribution(xy_p2, 3);   // Second corner
	computeStressContribution(xy_p3, 6);   // Third corner
	*/

	}



void triCKZ_element::computeBendingStiffnessMatrix(const double& thickness, Eigen::MatrixXd& element_BendingStiffnessMatrix)
{
	// Step 1: Set the Jacobian Products
	// 3 x 6 matrix store the jacobian products J^2 ((dL/dx)^2, (dL/dy)^2, (dL/dx dL/dy), etc )
	Eigen::MatrixXd jacobianProducts = Eigen::MatrixXd::Zero(3, 6); 

	computeJacobianCoefficients(jacobianProducts);

	std::vector<int> iadb = { 2, 3, 4, 8, 9, 10, 14, 15, 16 };  // DOF mapping 

	// Parameters for calculating Consistent mass matrix
	double elem_volume = this->triangle_area * thickness;

	// Initialize the element stiffness matrix
	Eigen::MatrixXd elemBendingStiffnessMatrix = Eigen::MatrixXd::Zero(18,18);

	double t_eff = (std::pow(thickness, 3)) / 12.0;

	double L1 = 0.0;
	double L2 = 0.0;
	double L3 = 0.0;

	for (int ip = 0; ip < 4; ip++)
	{
		L1 = integration_points.coeff(ip, 0);
		L2 = integration_points.coeff(ip, 1);
		L3 = integration_points.coeff(ip, 2);
		double int_wt = integration_points.coeff(ip, 3); // weight

		double hxy = this->triangle_area * int_wt;
		double factor = t_eff * hxy;

		//___________________________________________________________________________________________________________________
		// Strain Displacement Matrix at Integration Points
		Eigen::MatrixXd  strainDisplacement_Bmatrix = computeStrainDisplacementMatrix(L1, L2, L3, jacobianProducts);  // B is 3x9 matrix

		// Compute E = B^T * D * B
		Eigen::MatrixXd bending_stiffness_matrix_IP =
			strainDisplacement_Bmatrix.transpose() * this->elasticity_matrix * strainDisplacement_Bmatrix;

		// Apply integration weight factor
		bending_stiffness_matrix_IP = factor * bending_stiffness_matrix_IP; // 9 x 9 matrix


		// Map to the element stiffness matrix 18 x 18
		for (int i = 0; i < 9; i++)
		{
			int ii = iadb[i];
			for (int j = 0; j < 9; j++)
			{
				int jj = iadb[j];

				elemBendingStiffnessMatrix.coeffRef(ii, jj) =
					elemBendingStiffnessMatrix.coeff(ii, jj) + bending_stiffness_matrix_IP(i, j);
			}
		}

	}

	// Assign the result
	element_BendingStiffnessMatrix = elemBendingStiffnessMatrix;



	// matrixToString(this->element_BendingStiffnessMatrix);

	/*
	// Initialize the element stress matrix
	this->element_BendingStressMatrix.setZero();


	for (int i = 0; i < 3; i++)
	{
		L1 = bendingstress_integration_points.coeff(i, 0);
		L2 = bendingstress_integration_points.coeff(i, 1);
		L3 = bendingstress_integration_points.coeff(i, 2);

		//___________________________________________________________________________________________________________________
		// Strain Displacement Matrix at Integration Points
		Eigen::MatrixXd  strainDisplacement_Bmatrix = computeStrainDisplacementMatrix(L1, L2, L3);  // B is 3x9 matrix

		int row_offset = i * 3;

		this->element_BendingStressMatrix.block<3, 9>(row_offset, 0) =
			this->elasticity_matrix * strainDisplacement_Bmatrix;

	}

	// matrixToString(this->element_BendingStressMatrix);
	*/

}



void triCKZ_element::computeLumpedMassMatrix(const double& thickness, const double& materialdensity,
	Eigen::MatrixXd& elem_LumpedMassMatrix)
{

	// Volument of the element
	double elem_volume = this->triangle_area * thickness;
	double elem_mass = materialdensity * elem_volume *  (1.0/ 3.0);


	// Node 1
	elem_LumpedMassMatrix.coeffRef(0, 0) = elem_mass;
	elem_LumpedMassMatrix.coeffRef(1, 1) = elem_mass;
	elem_LumpedMassMatrix.coeffRef(2, 2) = elem_mass;

	// Node 2
	elem_LumpedMassMatrix.coeffRef(6, 6) = elem_mass;
	elem_LumpedMassMatrix.coeffRef(7, 7) = elem_mass;
	elem_LumpedMassMatrix.coeffRef(8, 8) = elem_mass;

	// Node 3
	elem_LumpedMassMatrix.coeffRef(12, 12) = elem_mass;
	elem_LumpedMassMatrix.coeffRef(13, 13) = elem_mass;
	elem_LumpedMassMatrix.coeffRef(14, 14) = elem_mass;


}





void triCKZ_element::computeConsistentMassMatrix(const double& thickness, const double& materialdensity,
	Eigen::MatrixXd& elem_ConsistentMassMatrix)
{

	// Volument of the element
	double elem_volume = this->triangle_area * thickness;
	double elem_mass = materialdensity * elem_volume;


	std::vector<int> iadb = { 2, 3, 4, 8, 9, 10, 14, 15, 16 };  // DOF mapping 

	double L1 = 0.0;
	double L2 = 0.0;
	double L3 = 0.0;
	double int_wt = 0.0;

	for (int ip = 0; ip < 4; ip++)
	{
		L1 = integration_points.coeff(ip, 0);
		L2 = integration_points.coeff(ip, 1);
		L3 = integration_points.coeff(ip, 2);
		int_wt = integration_points.coeff(ip, 3); // weight


		Eigen::VectorXd shapeFunction = Eigen::VectorXd::Zero(9); // 9 x 1 stores the shape function N
		Eigen::MatrixXd shapefunction_secondDerivativeMatrix = Eigen::MatrixXd::Zero(6, 9); // 6 x 9 stores the d^2N second derivatives of the shape function

		// Compute the shape function
		computeShapeFunctions(L1, L2, L3, shapeFunction, shapefunction_secondDerivativeMatrix);


		// Calculate the consistent mass matrix
		// Loop over shape function indices to accumulate mass contributions
		for (int j = 0; j < 9; j++)
		{
			int jj = iadb[j];
			for (int i = 0; i <= j; i++)
			{
				int ii = iadb[i];

				double val = shapeFunction[i] * shapeFunction[j] * elem_mass * int_wt;
				elem_ConsistentMassMatrix.coeffRef(ii, jj) += val;

				// Since it's symmetric, mirror the value if i != j
				if (ii != jj)
				{
					elem_ConsistentMassMatrix.coeffRef(jj, ii) += val;
				}
					
			}
		}


	}

	// Consistent mass due to membrane component
	std::vector<int> iadm = { 0, 1, 5, 6, 7, 11, 12, 13, 17 };  // DOF mapping 

	for (int i = 0; i < 6; i++)
	{
		int ii = iadm[i];
		for (int j = i; j < 6; j++)
		{
			int jj = iadm[j];

			int ij_sum = i + j;
			if ((ij_sum % 2) == 0)  // I + J is even
				elem_ConsistentMassMatrix.coeffRef(ii, jj) = elem_mass;

			if (i == j)  // diagonal
				elem_ConsistentMassMatrix.coeffRef(ii, jj) = 2.0 * elem_mass;

			// Symmetrically assign (optional if your matrix is symmetric by design)
			if (ii != jj)
				elem_ConsistentMassMatrix.coeffRef(jj, ii) = elem_ConsistentMassMatrix(ii, jj);
		}
	}


}






void triCKZ_element::computeJacobianCoefficients(Eigen::MatrixXd& jacobianProducts)
{
	// Reciprocal of 2 � triangle area (used as a scaling factor)
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

	Eigen::MatrixXd jacobianMatrix = Eigen::MatrixXd::Zero(2, 3); // 2 x 3 matrix stores the jacobian J (dL/dx, dL/dy, etc)

	jacobianMatrix.setZero(); // 2 x 3

	jacobianMatrix(0, 0) = b1 * invTwoArea;
	jacobianMatrix(0, 1) = b2 * invTwoArea;
	jacobianMatrix(0, 2) = b3 * invTwoArea;

	jacobianMatrix(1, 0) = c1 * invTwoArea;
	jacobianMatrix(1, 1) = c2 * invTwoArea;
	jacobianMatrix(1, 2) = c3 * invTwoArea;

	// Fill second matrix (3x6): products of derivatives of the area co-ordinates
	//
	// |    (dL1/dx)^2       (dL2/dx)^2       (dL3/dx)^2          2(dL1/dx dL2/dx)                 2(dL2/dx dL3/dx)                 2(dL2/dx dL3/dx)           |
	// |    (dL1/dy)^2       (dL2/dy)^2       (dL3/dy)^2          2(dL1/dy dL2/dy)                 2(dL2/dy dL3/dy)                 2(dL2/dy dL3/dy)           |
	// | (dL1/dx dL1/dy)  (dL2/dx dL2/dy)  (dL3/dx dL3/dy)  (dL1/dx dL2/dy)+(dL1/dy dL2/dx)  (dL2/dx dL3/dy)+(dL2/dy dL3/dx)  (dL3/dx dL1/dy)+(dL3/dy dL1/dx)  |
	//


	jacobianProducts.setZero(); // 3 x 6

	for (int j = 0; j < 3; ++j)
	{
		double bj = jacobianMatrix(0, j);
		double cj = jacobianMatrix(1, j);

		jacobianProducts(0, j) = bj * bj; // (dL1/dx)^2
		jacobianProducts(1, j) = cj * cj; // (dL1/dy)^2
		jacobianProducts(2, j) = bj * cj; // (dL1/dx dL1/dy)

		int m = j + 3;
		int next = (j + 1) % 3; // Wrap-around index for pairs

		double bjNext = jacobianMatrix(0, next);
		double cjNext = jacobianMatrix(1, next);

		jacobianProducts(0, m) = 2.0 * bj * bjNext; // 2(dL1/dx dL2/dx) 
		jacobianProducts(1, m) = 2.0 * cj * cjNext; // 2(dL1/dy dL2/dy) 
		jacobianProducts(2, m) = bj * cjNext + bjNext * cj; // (dL1/dx dL2/dy)+(dL1/dy dL2/dx)
	}

}



void triCKZ_element::computeShapeFunctions(const double& L1, const double& L2, const double& L3,
	Eigen::VectorXd& shapeFunction, Eigen::MatrixXd& shapefunction_secondDerivativeMatrix)
{
	// Differences in y-coordinates for B vector components
	double b1 = this->y2 - this->y3;
	double b2 = this->y3 - this->y1;
	double b3 = this->y1 - this->y2;

	// Differences in x-coordinates for C vector components
	double c1 = this->x3 - this->x2;
	double c2 = this->x1 - this->x3;
	double c3 = this->x2 - this->x1;


	shapeFunction.setZero(); // 9 x 1

	Eigen::MatrixXd shapeGradient = Eigen::MatrixXd::Zero(3, 9); // 3 x 9

	shapefunction_secondDerivativeMatrix.setZero(); // 6 x 9

	//__________________________________________________________________________________________________________________
	// Shape functions AN0
	shapeFunction(0) = L1 + (L1 * L1 * L2) + (L1 * L1 * L3) - (L1 * L2 * L2) - (L1 * L3 * L3);
	shapeFunction(1) = -b3 * ((L1 * L1 * L2) + (0.5 * L1 * L2 * L3)) + b2 * ((L3 * L1 * L1) + (0.5 * L1 * L2 * L3));
	shapeFunction(2) = -c3 * ((L1 * L1 * L2) + (0.5 * L1 * L2 * L3)) + c2 * ((L3 * L1 * L1) + (0.5 * L1 * L2 * L3));
	shapeFunction(3) = L2 + (L2 * L2 * L3) + (L2 * L2 * L1) - (L2 * L3 * L3) - (L2 * L1 * L1);
	shapeFunction(4) = -b1 * ((L2 * L2 * L3) + (0.5 * L1 * L2 * L3)) + b3 * ((L1 * L2 * L2) + (0.5 * L1 * L2 * L3));
	shapeFunction(5) = -c1 * ((L2 * L2 * L3) + (0.5 * L1 * L2 * L3)) + c3 * ((L1 * L2 * L2) + (0.5 * L1 * L2 * L3));
	shapeFunction(6) = L3 + (L3 * L3 * L1) + (L3 * L3 * L2) - (L3 * L1 * L1) - (L3 * L2 * L2);
	shapeFunction(7) = -b2 * ((L3 * L3 * L1) + (0.5 * L1 * L2 * L3)) + b1 * ((L2 * L3 * L3) + (0.5 * L1 * L2 * L3));
	shapeFunction(8) = -c2 * ((L3 * L3 * L1) + (0.5 * L1 * L2 * L3)) + c1 * ((L2 * L3 * L3) + (0.5 * L1 * L2 * L3));


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


	shapefunction_secondDerivativeMatrix(0, 0) = 2.0 * (L2 + L3); // d^2N1/dL1^2
	shapefunction_secondDerivativeMatrix(1, 3) = 2.0 * (L3 + L1); // d^2N2/dL2^2
	shapefunction_secondDerivativeMatrix(2, 6) = 2.0 * (L1 + L2); // d^2N3/dL3^2
	shapefunction_secondDerivativeMatrix(1, 0) = -2.0 * L1; // d^2N1/dL2^2
	shapefunction_secondDerivativeMatrix(2, 3) = -2.0 * L2; // d^2N2/dL3^2
	shapefunction_secondDerivativeMatrix(0, 6) = -2.0 * L3; // d^2N3/dL1^2
	shapefunction_secondDerivativeMatrix(2, 0) = -2.0 * L1; // d^2N1/dL3^2 
	shapefunction_secondDerivativeMatrix(0, 3) = -2.0 * L2; // d^2N2/dL1^2 
	shapefunction_secondDerivativeMatrix(1, 6) = -2.0 * L3; // d^2N3/dL2^2

	shapefunction_secondDerivativeMatrix(0, 1) = (-2.0 * b3 * L2) + (2.0 * b2 * L3); // d^2N'1/dL1^2
	shapefunction_secondDerivativeMatrix(1, 4) = (-2.0 * b1 * L3) + (2.0 * b3 * L1); // d^2N'2/dL2^2
	shapefunction_secondDerivativeMatrix(2, 7) = (-2.0 * b2 * L1) + (2.0 * b1 * L2); // d^2N'3/dL3^2

	shapefunction_secondDerivativeMatrix(0, 2) = (-2.0 * c3 * L2) + (2.0 * c2 * L3); // d^2N''1/dL1^2
	shapefunction_secondDerivativeMatrix(1, 5) = (-2.0 * c1 * L3) + (2.0 * c3 * L1); // d^2N''2/dL2^2
	shapefunction_secondDerivativeMatrix(2, 8) = (-2.0 * c2 * L1) + (2.0 * c1 * L2); // d^2N''3/dL3^2

	// Additional terms... after 3rd row
	// N
	shapefunction_secondDerivativeMatrix(3, 0) = +2.0 * L1 - 2.0 * L2; // d^2N1/dL1dL2
	shapefunction_secondDerivativeMatrix(4, 3) = +2.0 * L2 - 2.0 * L3; // d^2N2/dL2dL3
	shapefunction_secondDerivativeMatrix(5, 6) = +2.0 * L3 - 2.0 * L1; // d^2N3/dL3dL1

	shapefunction_secondDerivativeMatrix(5, 0) = +2.0 * L1 - 2.0 * L3; // d^2N1/dL3dL1
	shapefunction_secondDerivativeMatrix(3, 3) = +2.0 * L2 - 2.0 * L1; // d^2N2/dL1dL2
	shapefunction_secondDerivativeMatrix(4, 6) = +2.0 * L3 - 2.0 * L2; // d^2N3/dL2dL3

	// N'
	shapefunction_secondDerivativeMatrix(3, 1) = -b3 * (2.0 * L1 + 0.5 * L3) + b2 * (0.5 * L3); // d^2N'1/dL1dL2
	shapefunction_secondDerivativeMatrix(4, 4) = -b1 * (2.0 * L2 + 0.5 * L1) + b3 * (0.5 * L1); // d^2N'2/dL2dL3
	shapefunction_secondDerivativeMatrix(5, 7) = -b2 * (2.0 * L3 + 0.5 * L2) + b1 * (0.5 * L2); // d^2N'3/dL3dL1

	shapefunction_secondDerivativeMatrix(4, 1) = -b3 * (L1 * 0.5) + b2 * (L1 * 0.5); // d^2N'1/dL2dL3
	shapefunction_secondDerivativeMatrix(5, 4) = -b1 * (L2 * 0.5) + b3 * (L2 * 0.5); // d^2N'2/dL3dL1
	shapefunction_secondDerivativeMatrix(3, 7) = -b2 * (L3 * 0.5) + b1 * (L3 * 0.5); // d^2N'3/dL1dL2

	shapefunction_secondDerivativeMatrix(5, 1) = -b3 * (L2 * 0.5) + b2 * (2.0 * L1 + L2 * 0.5); // d^2N'1/dL3dL1
	shapefunction_secondDerivativeMatrix(3, 4) = -b1 * (L3 * 0.5) + b3 * (2.0 * L2 + L3 * 0.5); // d^2N'2/dL1dL2
	shapefunction_secondDerivativeMatrix(4, 7) = -b2 * (L1 * 0.5) + b1 * (2.0 * L3 + L1 * 0.5); // d^2N'3/dL2dL3

	// N''
	shapefunction_secondDerivativeMatrix(3, 2) = -c3 * (2.0 * L1 + L3 * 0.5) + c2 * (L3 * 0.5); // d^2N''1/dL1dL2
	shapefunction_secondDerivativeMatrix(4, 5) = -c1 * (2.0 * L2 + L1 * 0.5) + c3 * (L1 * 0.5); // d^2N''2/dL2dL3
	shapefunction_secondDerivativeMatrix(5, 8) = -c2 * (2.0 * L3 + L2 * 0.5) + c1 * (L2 * 0.5); // d^2N''3/dL3dL1

	shapefunction_secondDerivativeMatrix(4, 2) = -c3 * (L1 * 0.5) + c2 * (L1 * 0.5); // d^2N''1/dL2dL3
	shapefunction_secondDerivativeMatrix(5, 5) = -c1 * (L2 * 0.5) + c3 * (L2 * 0.5); // d^2N''2/dL3dL1
	shapefunction_secondDerivativeMatrix(3, 8) = -c2 * (L3 * 0.5) + c1 * (L3 * 0.5); // d^2N''3/dL1dL2

	shapefunction_secondDerivativeMatrix(5, 2) = -c3 * (L2 * 0.5) + c2 * (2.0 * L1 + L2 * 0.5); // d^2N''1/dL3dL1
	shapefunction_secondDerivativeMatrix(3, 5) = -c1 * (L3 * 0.5) + c3 * (2.0 * L2 + L3 * 0.5); // d^2N''2/dL1dL2
	shapefunction_secondDerivativeMatrix(4, 8) = -c2 * (L1 * 0.5) + c1 * (2.0 * L3 + L1 * 0.5); // d^2N''3/dL2dL3


}



Eigen::MatrixXd triCKZ_element::computeStrainDisplacementMatrix(const double& L1, const double& L2, const double& L3,
	const Eigen::MatrixXd& jacobianProducts)
{


	Eigen::VectorXd shapeFunction = Eigen::VectorXd::Zero(9); // 9 x 1 stores the shape function N
	Eigen::MatrixXd shapefunction_secondDerivativeMatrix = Eigen::MatrixXd::Zero(6, 9); // 6 x 9 stores the d^2N second derivatives of the shape function

	// Compute the shape function
	computeShapeFunctions(L1, L2, L3, shapeFunction, shapefunction_secondDerivativeMatrix);

	// Jacobian Products is a 3x6 matrix, Second derivative shape function is a 6x9 matrix
	Eigen::MatrixXd strainDisplacement_Bmatrix = jacobianProducts * shapefunction_secondDerivativeMatrix;  // Result is 3x9

	// Adjust signs 
	strainDisplacement_Bmatrix.row(0) = -1.0 * strainDisplacement_Bmatrix.row(0);
	strainDisplacement_Bmatrix.row(1) = -1.0 * strainDisplacement_Bmatrix.row(1);
	strainDisplacement_Bmatrix.row(2) = -2.0 * strainDisplacement_Bmatrix.row(2);

	return strainDisplacement_Bmatrix;  // B is 3x9 matrix

}


void triCKZ_element::coupleStressMatrices()
{

	//Eigen::MatrixXd element_combinedStressMatrix = Eigen::MatrixXd::Zero(18, 18);
	//std::vector<int> iadm = { 0, 1, 5, 6, 7, 11, 12, 13, 17 };  // DOF mapping 

	//for (int j = 0; j < 9; ++j)
	//{
	//	int jj = iadm[j];  // mapped column index for membrane DOFs

	//	for (int i = 0; i < 6; ++i)
	//	{
	//		element_combinedStressMatrix.coeffRef(i, jj) = 0.0;
	//		element_combinedStressMatrix.coeffRef(i + 6, jj) = 0.0;
	//		element_combinedStressMatrix.coeffRef(i + 12, jj) = 0.0;
	//	}

	//	for (int i = 0; i < 3; ++i)
	//	{
	//		int ii = i + 3;

	//		// Fill membrane matrix into upper block
	//		element_combinedStressMatrix.coeffRef(i, jj) = this->element_BendingStressMatrix.coeff(i, j);
	//		element_combinedStressMatrix.coeffRef(ii, jj) = this->element_BendingStressMatrix.coeff(i, j);

	//		// Fill membrane stress matrix into middle and lower blocks
	//		element_combinedStressMatrix.coeffRef(i, jj) = this->element_MembraneStressMatrix.coeff(i, j);
	//		element_combinedStressMatrix.coeffRef(i + 6, jj) = this->element_MembraneStressMatrix.coeff(i + 3, j);
	//		element_combinedStressMatrix.coeffRef(i + 12, jj) = this->element_MembraneStressMatrix.coeff(i + 6, j);

	//		element_combinedStressMatrix.coeffRef(ii, jj) = this->element_MembraneStressMatrix.coeff(i, j);
	//		element_combinedStressMatrix.coeffRef(ii + 6, jj) = this->element_MembraneStressMatrix.coeff(i + 3, j);
	//		element_combinedStressMatrix.coeffRef(ii + 12, jj) = this->element_MembraneStressMatrix.coeff(i + 6, j);
	//	}
	//}


}



void triCKZ_element::matrixToString(const Eigen::MatrixXd& mat)
{
	std::ostringstream oss;
	oss.precision(4);
	oss << std::fixed;

	for (int i = 0; i < mat.rows(); ++i) {
		for (int j = 0; j < mat.cols(); ++j) {
			oss << mat(i, j);
			if (j < mat.cols() - 1) oss << ", ";
		}
		oss << "\n";
	}
	std::string result = oss.str();

}


void triCKZ_element::vectorToString(const Eigen::VectorXd& vec)
{
	std::ostringstream oss;
	oss.precision(4);
	oss << std::fixed;

	for (int i = 0; i < vec.size(); ++i)
	{
		oss << vec(i);
		oss << "\n";
	}
	std::string result = oss.str();

}







