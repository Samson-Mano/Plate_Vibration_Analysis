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

	// Step 0: Compute the elasticity matrix
	computeElasticityMatrix(youngsmodulus, poissonsratio);


	// Step 1: Compute the P matrix of nodal triads
	std::array<Eigen::Matrix3d, 4> p_matrix_global;

	computeTriadPMatrix(x1_g_coord, y1_g_coord, z1_g_coord,
		x2_g_coord, y2_g_coord, z2_g_coord,
		x3_g_coord, y3_g_coord, z3_g_coord,
		x4_g_coord, y4_g_coord, z4_g_coord,
		p_matrix_global);



	// Step 2: Set the local co-ordinate system for the triangle
	std::array<Eigen::Matrix3d, 4> p_matrix_local;
	Eigen::Matrix3d local_coordinate_matrix = Eigen::Matrix3d::Zero(); // 3 x 3 local co-ordinate matrix
	Eigen::Vector3d ref_vector; // Reference vector in element local coordinate system

	computeLocalCoordinateSystem(x1_g_coord, y1_g_coord, z1_g_coord,
		x2_g_coord, y2_g_coord, z2_g_coord,
		x3_g_coord, y3_g_coord, z3_g_coord,
		x4_g_coord, y4_g_coord, z4_g_coord,
		p_matrix_global,
		p_matrix_local,
		local_coordinate_matrix,
		ref_vector);



	// Step 3: Compute B matrix for extra shape function (5 x 4 matrix) column index 25, 26, 27, 28
	Eigen::MatrixXd StrainDisplacementMatrixExtraShapeFunction = Eigen::MatrixXd::Zero(5, 4);

	computeBMatrixExtraShapeFunction(thickness, p_matrix_local,
		ref_vector, StrainDisplacementMatrixExtraShapeFunction);



	// Step 4: Compute the stiffness and element mass matrix
	Eigen::MatrixXd elemStiffnessMatrix = Eigen::MatrixXd::Zero(24, 24);
	Eigen::MatrixXd elemMassMatrix = Eigen::MatrixXd::Zero(24, 24);
	double total_element_mass = 0.0;

	computeStiffnessMatrix(thickness, materialdensity,
		p_matrix_local, local_coordinate_matrix, ref_vector, StrainDisplacementMatrixExtraShapeFunction,
		total_element_mass,	elemStiffnessMatrix, elemMassMatrix);



	// Step 5: Perform transformation to global co-ordinates
	// Transform rotation to element co-ordinate system

	transform_localrotation_to_globalrotation(elemStiffnessMatrix, p_matrix_local);
	transform_localrotation_to_globalrotation(elemMassMatrix, p_matrix_local);

	// Find the transpose of local coordinate matrix
	Eigen::Matrix3d local_coordinate_matrix_transpose = local_coordinate_matrix.transpose();

	// Transform stiffness to global system
	transform_stiffness_to_globalcoordinates(elemStiffnessMatrix, local_coordinate_matrix_transpose);
	transform_stiffness_to_globalcoordinates(elemMassMatrix, local_coordinate_matrix_transpose);

	// Diagonalize the mass matrix
	diagonalize_mass_matrix(elemMassMatrix, total_element_mass);


	// Add to the globalvariable
	this->element_StiffnessMatrix = elemStiffnessMatrix;
	this->element_LumpedMassMatrix = elemMassMatrix;

	// matrixToString(element_StiffnessMatrix);
	// matrixToString(element_LumpedMassMatrix);


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
	const double& x4_g_coord, const double& y4_g_coord, const double& z4_g_coord,
	std::array<Eigen::Matrix3d, 4>& p_matrix_global)
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
	p_matrix_global = { Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
		Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero() }; // p_matrix global


	// Compute the third column of P (unit direction vectors of cross products)
	p_matrix_global[0].col(2) = v21.cross(v41) / (al21 * al41);
	p_matrix_global[1].col(2) = v21.cross(v32) / (al21 * al32);
	p_matrix_global[2].col(2) = v34.cross(v32) / (al34 * al32);
	p_matrix_global[3].col(2) = v34.cross(v41) / (al34 * al41);

	const double tolerance_1 = m_pi / 180.0;

	for (int i = 0; i < 4; ++i)
	{
		Eigen::Vector3d z_axis = p_matrix_global[i].col(2);

		double xuli = z_axis(0);
		double yuli = z_axis(1);
		double zuli = z_axis(2);

		double av3i = z_axis.norm();
		double av1i = std::sqrt(yuli * yuli + zuli * zuli);
		double av2i = std::sqrt(std::pow(yuli * yuli + zuli * zuli, 2) + xuli * xuli * (yuli * yuli + zuli * zuli));

		if ((av1i / av3i) >= tolerance_1)
		{
			// Case 1
			p_matrix_global[i].col(0) = Eigen::Vector3d(0.0, -zuli / av1i, yuli / av1i);
			p_matrix_global[i].col(1) = Eigen::Vector3d(
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

			p_matrix_global[i].col(0) = Eigen::Vector3d(zuli / av1i, 0.0, -xuli / av1i);
			p_matrix_global[i].col(1) = Eigen::Vector3d(
				-yuli * xuli / av2i,
				(av1i * av1i) / av2i,
				-yuli * zuli / av2i
			);
		}

		// Normalize Z-axis again for consistency
		p_matrix_global[i].col(2) = z_axis.normalized();
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
	const double& x4_g_coord, const double& y4_g_coord, const double& z4_g_coord,
	const std::array<Eigen::Matrix3d, 4>& p_matrix_global,
	std::array<Eigen::Matrix3d, 4>& p_matrix_local,
	Eigen::Matrix3d& local_coordinate_matrix,
	Eigen::Vector3d& ref_vector)
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
	p_matrix_local = { Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
		Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero() }; // p_matrix local


	for (int i = 0; i < 4; i++)
	{
		p_matrix_local[i] = local_coordinate_matrix.transpose() * p_matrix_global[i];

	}


	// Step 4: Form reference vector in element local coordinate system
	ref_vector.setZero();

	// Compute vector from node 1 to node 2 in local coordinates
	ref_vector = this->quad_local_coords[1] - this->quad_local_coords[0];


	// Project VR onto local z-axis (p_matrix_local[0].col(2) * ref_vector)
	double vrn = p_matrix_local[0].col(2).dot(ref_vector);

	ref_vector = ref_vector - (vrn * p_matrix_local[0].col(2));

	ref_vector.normalize();// Normalize

}




void quadMITC4_element::computeStiffnessMatrix(const double& thickness, const double& materialdensity,
	const std::array<Eigen::Matrix3d, 4>& p_matrix_local,
	const Eigen::Matrix3d& local_coordinate_matrix,
	const Eigen::Vector3d& ref_vector,
	const Eigen::MatrixXd& StrainDisplacementMatrixExtraShapeFunction,
	double& total_element_mass,
	Eigen::MatrixXd& elemStiffnessMatrix,
	Eigen::MatrixXd& elemMassMatrix)
{

	double integration_ptx = 0.0;
	double integration_pty = 0.0;
	double integration_ptz = 0.0;

	// At zeroth point
	Eigen::Vector4d shapeFunction = Eigen::Vector4d::Zero(); // Shape function
	Eigen::MatrixXd shapefunction_firstDerivativeMatrix = Eigen::MatrixXd::Zero(2, 4); // Shape function First derivative
	Eigen::Matrix3d jacobianMatrix = Eigen::Matrix3d::Zero(); // Jacobian Matrix

	computeShapeFunctonNJacobianMatrix(0.0, 0.0, 0.0,
		thickness, p_matrix_local,
		shapeFunction, shapefunction_firstDerivativeMatrix, jacobianMatrix);


	Eigen::Matrix3d initial_transformation_matrix = computeInitialTransformationMatrix(jacobianMatrix);


	// Compute the Transverse shear strain matrix
	Eigen::MatrixXd TransverseShearStrainMatrix = Eigen::MatrixXd::Zero(24, 24);
		
	computeTransverseShearStrainMatrix(thickness, p_matrix_local, TransverseShearStrainMatrix);


	// Penalty parameter for drilling DOF
	double beta_p = (1.0e-3) * this->elasticity_matrix(2, 2);


	// Store the main stiffness matrix
	Eigen::MatrixXd StiffnessMatrix11 = Eigen::MatrixXd::Zero(24, 24);

	// Store the extra shape function's stiffness matrix
	Eigen::MatrixXd StiffnessMatrix12 = Eigen::MatrixXd::Zero(24, 4);
	Eigen::MatrixXd StiffnessMatrix22 = Eigen::MatrixXd::Zero(4, 4);

		
	total_element_mass = 0.0;

	for (int i = 0; i < 2; i++)
	{
		// integration point z
		integration_ptz = -1.0 * this->integration_points(i, 0);

		for (int j = 0; j < 2; j++)
		{
			// integration point y
			integration_pty = this->integration_points(j, 0);

			for (int k = 0; k < 2; k++)
			{
				// integration point x
				integration_ptx = (integration_pty < 0.0) ? +this->integration_points(k, 0)
					: -1.0 * this->integration_points(k, 0);

				//_____________________________________________________________________________________________________________
				// Step 1: Compute the shape function, derivative shape function and jacobian matrix at integration points
				// shapefunction_firstDerivativeMatrix: 2 x 4 (dN/de, dN/dn) for 4 shape functions
				Eigen::Vector4d shapeFunction = Eigen::Vector4d::Zero(); // Shape function
				Eigen::MatrixXd shapefunction_firstDerivativeMatrix = Eigen::MatrixXd::Zero(2, 4); // Shape function First derivative
				Eigen::Matrix3d jacobianMatrix = Eigen::Matrix3d::Zero(); // Jacobian Matrix
				Eigen::Matrix3d invjacobianMatrix = Eigen::Matrix3d::Zero(); // Inverse Jacobian Matrix
				Eigen::Matrix3d transformation_matrix = Eigen::Matrix3d::Zero(); // Transformation matrix
				double jacobian_determinant = 0.0;

				computeShapeFunctonNJacobianMatrix(integration_ptx, integration_pty, integration_ptz,
					thickness, p_matrix_local,
					shapeFunction, shapefunction_firstDerivativeMatrix, jacobianMatrix);

				// Calculate the jacobian determinant and  inverse of jacobian
				// invjacobianMatrix: 3 x 2 (dX/de, and dX/dn)
				invjacobianMatrix = jacobianMatrix.inverse();
				jacobian_determinant = jacobianMatrix.determinant();

				// Calculate the transformation matrix at the integration point
				transformation_matrix = computeTransformationMatrixFromReference(jacobianMatrix, ref_vector);


				//_____________________________________________________________________________________________________________
				// Step 2: Compute Strain Displacement Matrix(B) for numerical integration pont
				// having local coordinates integration_ptx, integration_pty and integration_ptz

				Eigen::MatrixXd StrainDisplacementMatrixAtIntegrationPt = Eigen::MatrixXd::Zero(6, 28);
				Eigen::MatrixXd transformation_matrix_phi = Eigen::MatrixXd::Zero(5, 6);


				computeMainStrainDisplacementMatrix(integration_ptx, integration_pty, integration_ptz,
					thickness, p_matrix_local, TransverseShearStrainMatrix, initial_transformation_matrix,
					shapeFunction, shapefunction_firstDerivativeMatrix,
					jacobianMatrix, invjacobianMatrix,
					transformation_matrix,
					StrainDisplacementMatrixAtIntegrationPt, transformation_matrix_phi);


				// Transform B to BL: BL = PHI * B
				Eigen::MatrixXd StrainDisplacementMatrixTransformed = Eigen::MatrixXd::Zero(5, 28);

				StrainDisplacementMatrixTransformed = transformation_matrix_phi * StrainDisplacementMatrixAtIntegrationPt;


				//_____________________________________________________________________________________________________________
				// Step 3: Modify the strain displacement matrix to add the extra shape function stiffness
				// Add extra shape function B matrix (5x4) to the last 4 columns (cols 24 to 27) of B matrix transformed (5x28)
				StrainDisplacementMatrixTransformed.block<5, 4>(0, 24) += StrainDisplacementMatrixExtraShapeFunction;


				//_____________________________________________________________________________________________________________
				// Step 4: Compute the Stress matrix (S) for integration point
				Eigen::MatrixXd StressMatrixAtIntegrationPt = Eigen::MatrixXd::Zero(5, 28);

				StressMatrixAtIntegrationPt = this->elasticity_matrix * StrainDisplacementMatrixTransformed;


				//_____________________________________________________________________________________________________________
				// Step 5: Compute the Stiffness for drilling degrees of freedom
				// Membrane stiffness formulation to include rotation about the normal to the plane of the element
				Eigen::VectorXd StrainDisplacementDrillingDOFMatrix = Eigen::VectorXd::Zero(24);

				computeStrainDisplacementMatrixDrillingDOF(integration_ptx, integration_pty, integration_ptz,
					thickness, p_matrix_local,
					shapeFunction, shapefunction_firstDerivativeMatrix,
					invjacobianMatrix, transformation_matrix,
					StrainDisplacementDrillingDOFMatrix);



				//_____________________________________________________________________________________________________________
				// Step 6: Compute the Main and Extra Stiffness matrices
				// Compute the main stiffness matrix
				StiffnessMatrix11 += (StrainDisplacementMatrixTransformed.block<5, 24>(0, 0).transpose() *
					StressMatrixAtIntegrationPt.block<5, 24>(0, 0)) * jacobian_determinant;


				// Compute the stiffnesss matrix for the extra shape function
				StiffnessMatrix12 += (StrainDisplacementMatrixTransformed.block<5, 24>(0, 0).transpose() *
					StressMatrixAtIntegrationPt.block<5, 4>(0, 24)) * jacobian_determinant;

				StiffnessMatrix22 += (StrainDisplacementMatrixTransformed.block<5, 4>(0, 24).transpose() *
					StressMatrixAtIntegrationPt.block<5, 4>(0, 24)) * jacobian_determinant;


				// Step 6A: Compute and add drilling DOF stiffness contribution to the stiffness matrix
				for (int p = 0; p < 24; p++)
				{
					for (int q = p; q < 24; q++)
					{
						double val = StrainDisplacementDrillingDOFMatrix(p) * StrainDisplacementDrillingDOFMatrix(q) * beta_p * jacobian_determinant;
						StiffnessMatrix11(p, q) += val;

						if (p != q)
						{
							StiffnessMatrix11(q, p) += val;  // Ensure symmetry
						}
					}
				}


				//_____________________________________________________________________________________________________________
				// Step 7: Compute the Consistent mass matrix
				// Compute the incremental mass
				double element_mass = jacobian_determinant * materialdensity;
				total_element_mass = total_element_mass + element_mass;

				elemMassMatrix = elemMassMatrix +
					computeElemMassMatrixAtIntegrationPoint(shapeFunction, p_matrix_local,
						thickness, integration_ptz, element_mass);


			}
		}
	}


	// Static condensation of stiffness matrices to form the final element stiffness matrix
	//________________________________________________________________________________________________________________
	// Invert the 4�4 submatrix corresponding to the extra shape functions
	Eigen::MatrixXd StiffnessMatrix22Inv = StiffnessMatrix22.inverse();


	// Multiply the inverse with the transpose of the coupling matrix (result: 4�24)
	Eigen::MatrixXd StiffnessMatrix22Inv12 = StiffnessMatrix22Inv * StiffnessMatrix12.transpose();  // (4�24)


	// Perform the condensation: K11 - K12 * inv(K22) * K12^T
	elemStiffnessMatrix = StiffnessMatrix11 - (StiffnessMatrix12 * StiffnessMatrix22Inv12);



}




void quadMITC4_element::computeTransverseShearStrainMatrix(const double& thickness,
	const std::array<Eigen::Matrix3d, 4>& p_matrix_local,
	Eigen::MatrixXd& TransverseShearStrainMatrix)
{

	// Formulate the transverse shear strain matrix at edge centers

	Eigen::Vector4d shapeFunction = Eigen::Vector4d::Zero(); // Shape function
	Eigen::MatrixXd shapefunction_firstDerivativeMatrix = Eigen::MatrixXd::Zero(2, 4); // Shape function First derivative
	Eigen::Matrix3d jacobianMatrix = Eigen::Matrix3d::Zero(); // Jacobian Matrix
	double gv22 = 0.0;
	double gv21 = 0.0;

	//________________________________________________________________________________________________
		// For point A (-1.0, 0.0, 0.0)
	computeShapeFunctonNJacobianMatrix(-1.0, 0.0, 0.0,
		thickness, p_matrix_local,
		shapeFunction, shapefunction_firstDerivativeMatrix, jacobianMatrix);

	// First Row (Column 0 to 2)
	TransverseShearStrainMatrix(0, 0) = -jacobianMatrix(2, 0) / 4.0;
	TransverseShearStrainMatrix(0, 1) = -jacobianMatrix(2, 1) / 4.0;
	TransverseShearStrainMatrix(0, 2) = -jacobianMatrix(2, 2) / 4.0;

	// gv22 (1st node)
	gv22 = jacobianMatrix(1, 0) * p_matrix_local[0](0, 1) +
		jacobianMatrix(1, 1) * p_matrix_local[0](1, 1) +
		jacobianMatrix(1, 2) * p_matrix_local[0](2, 1);
	TransverseShearStrainMatrix(0, 3) = -gv22 * thickness / 8.0;

	// gv21 (1st node)
	gv21 = jacobianMatrix(1, 0) * p_matrix_local[0](0, 0) +
		jacobianMatrix(1, 1) * p_matrix_local[0](1, 0) +
		jacobianMatrix(1, 2) * p_matrix_local[0](2, 0);
	TransverseShearStrainMatrix(0, 4) = gv21 * thickness / 8.0;

	// First Row (Coulmn 18 to 20)
	TransverseShearStrainMatrix(0, 18) = jacobianMatrix(2, 0) / 4.0;
	TransverseShearStrainMatrix(0, 19) = jacobianMatrix(2, 1) / 4.0;
	TransverseShearStrainMatrix(0, 20) = jacobianMatrix(2, 2) / 4.0;

	// gv22 (4th node)
	gv22 = jacobianMatrix(1, 0) * p_matrix_local[3](0, 1) +
		jacobianMatrix(1, 1) * p_matrix_local[3](1, 1) +
		jacobianMatrix(1, 2) * p_matrix_local[3](2, 1);
	TransverseShearStrainMatrix(0, 21) = -gv22 * thickness / 8.0;

	// gv21 (4th node)
	gv21 = jacobianMatrix(1, 0) * p_matrix_local[3](0, 0) +
		jacobianMatrix(1, 1) * p_matrix_local[3](1, 0) +
		jacobianMatrix(1, 2) * p_matrix_local[3](2, 0);
	TransverseShearStrainMatrix(0, 22) = gv21 * thickness / 8.0;


	//________________________________________________________________________________________________
	// For point B (1.0, 0.0, 0.0)
	computeShapeFunctonNJacobianMatrix(-1.0, 0.0, 0.0,
		thickness, p_matrix_local,
		shapeFunction, shapefunction_firstDerivativeMatrix, jacobianMatrix);

	// Second Row (Column 6 to 8)
	TransverseShearStrainMatrix(1, 6) = -jacobianMatrix(2, 0) / 4.0;
	TransverseShearStrainMatrix(1, 7) = -jacobianMatrix(2, 1) / 4.0;
	TransverseShearStrainMatrix(1, 8) = -jacobianMatrix(2, 2) / 4.0;

	// gv22 for node 2
	gv22 = jacobianMatrix(1, 0) * p_matrix_local[1](0, 1) +
		jacobianMatrix(1, 1) * p_matrix_local[1](1, 1) +
		jacobianMatrix(1, 2) * p_matrix_local[1](2, 1);
	TransverseShearStrainMatrix(1, 9) = -gv22 * thickness / 8.0;

	// gv21 for node 2
	gv21 = jacobianMatrix(1, 0) * p_matrix_local[1](0, 0) +
		jacobianMatrix(1, 1) * p_matrix_local[1](1, 0) +
		jacobianMatrix(1, 2) * p_matrix_local[1](2, 0);
	TransverseShearStrainMatrix(1, 10) = gv21 * thickness / 8.0;

	// Second Row (Column 12 to 14)
	TransverseShearStrainMatrix(1, 12) = jacobianMatrix(2, 0) / 4.0;
	TransverseShearStrainMatrix(1, 13) = jacobianMatrix(2, 1) / 4.0;
	TransverseShearStrainMatrix(1, 14) = jacobianMatrix(2, 2) / 4.0;

	// gv22 for node 3
	gv22 = jacobianMatrix(1, 0) * p_matrix_local[2](0, 1) +
		jacobianMatrix(1, 1) * p_matrix_local[2](1, 1) +
		jacobianMatrix(1, 2) * p_matrix_local[2](2, 1);
	TransverseShearStrainMatrix(1, 15) = -gv22 * thickness / 8.0;

	// gv21 for node 3
	gv21 = jacobianMatrix(1, 0) * p_matrix_local[2](0, 0) +
		jacobianMatrix(1, 1) * p_matrix_local[2](1, 0) +
		jacobianMatrix(1, 2) * p_matrix_local[2](2, 0);
	TransverseShearStrainMatrix(1, 16) = gv21 * thickness / 8.0;


	//________________________________________________________________________________________________
	// For point C (0.0, -1.0, 0.0)
	computeShapeFunctonNJacobianMatrix(0.0, -1.0, 0.0,
		thickness, p_matrix_local,
		shapeFunction, shapefunction_firstDerivativeMatrix, jacobianMatrix);


	// Third Row (Column 0 to 2)
	TransverseShearStrainMatrix(2, 0) = -jacobianMatrix(2, 0) / 4.0;
	TransverseShearStrainMatrix(2, 1) = -jacobianMatrix(2, 1) / 4.0;
	TransverseShearStrainMatrix(2, 2) = -jacobianMatrix(2, 2) / 4.0;

	// gv22 for node 1 (x-direction)
	gv22 = jacobianMatrix(0, 0) * p_matrix_local[0](0, 1) +
		jacobianMatrix(0, 1) * p_matrix_local[0](1, 1) +
		jacobianMatrix(0, 2) * p_matrix_local[0](2, 1);
	TransverseShearStrainMatrix(2, 3) = -gv22 * thickness / 8.0;

	// gv21 for node 1
	gv21 = jacobianMatrix(0, 0) * p_matrix_local[0](0, 0) +
		jacobianMatrix(0, 1) * p_matrix_local[0](1, 0) +
		jacobianMatrix(0, 2) * p_matrix_local[0](2, 0);
	TransverseShearStrainMatrix(2, 4) = gv21 * thickness / 8.0;

	// Third Row (Column 6 to 8)
	TransverseShearStrainMatrix(2, 6) = jacobianMatrix(2, 0) / 4.0;
	TransverseShearStrainMatrix(2, 7) = jacobianMatrix(2, 1) / 4.0;
	TransverseShearStrainMatrix(2, 8) = jacobianMatrix(2, 2) / 4.0;

	// gv22 for node 2 (x-direction)
	gv22 = jacobianMatrix(0, 0) * p_matrix_local[1](0, 1) +
		jacobianMatrix(0, 1) * p_matrix_local[1](1, 1) +
		jacobianMatrix(0, 2) * p_matrix_local[1](2, 1);
	TransverseShearStrainMatrix(2, 9) = -gv22 * thickness / 8.0;

	// gv21 for node 2
	gv21 = jacobianMatrix(0, 0) * p_matrix_local[1](0, 0) +
		jacobianMatrix(0, 1) * p_matrix_local[1](1, 0) +
		jacobianMatrix(0, 2) * p_matrix_local[1](2, 0);
	TransverseShearStrainMatrix(2, 10) = gv21 * thickness / 8.0;


	//________________________________________________________________________________________________
	// For point D (0.0, 1.0, 0.0)
	computeShapeFunctonNJacobianMatrix(0.0, 1.0, 0.0,
		thickness, p_matrix_local,
		shapeFunction, shapefunction_firstDerivativeMatrix, jacobianMatrix);

	// Fourth Row (Column 12 to 14)
	TransverseShearStrainMatrix(3, 12) = jacobianMatrix(2, 0) / 4.0;
	TransverseShearStrainMatrix(3, 13) = jacobianMatrix(2, 1) / 4.0;
	TransverseShearStrainMatrix(3, 14) = jacobianMatrix(2, 2) / 4.0;

	// gv22 for node 3 (x-direction)
	gv22 = jacobianMatrix(0, 0) * p_matrix_local[2](0, 1) +
		jacobianMatrix(0, 1) * p_matrix_local[2](1, 1) +
		jacobianMatrix(0, 2) * p_matrix_local[2](2, 1);
	TransverseShearStrainMatrix(3, 15) = -gv22 * thickness / 8.0;

	// gv21 for node 3
	gv21 = jacobianMatrix(0, 0) * p_matrix_local[2](0, 0) +
		jacobianMatrix(0, 1) * p_matrix_local[2](1, 0) +
		jacobianMatrix(0, 2) * p_matrix_local[2](2, 0);
	TransverseShearStrainMatrix(3, 16) = gv21 * thickness / 8.0;

	// Fourth Row (Column 18 to 20)
	TransverseShearStrainMatrix(3, 18) = -jacobianMatrix(2, 0) / 4.0;
	TransverseShearStrainMatrix(3, 19) = -jacobianMatrix(2, 1) / 4.0;
	TransverseShearStrainMatrix(3, 20) = -jacobianMatrix(2, 2) / 4.0;

	// gv22 for node 4
	gv22 = jacobianMatrix(0, 0) * p_matrix_local[3](0, 1) +
		jacobianMatrix(0, 1) * p_matrix_local[3](1, 1) +
		jacobianMatrix(0, 2) * p_matrix_local[3](2, 1);
	TransverseShearStrainMatrix(3, 21) = -gv22 * thickness / 8.0;

	// gv21 for node 4
	gv21 = jacobianMatrix(0, 0) * p_matrix_local[3](0, 0) +
		jacobianMatrix(0, 1) * p_matrix_local[3](1, 0) +
		jacobianMatrix(0, 2) * p_matrix_local[3](2, 0);
	TransverseShearStrainMatrix(3, 22) = gv21 * thickness / 8.0;

}



void quadMITC4_element::computeStrainDisplacementMatrixDrillingDOF(const double& integration_ptx, const double& integration_pty, const double& integration_ptz,
	const double& thickness, const std::array<Eigen::Matrix3d, 4>& p_matrix_local,
	const Eigen::Vector4d& shapeFunction, const Eigen::MatrixXd& shapefunction_firstDerivativeMatrix,
	const Eigen::Matrix3d& invjacobianMatrix, const Eigen::Matrix3d& transformation_matrix,
	Eigen::VectorXd& StrainDisplacementDrillingDOFMatrix)
{
	// Computes the strain displacement matrix (B) of drilling degree of freedom 
	// Membrane stiffness formulation to include rotation about the normal to the plane of the element

	Eigen::MatrixXd shapefunction_GlobalfirstDerivativeMatrix = Eigen::MatrixXd::Zero(3, 4);

	for (int i = 0; i < 4; i++)  // loop over shape functions
	{
		for (int j = 0; j < 3; j++)  // loop over global coordinates x, y, z
		{
			shapefunction_GlobalfirstDerivativeMatrix.coeffRef(j, i) = invjacobianMatrix(j, 0) * shapefunction_firstDerivativeMatrix(0, i) +
				invjacobianMatrix(j, 1) * shapefunction_firstDerivativeMatrix(1, i);
		}
	}


	Eigen::MatrixXd shapefunction_LocalfirstDerivativeMatrix = Eigen::MatrixXd::Zero(2, 5);

	for (int j = 0; j < 2; j++)  // local directions e and n
	{
		for (int i = 0; i < 4; i++)  // shape function index
		{
			shapefunction_LocalfirstDerivativeMatrix.coeffRef(j, i) =
				transformation_matrix(0, j) * shapefunction_GlobalfirstDerivativeMatrix(0, i) +
				transformation_matrix(1, j) * shapefunction_GlobalfirstDerivativeMatrix(1, i) +
				transformation_matrix(2, j) * shapefunction_GlobalfirstDerivativeMatrix(2, i);
		}

		// Compute DNL(j, 4) � the 5th column
		shapefunction_LocalfirstDerivativeMatrix.coeffRef(j, 4) =
			transformation_matrix(0, j) * invjacobianMatrix(0, 2) +
			transformation_matrix(1, j) * invjacobianMatrix(1, 2) +
			transformation_matrix(2, j) * invjacobianMatrix(2, 2);
	}


	// Strain Displacement Matrix of Drilling DOF
	StrainDisplacementDrillingDOFMatrix = Eigen::VectorXd::Zero(24);  // 4 nodes � 6 DOF each = 24 

	for (int i = 0; i < 4; ++i)  // I = 1 to 4
	{
		int ii = 6 * i;

		// BA(II+1 to II+3)
		StrainDisplacementDrillingDOFMatrix(ii + 0) =
			(shapefunction_LocalfirstDerivativeMatrix(0, i) * transformation_matrix(0, 1) -
				shapefunction_LocalfirstDerivativeMatrix(1, i) * transformation_matrix(0, 0)) / 2.0;

		StrainDisplacementDrillingDOFMatrix(ii + 1) =
			(shapefunction_LocalfirstDerivativeMatrix(0, i) * transformation_matrix(1, 1) -
				shapefunction_LocalfirstDerivativeMatrix(1, i) * transformation_matrix(1, 0)) / 2.0;

		StrainDisplacementDrillingDOFMatrix(ii + 2) =
			(shapefunction_LocalfirstDerivativeMatrix(0, i) * transformation_matrix(2, 1) -
				shapefunction_LocalfirstDerivativeMatrix(1, i) * transformation_matrix(2, 0)) / 2.0;


		// fact1 and fact2
		double fact1 = integration_ptz * shapefunction_LocalfirstDerivativeMatrix(0, i) +
			shapeFunction(i) * shapefunction_LocalfirstDerivativeMatrix(0, 4);

		double fact2 = integration_ptz * shapefunction_LocalfirstDerivativeMatrix(1, i) +
			shapeFunction(i) * shapefunction_LocalfirstDerivativeMatrix(1, 4);


		// TV values
		double tv22 = transformation_matrix.col(1).dot(p_matrix_local[i].col(1));
		double tv21 = transformation_matrix.col(1).dot(p_matrix_local[i].col(0));
		double tv12 = transformation_matrix.col(0).dot(p_matrix_local[i].col(1));
		double tv11 = transformation_matrix.col(0).dot(p_matrix_local[i].col(0));


		// BA(II+4) and BA(II+5)
		StrainDisplacementDrillingDOFMatrix(ii + 3) = -((fact1 * tv22) - (fact2 * tv12)) * (thickness / 4.0);
		StrainDisplacementDrillingDOFMatrix(ii + 4) = ((fact1 * tv21) - (fact2 * tv11)) * (thickness / 4.0);

		// BA(II+6)
		StrainDisplacementDrillingDOFMatrix(ii + 5) = -shapeFunction(i);

	}


}



void quadMITC4_element::computeMainStrainDisplacementMatrix(const double& integration_ptx, const double& integration_pty, const double& integration_ptz,
	const double& thickness, const std::array<Eigen::Matrix3d, 4>& p_matrix_local,
	const Eigen::MatrixXd& TransverseShearStrainMatrix, const Eigen::Matrix3d& initial_transformation_matrix,
	const Eigen::Vector4d& shapeFunction, const Eigen::MatrixXd& shapefunction_firstDerivativeMatrix,
	const Eigen::Matrix3d& jacobianMatrix, const Eigen::Matrix3d& invjacobianMatrix,
	const Eigen::Matrix3d& transformation_matrix,
	Eigen::MatrixXd& StrainDisplacementMatrixAtIntegrationPt, Eigen::MatrixXd& transformation_matrix_phi)
{
	// Computes the strain displacement matrix (B) and
	// Transformation matric phi

	double gv22 = 0.0;
	double gv21 = 0.0;

	double gv21_jc1 = 0.0;
	double gv22_jc2 = 0.0;

	double gv11 = 0.0;
	double gv12 = 0.0;

	// Calculate the phi transformation matrix
	transformation_matrix_phi = Eigen::MatrixXd::Zero(5, 6);
	transformation_matrix_phi = computePhiMatrix(invjacobianMatrix, transformation_matrix);

	// Initialize the Strain Displacement B - matrix
	StrainDisplacementMatrixAtIntegrationPt = Eigen::MatrixXd::Zero(6, 28);

	for (int i = 0; i < 4; ++i)
	{
		int ii = 6 * i;

		// Row 1
		StrainDisplacementMatrixAtIntegrationPt(0, ii + 0) = shapefunction_firstDerivativeMatrix(0, i) * jacobianMatrix(0, 0);
		StrainDisplacementMatrixAtIntegrationPt(0, ii + 1) = shapefunction_firstDerivativeMatrix(0, i) * jacobianMatrix(0, 1);
		StrainDisplacementMatrixAtIntegrationPt(0, ii + 2) = shapefunction_firstDerivativeMatrix(0, i) * jacobianMatrix(0, 2);

		gv22 = jacobianMatrix.row(0).dot(p_matrix_local[i].col(1));
		StrainDisplacementMatrixAtIntegrationPt(0, ii + 3) = -integration_ptz * shapefunction_firstDerivativeMatrix(0, i) * gv22 * thickness / 2.0;

		gv21 = jacobianMatrix.row(0).dot(p_matrix_local[i].col(0));
		StrainDisplacementMatrixAtIntegrationPt(0, ii + 4) = integration_ptz * shapefunction_firstDerivativeMatrix(0, i) * gv21 * thickness / 2.0;

		// Row 2
		StrainDisplacementMatrixAtIntegrationPt(1, ii + 0) = shapefunction_firstDerivativeMatrix(1, i) * jacobianMatrix(1, 0);
		StrainDisplacementMatrixAtIntegrationPt(1, ii + 1) = shapefunction_firstDerivativeMatrix(1, i) * jacobianMatrix(1, 1);
		StrainDisplacementMatrixAtIntegrationPt(1, ii + 2) = shapefunction_firstDerivativeMatrix(1, i) * jacobianMatrix(1, 2);

		gv22 = jacobianMatrix.row(1).dot(p_matrix_local[i].col(1));
		StrainDisplacementMatrixAtIntegrationPt(1, ii + 3) = -integration_ptz * shapefunction_firstDerivativeMatrix(1, i) * gv22 * thickness / 2.0;

		gv21 = jacobianMatrix.row(1).dot(p_matrix_local[i].col(0));
		StrainDisplacementMatrixAtIntegrationPt(1, ii + 4) = integration_ptz * shapefunction_firstDerivativeMatrix(1, i) * gv21 * thickness / 2.0;

		// Row 3 (only rotational DOFs)
		gv22 = jacobianMatrix.row(2).dot(p_matrix_local[i].col(1));
		StrainDisplacementMatrixAtIntegrationPt(2, ii + 3) = -shapeFunction(i) * gv22 * thickness / 2.0;

		gv21 = jacobianMatrix.row(2).dot(p_matrix_local[i].col(0));
		StrainDisplacementMatrixAtIntegrationPt(2, ii + 4) = shapeFunction(i) * gv21 * thickness / 2.0;

		// Row 4 (shear strain)
		StrainDisplacementMatrixAtIntegrationPt(3, ii + 0) = (shapefunction_firstDerivativeMatrix(0, i) * jacobianMatrix(1, 0) +
			shapefunction_firstDerivativeMatrix(1, i) * jacobianMatrix(0, 0)) / 2.0;
		StrainDisplacementMatrixAtIntegrationPt(3, ii + 1) = (shapefunction_firstDerivativeMatrix(0, i) * jacobianMatrix(1, 1) +
			shapefunction_firstDerivativeMatrix(1, i) * jacobianMatrix(0, 1)) / 2.0;
		StrainDisplacementMatrixAtIntegrationPt(3, ii + 2) = (shapefunction_firstDerivativeMatrix(0, i) * jacobianMatrix(1, 2) +
			shapefunction_firstDerivativeMatrix(1, i) * jacobianMatrix(0, 2)) / 2.0;

		gv21_jc1 = jacobianMatrix.row(0).dot(p_matrix_local[i].col(1));
		gv22_jc2 = jacobianMatrix.row(1).dot(p_matrix_local[i].col(1));
		StrainDisplacementMatrixAtIntegrationPt(3, ii + 3) = -integration_ptz *
			(shapefunction_firstDerivativeMatrix(1, i) * gv21_jc1 + shapefunction_firstDerivativeMatrix(0, i) * gv22_jc2) * thickness / 4.0;

		gv11 = jacobianMatrix.row(0).dot(p_matrix_local[i].col(0));
		gv12 = jacobianMatrix.row(1).dot(p_matrix_local[i].col(0));
		StrainDisplacementMatrixAtIntegrationPt(3, ii + 4) = integration_ptz *
			(shapefunction_firstDerivativeMatrix(1, i) * gv11 + shapefunction_firstDerivativeMatrix(0, i) * gv12) * thickness / 4.0;
	}

	for (int j = 0; j < 24; ++j)
	{
		StrainDisplacementMatrixAtIntegrationPt(4, j) = (1.0 - integration_ptx) * TransverseShearStrainMatrix(0, j) / 2.0 +
			(1.0 + integration_ptx) * TransverseShearStrainMatrix(1, j) / 2.0;

		StrainDisplacementMatrixAtIntegrationPt(5, j) = (1.0 - integration_pty) * TransverseShearStrainMatrix(2, j) / 2.0 +
			(1.0 + integration_pty) * TransverseShearStrainMatrix(3, j) / 2.0;
	}

	// Calculate the B matrix corresponding to the extra shape function

	double gt11 = jacobianMatrix.row(0).dot(initial_transformation_matrix.col(0));
	double gt12 = jacobianMatrix.row(0).dot(initial_transformation_matrix.col(1));
	double gt21 = jacobianMatrix.row(1).dot(initial_transformation_matrix.col(0));
	double gt22 = jacobianMatrix.row(1).dot(initial_transformation_matrix.col(1));

	StrainDisplacementMatrixAtIntegrationPt(0, 24) = -2.0 * integration_ptx * gt11;
	StrainDisplacementMatrixAtIntegrationPt(0, 25) = -2.0 * integration_ptx * gt12;
	StrainDisplacementMatrixAtIntegrationPt(1, 26) = -2.0 * integration_pty * gt21;
	StrainDisplacementMatrixAtIntegrationPt(1, 27) = -2.0 * integration_pty * gt22;
	StrainDisplacementMatrixAtIntegrationPt(3, 24) = -integration_ptx * gt21;
	StrainDisplacementMatrixAtIntegrationPt(3, 25) = -integration_ptx * gt22;
	StrainDisplacementMatrixAtIntegrationPt(3, 26) = -integration_pty * gt11;
	StrainDisplacementMatrixAtIntegrationPt(3, 27) = -integration_pty * gt12;


}




void quadMITC4_element::computeBMatrixExtraShapeFunction(const double& thickness, const std::array<Eigen::Matrix3d, 4>& p_matrix_local,
	const Eigen::Vector3d& ref_vector,
	Eigen::MatrixXd& StrainDisplacementMatrixExtraShapeFunction)
{
	// Computes the B Matrix for extra shape function (Bulge shape function of quad element)

	double integration_ptx = 0.0;
	double integration_pty = 0.0;
	double integration_ptz = 0.0;


	// At zeroth point
	Eigen::Vector4d shapeFunction = Eigen::Vector4d::Zero(); // Shape function
	Eigen::MatrixXd shapefunction_firstDerivativeMatrix = Eigen::MatrixXd::Zero(2, 4); // Shape function First derivative
	Eigen::Matrix3d jacobianMatrix = Eigen::Matrix3d::Zero(); // Jacobian Matrix

	computeShapeFunctonNJacobianMatrix(0.0, 0.0, 0.0, thickness, p_matrix_local,
		shapeFunction, shapefunction_firstDerivativeMatrix, jacobianMatrix);


	Eigen::Matrix3d initial_transformation_matrix = computeInitialTransformationMatrix(jacobianMatrix);

	// Store the strain displacement matrix at extra shape function  bic_matrix = Eigen::MatrixXd::Zero(5, 4);
	StrainDisplacementMatrixExtraShapeFunction = Eigen::MatrixXd::Zero(5, 4);


	for (int i = 0; i < 2; i++)
	{
		// integration point z
		integration_ptz = -1.0 * this->integration_points(i, 0);

		for (int j = 0; j < 2; j++)
		{
			// integration point y
			integration_pty = this->integration_points(j, 0);

			for (int k = 0; k < 2; k++)
			{
				// integration point x
				integration_ptx = (integration_pty < 0.0) ? +this->integration_points(k, 0)
					: -1.0 * this->integration_points(k, 0);


				// Compute Strain Displacement Matrix(B) for numerical integration pont
				// having local coordinates integration_ptx, integration_pty and integration_ptz

				Eigen::MatrixXd StrainDisplacementMatrixAtIntegrationPt = Eigen::MatrixXd::Zero(6, 28);
				Eigen::MatrixXd transformation_matrix_phi = Eigen::MatrixXd::Zero(5, 6);


				computeExtraStrainDisplacementMatrix(integration_ptx, integration_pty, integration_ptz,
					thickness, p_matrix_local, ref_vector, initial_transformation_matrix,
					StrainDisplacementMatrixAtIntegrationPt, transformation_matrix_phi);


				// Transform B to BL: BL = PHI * B_column
				Eigen::MatrixXd bl_matrix = Eigen::MatrixXd::Zero(5, 4);

				bl_matrix = transformation_matrix_phi * StrainDisplacementMatrixAtIntegrationPt.block<6, 4>(0, 24);


				// Accumulate to the Strain Displacement Matrix of Extra Shape Function
				StrainDisplacementMatrixExtraShapeFunction.noalias() =
					StrainDisplacementMatrixExtraShapeFunction - bl_matrix;

			}

		}

	}


}



void quadMITC4_element::computeExtraStrainDisplacementMatrix(const double& integration_ptx, const double& integration_pty, const double& integration_ptz,
	const double& thickness, const std::array<Eigen::Matrix3d, 4>& p_matrix_local, const Eigen::Vector3d& ref_vector,
	const Eigen::Matrix3d& initial_transformation_matrix,
	Eigen::MatrixXd& StrainDisplacementMatrixAtIntegrationPt, Eigen::MatrixXd& transformation_matrix_phi)
{
	// Computes the strain displacement matrix (B) and
	// Transformation matric phi
	StrainDisplacementMatrixAtIntegrationPt = Eigen::MatrixXd::Zero(6, 28);
	transformation_matrix_phi = Eigen::MatrixXd::Zero(5, 6);


	Eigen::Vector4d shapeFunction = Eigen::Vector4d::Zero(); // Shape function
	Eigen::MatrixXd shapefunction_firstDerivativeMatrix = Eigen::MatrixXd::Zero(2, 4); // Shape function First derivative
	Eigen::Matrix3d jacobianMatrix = Eigen::Matrix3d::Zero(); // Jacobian Matrix


	// Compute the shape function, derivative shape function and jacobian matrix at integration points
	computeShapeFunctonNJacobianMatrix(integration_ptx, integration_pty, integration_ptz,
		thickness, p_matrix_local,
		shapeFunction, shapefunction_firstDerivativeMatrix, jacobianMatrix);


	// Calculate the jacobian determinant and  inverse of jacobian
	Eigen::Matrix3d invjacobianMatrix = jacobianMatrix.inverse();

	// Calculate the transformation matrix at the integration point
	Eigen::Matrix3d transformation_matrix = computeTransformationMatrixFromReference(jacobianMatrix, ref_vector);


	// Calculate the phi transformation matrix
	transformation_matrix_phi = computePhiMatrix(invjacobianMatrix, transformation_matrix);

	// Extra shape functions (always calculated)
	double gt11 = jacobianMatrix.row(0).dot(initial_transformation_matrix.col(0));
	double gt12 = jacobianMatrix.row(0).dot(initial_transformation_matrix.col(1));
	double gt21 = jacobianMatrix.row(1).dot(initial_transformation_matrix.col(0));
	double gt22 = jacobianMatrix.row(1).dot(initial_transformation_matrix.col(1));

	StrainDisplacementMatrixAtIntegrationPt(0, 24) = -2.0 * integration_ptx * gt11;
	StrainDisplacementMatrixAtIntegrationPt(0, 25) = -2.0 * integration_ptx * gt12;
	StrainDisplacementMatrixAtIntegrationPt(1, 26) = -2.0 * integration_pty * gt21;
	StrainDisplacementMatrixAtIntegrationPt(1, 27) = -2.0 * integration_pty * gt22;
	StrainDisplacementMatrixAtIntegrationPt(3, 24) = -integration_ptx * gt21;
	StrainDisplacementMatrixAtIntegrationPt(3, 25) = -integration_ptx * gt22;
	StrainDisplacementMatrixAtIntegrationPt(3, 26) = -integration_pty * gt11;
	StrainDisplacementMatrixAtIntegrationPt(3, 27) = -integration_pty * gt12;

}




void quadMITC4_element::computeShapeFunctonNJacobianMatrix(const double& xp, const double& yp, const double& zp,
	const double& thickness, const std::array<Eigen::Matrix3d, 4>& p_matrix_local,
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


	Eigen::MatrixXd phi_matrix = Eigen::MatrixXd::Zero(5, 6);

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


Eigen::MatrixXd quadMITC4_element::computeElemMassMatrixAtIntegrationPoint(const Eigen::Vector4d& shapeFunction,
	const std::array<Eigen::Matrix3d, 4>& p_matrix_local,
	const double& thickness,
	const double& integration_ptz,
	const double& element_mass)
{

	// Mass B Matrix
	Eigen::MatrixXd massBMatrix = Eigen::MatrixXd::Zero(3, 24);

	int index = 0;
	for (int j = 0; j < 4; j++)
	{
		double f1 = shapeFunction(j);
		double f2 = f1 * integration_ptz * thickness * 0.5;

		// Identity diagonal
		massBMatrix.coeffRef(0, index + 0) = f1;
		massBMatrix.coeffRef(1, index + 1) = f1;
		massBMatrix.coeffRef(2, index + 2) = f1;

		// Rotational terms using corrected form (negative P(:,2) and positive P(:,1))
		massBMatrix.coeffRef(0, index + 3) = -f2 * p_matrix_local[j](0, 1);
		massBMatrix.coeffRef(1, index + 3) = -f2 * p_matrix_local[j](1, 1);
		massBMatrix.coeffRef(2, index + 3) = -f2 * p_matrix_local[j](2, 1);

		massBMatrix.coeffRef(0, index + 4) = f2 * p_matrix_local[j](0, 0);
		massBMatrix.coeffRef(1, index + 4) = f2 * p_matrix_local[j](1, 0);
		massBMatrix.coeffRef(2, index + 4) = f2 * p_matrix_local[j](2, 0);

		// Zero column (I+6)
		massBMatrix.coeffRef(0, index + 5) = 0.0;
		massBMatrix.coeffRef(1, index + 5) = 0.0;
		massBMatrix.coeffRef(2, index + 5) = 0.0;

		index += 6;
	}


	// Assemble element mass matrix: EM = mB^T * mB * element_mass
	Eigen::MatrixXd elemMassMatrix = Eigen::MatrixXd::Zero(24, 24);
	elemMassMatrix = (massBMatrix.transpose() * massBMatrix) * element_mass;


	return elemMassMatrix;

}



void quadMITC4_element::transform_localrotation_to_globalrotation(Eigen::MatrixXd& K_matrix,
	const std::array<Eigen::Matrix3d, 4>& p_matrix)
{
	const int nodes = 4;
	const int dof_per_node = 6;
	const int total_dof = nodes * dof_per_node;

	// Create 6x6 block-diagonal rotation matrices for each node
	std::array<Eigen::Matrix<double, 6, 6>, nodes> P_blocks;
	for (int i = 0; i < nodes; i++)
	{
		Eigen::Matrix<double, 6, 6> B = Eigen::Matrix<double, 6, 6>::Zero();
		B.topLeftCorner<3, 3>().setIdentity();
		B.bottomRightCorner<3, 3>() = p_matrix[i].transpose(); // Transpose of p_matrix !!!!
		P_blocks[i] = B;
	}

	// Create a deep copy for local, zero global result
	Eigen::MatrixXd K_local = K_matrix;
	Eigen::MatrixXd K_global = Eigen::MatrixXd::Zero(total_dof, total_dof);

	// Perform block-wise transformation
	for (int i = 0; i < nodes; i++)
	{
		for (int j = i; j < nodes; j++)
		{
			int row = i * dof_per_node;
			int col = j * dof_per_node;

			Eigen::Matrix<double, 6, 6> block = K_local.block<6, 6>(row, col);
			Eigen::Matrix<double, 6, 6> transformed_block =
				P_blocks[i].transpose() * block * P_blocks[j];

			// Assign to global matrix
			K_global.block<6, 6>(row, col) = transformed_block;
			if (i != j)
			{
				K_global.block<6, 6>(col, row) = transformed_block.transpose();  // symmetric
			}
		}
	}

	// Write result back
	K_matrix = std::move(K_global);


}



void quadMITC4_element::transform_stiffness_to_globalcoordinates(Eigen::MatrixXd& K_matrix,
	const Eigen::Matrix3d& local_coordinate_matrix)
{

	const int nodes = 4;
	const int dof_per_node = 6;
	const int total_dof = nodes * dof_per_node;


	Eigen::Matrix<double, 6, 6> P_node = Eigen::Matrix<double, 6, 6>::Zero();
	P_node.topLeftCorner<3, 3>() = local_coordinate_matrix; // Translations transformed
	P_node.bottomRightCorner<3, 3>() = local_coordinate_matrix; // Rotations transformed


	// Build full 24�24 transformation matrix (block diagonal),
	// each block transforms 6 DOFs: [I3 for translations, E0 for rotations]

	Eigen::MatrixXd trans_Pmatrix = Eigen::MatrixXd::Zero(total_dof, total_dof);

	for (int i = 0; i < nodes; i++) 
	{
		int idx = i * dof_per_node;
		trans_Pmatrix.block<6, 6>(idx, idx) = P_node;
	}


	// Apply the transformation
	Eigen::MatrixXd K_global = Eigen::MatrixXd::Zero(total_dof, total_dof);

	K_global = trans_Pmatrix.transpose() * K_matrix * trans_Pmatrix;

	// Write result back
	K_matrix = std::move(K_global);


}


void quadMITC4_element::diagonalize_mass_matrix(Eigen::MatrixXd& massMatrix, const double& element_mass)
{
	const int nodes = 4;      // Number of nodes
	const int dof_per_node = 6;     // DOFs per node
	const int total_dof = nodes * dof_per_node;  // Total DOFs

	// 1. Compute total translational diagonal mass
	double emss1 = 0.0, emss2 = 0.0, emss3 = 0.0;

	for (int i = 0; i < nodes; i++) 
	{
		int row_base = i * dof_per_node;

		emss1 += massMatrix(row_base + 0, row_base + 0);  // x
		emss2 += massMatrix(row_base + 1, row_base + 1);  // y
		emss3 += massMatrix(row_base + 2, row_base + 2);  // z
	}

	double emss = (emss1 > 1e-20) ? emss1 :
		(emss2 > 1e-20) ? emss2 :
		(emss3 > 1e-20) ? emss3 : 0.0;

	// 2. Compute scaling factor
	double fact = (emss != 0.0) ? element_mass / emss : 0.0;

	// 3. Apply scaling and zero off-diagonal terms
	for (int i = 0; i < total_dof; i++)
	{
		for (int j = 0; j < total_dof; j++)
		{
			if (i == j) 
			{
				massMatrix(i, j) *= fact;
			}
			else 
			{
				massMatrix(i, j) = 0.0;
			}
		}
	}


}



void quadMITC4_element::matrixToString(const Eigen::MatrixXd& mat)
{
	std::ostringstream oss;
	oss.precision(8);
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


void quadMITC4_element::vectorToString(const Eigen::VectorXd& vec)
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









