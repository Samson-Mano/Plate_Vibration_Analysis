#include "modal_analysis_solver.h"
#include "triCKZ_element.h"

modal_analysis_solver::modal_analysis_solver()
{
	// Empty constructor
}

modal_analysis_solver::~modal_analysis_solver()
{
	// Empty destructor
}


void modal_analysis_solver::clear_results()
{
	// Clear the eigen values and eigen vectors
	constrained_node_map.clear(); // Constrained Node  map
	nodeid_map.clear(); // Node ID map
	number_of_modes = 0;
	node_count = 0;

	mode_result_str.clear();
	is_modal_analysis_complete = false;
}


void modal_analysis_solver::modal_analysis_start(const nodes_list_store& model_nodes,
	const elementtri_list_store& model_trielements,
	const elementquad_list_store& model_quadelements,
	const nodecnst_list_store& node_cnst,
	const material_data& mat_data,
	rslt_nodes_list_store& modal_result_nodes,
	rslt_elementtri_list_store& modal_result_trielements,
	rslt_elementquad_list_store& modal_result_quadelements)
{
	// Main solver call
	this->is_modal_analysis_complete = false;

	// Check the model
	// Number of nodes
	if (model_nodes.node_count == 0)
	{
		return;
	}

	// Number of quads/ tris
	if ((model_quadelements.elementquad_count + model_trielements.elementtri_count) == 0)
	{
		return;
	}

	//____________________________________________
	Eigen::initParallel();  // Initialize Eigen's thread pool

	stopwatch.start();

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Modal analysis - started" << std::endl;
	//____________________________________________________________________________________________________________________


	this->node_count = model_nodes.node_count;


	this->matrix_size = 0;


	// Create stiffness matrix






	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal analysis complete at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


}


void modal_analysis_solver::computeLocalCoordinateSystem(
	const Eigen::Vector3d& p,  // Point P
	const Eigen::Vector3d& q,  // Point Q
	const Eigen::Vector3d& r,  // Point R
	double& sin_angle,
	double& cos_angle,
	double& dpq,
	double& dpr,
	Eigen::Matrix3d& coordinateSystemE)
{
	// Vectors PQ and PR
	Eigen::Vector3d v_pq = q - p;
	Eigen::Vector3d v_pr = r - p;

	dpq = v_pq.norm();
	dpr = v_pr.norm();

	// Normal vector to the triangle (Z-axis)
	Eigen::Vector3d v_z = v_pq.cross(v_pr);
	double dz = v_z.norm();

	// Sine and Cosine of angle between PQ and PR
	sin_angle = dz / (dpq * dpr);
	cos_angle = v_pq.dot(v_pr) / (dpq * dpr);

	// Unit vectors for local coordinate system
	Eigen::Vector3d x_local = v_pq.normalized();           // Local x-axis
	Eigen::Vector3d z_local = v_z.normalized();            // Local z-axis
	Eigen::Vector3d y_local = z_local.cross(x_local);      // Local y-axis

	// Construct transformation matrix E (columns: X, Y, Z)
	coordinateSystemE.col(0) = x_local;
	coordinateSystemE.col(1) = y_local;
	coordinateSystemE.col(2) = z_local;

}


Eigen::MatrixXd modal_analysis_solver::generateTriangleIntegrationPoints(int nip)
{
	Eigen::MatrixXd integration_points(nip, 4); // Each row: [L1, L2, L3, weight]
	Eigen::Vector4d temp_row;

	temp_row(0) = 1.0 / 3.0;
	temp_row(1) = 1.0 / 3.0;
	temp_row(2) = 1.0 / 3.0;

	if (nip == 4)
	{
		temp_row(3) = -27.0 / 48.0;
	}
	else if (nip == 7)
	{
		temp_row(3) = 9.0 / 40.0;
	}
	else
	{
		temp_row(3) = 0.0;
	}

	integration_points.row(0) = temp_row;

	double alpha = 0.6;
	double beta = 0.2;
	double weight = 25.0 / 48.0;
	double sq15 = std::sqrt(15.0);

	if (nip == 7)
	{
		alpha = (9.0 + 2.0 * sq15) / 21.0;
		beta = (6.0 - sq15) / 21.0;
		weight = (155.0 - sq15) / 1200.0;
	}

	for (int i = 1; i <= 3; ++i)
	{
		temp_row(0) = beta;
		temp_row(1) = beta;
		temp_row(2) = beta;
		temp_row(3) = weight;

		temp_row(i - 1) = alpha;  // Set L1, L2, or L3
		integration_points.row(i) = temp_row;
	}

	if (nip == 4) return integration_points;

	alpha = (9.0 - 2.0 * sq15) / 21.0;
	beta = (6.0 + sq15) / 21.0;
	weight = (155.0 + sq15) / 1200.0;

	for (int i = 4; i <= 6; ++i)
	{
		temp_row(0) = beta;
		temp_row(1) = beta;
		temp_row(2) = beta;
		temp_row(3) = weight;

		int j = i - 3;
		temp_row(j - 1) = alpha;  // Set L1, L2, or L3
		integration_points.row(i) = temp_row;
	}

	return integration_points;

}


void modal_analysis_solver::computeJacobianCoefficients(
	double x1, double y1,
	double x2, double y2,
	double x3, double y3,
	double triangle_area,
	Eigen::Matrix<double, 2, 3>& jacobianMatrix,
	Eigen::Matrix<double, 3, 6>& jacobianProducts)
{

	// Reciprocal of 2 × triangle area (used as a scaling factor)
	double invTwoArea = 1.0 / (2.0 * triangle_area);

	// Differences in y-coordinates for B vector components
	double b1 = y2 - y3;
	double b2 = y3 - y1;
	double b3 = y1 - y2;

	// Differences in x-coordinates for C vector components
	double c1 = x3 - x2;
	double c2 = x1 - x3;
	double c3 = x2 - x1;

	// Fill first Jacobian matrix (2x3): derivatives of area co-ordinates L1, L2, L3
	// 
	//  | dL1/dx  dL2/dx  dL3/dx |
	//  | dL1/dy  dL2/dy  dL3/dy |
	//

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


void modal_analysis_solver::computeShapeFunctions(double B1, double B2, double B3,
	double C1, double C2, double C3,
	double L1, double L2, double L3,
	const Eigen::Matrix<double, 2, 3>& jacobianMatrix,
	const Eigen::Matrix<double, 3, 6>& jacobianProducts,
	Eigen::VectorXd& shapeFunction,         // Size 9
	Eigen::MatrixXd& shapeGradient,         // Size 2x9
	Eigen::MatrixXd& secondDerivativeMatrix) // Size 6x9
{

	shapeFunction.setZero(9);
	shapeGradient.setZero(3, 9);
	secondDerivativeMatrix.setZero(6, 9);

	//__________________________________________________________________________________________________________________
	// Shape functions AN0
	shapeFunction(0) = L1 + (L1 * L1 * L2) + (L1 * L1 * L3) - (L1 * L2 * L2) - (L1 * L3 * L3);
	shapeFunction(1) = -B3 * ((L1 * L1 * L2) + (0.5 * L1 * L2 * L3)) + B2 * ((L3 * L1 * L1) + (0.5 * L1 * L2 * L3));
	shapeFunction(2) = -C3 * ((L1 * L1 * L2) + (0.5 * L1 * L2 * L3)) + C2 * ((L3 * L1 * L1) + (0.5 * L1 * L2 * L3));
	shapeFunction(3) = L2 + (L2 * L2 * L3) + (L2 * L2 * L1) - (L2 * L3 * L3) - (L2 * L1 * L1);
	shapeFunction(4) = -B1 * ((L2 * L2 * L3) + (0.5 * L1 * L2 * L3)) + B3 * ((L1 * L2 * L2) + (0.5 * L1 * L2 * L3));
	shapeFunction(5) = -C1 * ((L2 * L2 * L3) + (0.5 * L1 * L2 * L3)) + C3 * ((L1 * L2 * L2) + (0.5 * L1 * L2 * L3));
	shapeFunction(6) = L3 + (L3 * L3 * L1) + (L3 * L3 * L2) - (L3 * L1 * L1) - (L3 * L2 * L2);
	shapeFunction(7) = -B2 * ((L3 * L3 * L1) + (0.5 * L1 * L2 * L3)) + B1 * ((L2 * L3 * L3) + (0.5 * L1 * L2 * L3));
	shapeFunction(8) = -C2 * ((L3 * L3 * L1) + (0.5 * L1 * L2 * L3)) + C1 * ((L2 * L3 * L3) + (0.5 * L1 * L2 * L3));


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
	shapeGradient(0, 1) = -B3 * (2.0 * L1 * L2 + 0.5 * L2 * L3) + B2 * (2.0 * L3 * L1 + 0.5 * L2 * L3); // dN'1/dL1
	shapeGradient(1, 4) = -B1 * (2.0 * L2 * L3 + 0.5 * L3 * L1) + B3 * (2.0 * L1 * L2 + 0.5 * L3 * L1); // dN'2/dL2
	shapeGradient(2, 7) = -B2 * (2.0 * L3 * L1 + 0.5 * L1 * L2) + B1 * (2.0 * L2 * L3 + 0.5 * L1 * L2); // dN'3/dL3

	shapeGradient(1, 1) = -B3 * (L1 * L1 + 0.5 * L1 * L3) + B2 * (0.5 * L1 * L3); // dN'1/dL2
	shapeGradient(2, 4) = -B1 * (L2 * L2 + 0.5 * L2 * L1) + B3 * (0.5 * L2 * L1); // dN'2/dL3
	shapeGradient(0, 7) = -B2 * (L3 * L3 + 0.5 * L3 * L2) + B1 * (0.5 * L3 * L2); // dN'3/dL1

	shapeGradient(2, 1) = -B3 * (0.5 * L1 * L2) + B2 * (L1 * L1 + 0.5 * L1 * L2); // dN'1/dL3
	shapeGradient(0, 4) = -B1 * (0.5 * L2 * L3) + B3 * (L2 * L2 + 0.5 * L2 * L3); // dN'2/dL1
	shapeGradient(1, 7) = -B2 * (0.5 * L3 * L1) + B1 * (L3 * L3 + 0.5 * L3 * L1); // dN'3/dL2

	// C terms
	shapeGradient(0, 2) = -C3 * (2.0 * L1 * L2 + 0.5 * L2 * L3) + C2 * (2.0 * L3 * L1 + 0.5 * L2 * L3); // dN''1/dL1
	shapeGradient(1, 5) = -C1 * (2.0 * L2 * L3 + 0.5 * L3 * L1) + C3 * (2.0 * L1 * L2 + 0.5 * L3 * L1); // dN''2/dL2
	shapeGradient(2, 8) = -C2 * (2.0 * L3 * L1 + 0.5 * L1 * L2) + C1 * (2.0 * L2 * L3 + 0.5 * L1 * L2); // dN''3/dL3

	shapeGradient(1, 2) = -C3 * (L1 * L1 + 0.5 * L1 * L3) + C2 * (0.5 * L1 * L3); // dN''1/dL2
	shapeGradient(2, 5) = -C1 * (L2 * L2 + 0.5 * L2 * L1) + C3 * (0.5 * L2 * L1); // dN''2/dL3
	shapeGradient(0, 8) = -C2 * (L3 * L3 + 0.5 * L3 * L2) + C1 * (0.5 * L3 * L2); // dN''3/dL1

	shapeGradient(2, 2) = -C3 * (0.5 * L1 * L2) + C2 * (L1 * L1 + 0.5 * L1 * L2); // dN''1/dL3
	shapeGradient(0, 5) = -C1 * (0.5 * L2 * L3) + C3 * (L2 * L2 + 0.5 * L2 * L3); // dN''2/dL1
	shapeGradient(1, 8) = -C2 * (0.5 * L3 * L1) + C1 * (L3 * L3 + 0.5 * L3 * L1); // dN''3/dL2


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


	secondDerivativeMatrix(0, 0) = 2.0 * (L2 + L3); // d^2N1/dL1^2
	secondDerivativeMatrix(1, 3) = 2.0 * (L3 + L1); // d^2N2/dL2^2
	secondDerivativeMatrix(2, 6) = 2.0 * (L1 + L2); // d^2N3/dL3^2
	secondDerivativeMatrix(1, 0) = -2.0 * L1; // d^2N1/dL2^2
	secondDerivativeMatrix(2, 3) = -2.0 * L2; // d^2N2/dL3^2
	secondDerivativeMatrix(0, 6) = -2.0 * L3; // d^2N3/dL1^2
	secondDerivativeMatrix(2, 0) = -2.0 * L1; // d^2N1/dL3^2 
	secondDerivativeMatrix(0, 3) = -2.0 * L2; // d^2N2/dL1^2 
	secondDerivativeMatrix(1, 6) = -2.0 * L3; // d^2N3/dL2^2

	secondDerivativeMatrix(0, 1) = (-2.0 * B3 * L2) + (2.0 * B2 * L3); // d^2N'1/dL1^2
	secondDerivativeMatrix(1, 4) = (-2.0 * B1 * L3) + (2.0 * B3 * L1); // d^2N'2/dL2^2
	secondDerivativeMatrix(2, 7) = (-2.0 * B2 * L1) + (2.0 * B1 * L2); // d^2N'3/dL3^2

	secondDerivativeMatrix(0, 2) = (-2.0 * C3 * L2) + (2.0 * C2 * L3); // d^2N''1/dL1^2
	secondDerivativeMatrix(1, 5) = (-2.0 * C1 * L3) + (2.0 * C3 * L1); // d^2N''2/dL2^2
	secondDerivativeMatrix(2, 8) = (-2.0 * C2 * L1) + (2.0 * C1 * L2); // d^2N''3/dL3^2

	// Additional terms... after 3rd row
	// N
	secondDerivativeMatrix(3, 0) = +2.0 * L1 - 2.0 * L2; // d^2N1/dL1dL2
	secondDerivativeMatrix(4, 3) = +2.0 * L2 - 2.0 * L3; // d^2N2/dL2dL3
	secondDerivativeMatrix(5, 6) = +2.0 * L3 - 2.0 * L1; // d^2N3/dL3dL1

	secondDerivativeMatrix(5, 0) = +2.0 * L1 - 2.0 * L3; // d^2N1/dL3dL1
	secondDerivativeMatrix(3, 3) = +2.0 * L2 - 2.0 * L1; // d^2N2/dL1dL2
	secondDerivativeMatrix(4, 6) = +2.0 * L3 - 2.0 * L2; // d^2N3/dL2dL3

	// N'
	secondDerivativeMatrix(3, 1) = -B3 * (2.0 * L1 + 0.5 * L3) + B2 * (0.5 * L3); // d^2N'1/dL1dL2
	secondDerivativeMatrix(4, 4) = -B1 * (2.0 * L2 + 0.5 * L1) + B3 * (0.5 * L1); // d^2N'2/dL2dL3
	secondDerivativeMatrix(5, 7) = -B2 * (2.0 * L3 + 0.5 * L2) + B1 * (0.5 * L2); // d^2N'3/dL3dL1

	secondDerivativeMatrix(4, 1) = -B3 * (L1 * 0.5) + B2 * (L1 * 0.5); // d^2N'1/dL2dL3
	secondDerivativeMatrix(5, 4) = -B1 * (L2 * 0.5) + B3 * (L2 * 0.5); // d^2N'2/dL3dL1
	secondDerivativeMatrix(3, 7) = -B2 * (L3 * 0.5) + B1 * (L3 * 0.5); // d^2N'3/dL1dL2

	secondDerivativeMatrix(5, 1) = -B3 * (L2 * 0.5) + B2 * (2.0 * L1 + L2 * 0.5); // d^2N'1/dL3dL1
	secondDerivativeMatrix(3, 4) = -B1 * (L3 * 0.5) + B3 * (2.0 * L2 + L3 * 0.5); // d^2N'2/dL1dL2
	secondDerivativeMatrix(4, 7) = -B2 * (L1 * 0.5) + B1 * (2.0 * L3 + L1 * 0.5); // d^2N'3/dL2dL3

	// N''
	secondDerivativeMatrix(3, 2) = -C3 * (2.0 * L1 + L3 * 0.5) + C2 * (L3 * 0.5); // d^2N''1/dL1dL2
	secondDerivativeMatrix(4, 5) = -C1 * (2.0 * L2 + L1 * 0.5) + C3 * (L1 * 0.5); // d^2N''2/dL2dL3
	secondDerivativeMatrix(5, 8) = -C2 * (2.0 * L3 + L2 * 0.5) + C1 * (L2 * 0.5); // d^2N''3/dL3dL1

	secondDerivativeMatrix(4, 2) = -C3 * (L1 * 0.5) + C2 * (L1 * 0.5); // d^2N''1/dL2dL3
	secondDerivativeMatrix(5, 5) = -C1 * (L2 * 0.5) + C3 * (L2 * 0.5); // d^2N''2/dL3dL1
	secondDerivativeMatrix(3, 8) = -C2 * (L3 * 0.5) + C1 * (L3 * 0.5); // d^2N''3/dL1dL2

	secondDerivativeMatrix(5, 2) = -C3 * (L2 * 0.5) + C2 * (2.0 * L1 + L2 * 0.5); // d^2N''1/dL3dL1
	secondDerivativeMatrix(3, 5) = -C1 * (L3 * 0.5) + C3 * (2.0 * L2 + L3 * 0.5); // d^2N''2/dL1dL2
	secondDerivativeMatrix(4, 8) = -C2 * (L1 * 0.5) + C1 * (2.0 * L3 + L1 * 0.5); // d^2N''3/dL2dL3


}




Eigen::MatrixXd modal_analysis_solver::computeStrainDisplacementMatrix(
	const Eigen::Matrix<double, 3, 6>& jacobianProducts,
	const Eigen::MatrixXd& shapefunction_secondDerivativeMatrix)
{
	Eigen::MatrixXd strain_displacement_matrix(3, 9);
	strain_displacement_matrix.setZero();

	// Compute B(I,J) = JC2(I,K) * AN2(K,J)
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 9; ++j)
		{
			double bij = 0.0;
			for (int k = 0; k < 6; ++k)
			{
				bij += jacobianProducts(i, k) * shapefunction_secondDerivativeMatrix(k, j);
			}
			strain_displacement_matrix(i, j) = bij;
		}
	}

	// Apply scaling factors
	for (int j = 0; j < 9; ++j)
	{
		strain_displacement_matrix(0, j) = -strain_displacement_matrix(0, j);
		strain_displacement_matrix(1, j) = -strain_displacement_matrix(1, j);
		strain_displacement_matrix(2, j) = -2.0 * strain_displacement_matrix(2, j);
	}

	return strain_displacement_matrix;
}




