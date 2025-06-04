#include "modal_analysis_solver.h"

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

	// Fill first Jacobian matrix (2x3): derivatives of shape functions
	jacobianMatrix(0, 0) = b1 * invTwoArea;
	jacobianMatrix(0, 1) = b2 * invTwoArea;
	jacobianMatrix(0, 2) = b3 * invTwoArea;

	jacobianMatrix(1, 0) = c1 * invTwoArea;
	jacobianMatrix(1, 1) = c2 * invTwoArea;
	jacobianMatrix(1, 2) = c3 * invTwoArea;

	// Fill second matrix (3x6): products of derivatives for stiffness calculation
	for (int j = 0; j < 3; ++j)
	{
		double bj = jacobianMatrix(0, j);
		double cj = jacobianMatrix(1, j);

		jacobianProducts(0, j) = bj * bj;           // b^2
		jacobianProducts(1, j) = cj * cj;           // c^2
		jacobianProducts(2, j) = bj * cj;           // b * c

		int m = j + 3;
		int next = (j + 1) % 3; // Wrap-around index for pairs

		double bjNext = jacobianMatrix(0, next);
		double cjNext = jacobianMatrix(1, next);

		jacobianProducts(0, m) = 2.0 * bj * bjNext;
		jacobianProducts(1, m) = 2.0 * cj * cjNext;
		jacobianProducts(2, m) = bj * cjNext + bjNext * cj;
	}

}








