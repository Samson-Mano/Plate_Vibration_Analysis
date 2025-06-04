#pragma once
#include <iostream>
#include <fstream>

// FE Objects
#include "../geometry_store/fe_objects/nodes_list_store.h"
#include "../geometry_store/fe_objects/elementtri_list_store.h"
#include "../geometry_store/fe_objects/elementquad_list_store.h"
#include "../geometry_store/fe_objects/nodecnst_list_store.h"

// FE Results Modal Analysis
#include "../geometry_store/result_objects/rslt_nodes_list_store.h"
#include "../geometry_store/result_objects/rslt_elementtri_list_store.h"
#include "../geometry_store/result_objects/rslt_elementquad_list_store.h"

// Stop watch
#include "../events_handler/Stopwatch_events.h"

#include <cmath>
#include <boost/math/special_functions/bessel.hpp>

#pragma warning(push)
#pragma warning (disable : 26451)
#pragma warning (disable : 26495)
#pragma warning (disable : 6255)
#pragma warning (disable : 6294)
#pragma warning (disable : 26813)
#pragma warning (disable : 26454)

// Optimization for Eigen Library
// 1) OpenMP (Yes (/openmp)
//	 Solution Explorer->Configuration Properties -> C/C++ -> Language -> Open MP Support
// 2) For -march=native, choose "AVX2" or the latest supported instruction set.
//   Solution Explorer->Configuration Properties -> C/C++ -> Code Generation -> Enable Enhanced Instruction Set 

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/Eigenvalues>
// Define the sparse matrix type for the reduced global stiffness matrix
typedef Eigen::SparseMatrix<double> SparseMatrix;
#pragma warning(pop)

typedef boost::math::policies::policy<
	boost::math::policies::domain_error<boost::math::policies::ignore_error>,
	boost::math::policies::overflow_error<boost::math::policies::ignore_error>,
	boost::math::policies::underflow_error<boost::math::policies::ignore_error>,
	boost::math::policies::denorm_error<boost::math::policies::ignore_error>,
	boost::math::policies::pole_error<boost::math::policies::ignore_error>,
	boost::math::policies::evaluation_error<boost::math::policies::ignore_error>
> ignore_all_policy;


class modal_analysis_solver
{
public:
	// Result store
	std::unordered_map<int, bool> constrained_node_map; // Node ID and Bool True = Constrained, False = Un Constrained
	std::unordered_map<int, int> nodeid_map; // Node ID map for eigen vectors
	int paint_mode_count = 100; // Draw only first 100 modes (To save memory)
	int number_of_modes = 0;
	int node_count = 0;
	int matrix_size = 0;
	int model_type = 0;

	// Eigen values matrices
	Eigen::VectorXd angular_freq_vector;
	Eigen::VectorXd eigen_values_vector;

	// Eigen vector matrices
	// Eigen::MatrixXd displ_vectors_matrix;
	Eigen::MatrixXd eigen_vectors_matrix;

	std::vector<std::string> mode_result_str;
	bool is_modal_analysis_complete = false;

	modal_analysis_solver();
	~modal_analysis_solver();
	void clear_results();

	void modal_analysis_start(const nodes_list_store& model_nodes,
		const elementtri_list_store& model_trielements,
		const elementquad_list_store& model_quadelements,
		const nodecnst_list_store& node_cnst,
		const material_data& mat_data,
		rslt_nodes_list_store& modal_result_nodes,
		rslt_elementtri_list_store& modal_result_trielements,
		rslt_elementquad_list_store& modal_result_quadelements);

private:
	const double m_pi = 3.14159265358979323846;
	const double epsilon = 0.000001;
	// const bool print_matrix = true;

	Stopwatch_events stopwatch;
	std::stringstream stopwatch_elapsed_str;

	void computeLocalCoordinateSystem(
		const Eigen::Vector3d& p,  // Point P
		const Eigen::Vector3d& q,  // Point Q
		const Eigen::Vector3d& r,  // Point R
		double& sin_angle,
		double& cos_angle,
		double& dpq,
		double& dpr, Eigen::Matrix3d& coordinateSystemE);


	Eigen::MatrixXd generateTriangleIntegrationPoints(int nip);


	void computeJacobianCoefficients(
		double x1, double y1,
		double x2, double y2,
		double x3, double y3,
		double triangle_area,
		Eigen::Matrix<double, 2, 3>& jacobianMatrix,
		Eigen::Matrix<double, 3, 6>& jacobianProducts);


};
