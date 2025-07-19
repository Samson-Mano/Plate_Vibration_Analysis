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

// Element stiffness support
#include "triCKZ_element.h"
#include "quadMITC4_element.h"


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
	int number_of_modes = 0;
	std::unordered_map<int, int> nodeid_map; // Node ID map
	std::unordered_map<int, double> m_eigenvalues;
	std::unordered_map<int, std::vector<double>> m_eigenvectors;
	std::vector<std::string> mode_result_str;

	int numDOF = 0;
	int reducedDOF = 0;


	std::unordered_map<int, bool> constrained_node_map; // Node ID and Bool True = Constrained, False = Un Constrained
	int paint_mode_count = 100; // Draw only first 100 modes (To save memory)
	// int node_count = 0;
	// int matrix_size = 0;

	// Matrix stored
	Eigen::VectorXd reduced_modalMass;
	Eigen::VectorXd reduced_modalStiff;
	Eigen::VectorXi globalDOFMatrix;
	Eigen::MatrixXd reduced_eigenvectors;
	Eigen::MatrixXd global_eigenvectors;
	// Eigen::MatrixXd global_eigenvectors_transformed;


	// Eigen values matrices
	Eigen::VectorXd angular_freq_vector;
	Eigen::VectorXd eigen_values_vector;

	// Eigen vector matrices
	Eigen::MatrixXd eigen_vectors_matrix;

	bool is_modal_analysis_complete = false;

	modal_analysis_solver();
	~modal_analysis_solver();
	void clear_results();

	void modal_analysis_start(const nodes_list_store& model_nodes,
		const elementtri_list_store& model_trielements,
		const elementquad_list_store& model_quadelements,
		const nodecnst_list_store& node_cnst,
		std::unordered_map<int, material_data>& material_list,
		rslt_nodes_list_store& modal_result_nodes,
		rslt_elementtri_list_store& modal_result_trielements,
		rslt_elementquad_list_store& modal_result_quadelements);

private:
	const double m_pi = 3.14159265358979323846;
	const double epsilon = 0.000001;
	// const bool print_matrix = true;

	Stopwatch_events stopwatch;
	std::stringstream stopwatch_elapsed_str;

	
	void get_global_stiffness_and_mass_matrix(Eigen::MatrixXd& globalStiffnessMatrix,
		Eigen::MatrixXd& globalMassMatrix,
		const elementtri_list_store& model_trielements,
		const elementquad_list_store& model_quadelements,
		std::unordered_map<int, material_data>& material_list);


	void get_global_dof_matrix(Eigen::VectorXi& globalDOFMatrix,
		const nodes_list_store& model_nodes,
		const nodecnst_list_store& node_cnst,
		const int& numDOF,
		int& reducedDOF);


	void get_reduced_global_matrices(Eigen::MatrixXd& reduced_globalStiffnessMatrix,
		Eigen::MatrixXd& reduced_globalMassMatrix,
		const Eigen::MatrixXd& globalStiffnessMatrix,
		const Eigen::MatrixXd& globalMassMatrix,
		const Eigen::VectorXi& globalDOFMatrix,
		const int& numDOF,
		const int& reducedDOF);



	void filter_eigenvalues_eigenvectors(Eigen::VectorXd& eigenvalues,
		Eigen::MatrixXd& eigenvectors);


	void sort_eigen_values_vectors(Eigen::VectorXd& eigenvalues,
		Eigen::MatrixXd& eigenvectors,
		const int& m_size);


	void normalize_eigen_vectors(Eigen::MatrixXd& eigenvectors);


	void get_globalized_eigen_vector_matrix(Eigen::MatrixXd& global_eigenvectors,
		const Eigen::MatrixXd& reduced_eigenvectors,
		const Eigen::VectorXi& globalDOFMatrix,
		const int& numDOF,
		const int& reducedDOF);


	void get_modal_participation_factor(Eigen::VectorXd& participation_factor,
		const Eigen::MatrixXd& globalPointMassMatrix,
		const Eigen::MatrixXd& global_eigenvectors,
		const int& numDOF,
		const int& reducedDOF);


	void map_modal_analysis_results(const nodes_list_store& model_nodes,
		const elementtri_list_store& model_trielements,
		const elementquad_list_store& model_quadelements,
		rslt_nodes_list_store& modal_result_nodes,
		rslt_elementtri_list_store& modal_result_trielements,
		rslt_elementquad_list_store& modal_result_quadelements);


	void get_modal_matrices(Eigen::VectorXd& reduced_modalMass,
		Eigen::VectorXd& reduced_modalStiff,
		const Eigen::MatrixXd& reduced_eigenvectors,
		const Eigen::MatrixXd& reduced_globalMassMatrix,
		const Eigen::MatrixXd& reduced_globalStiffnessMatrix,
		const int& reducedDOF);

};



