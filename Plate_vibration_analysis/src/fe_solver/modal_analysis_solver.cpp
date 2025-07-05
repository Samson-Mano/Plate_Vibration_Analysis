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

	mode_result_str.clear();
	is_modal_analysis_complete = false;
}


void modal_analysis_solver::modal_analysis_start(const nodes_list_store& model_nodes,
	const elementtri_list_store& model_trielements,
	const elementquad_list_store& model_quadelements,
	const nodecnst_list_store& node_cnst,
	std::unordered_map<int, material_data>& material_list,
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


	// this->node_count = model_nodes.node_count;
	// this->matrix_size = 0;

	// Create a node ID map (to create a nodes as ordered and numbered from 0,1,2...n)
	int i = 0;
	for (auto& nd : model_nodes.nodeMap)
	{
		nodeid_map[nd.first] = i;
		i++;
	}

	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Node maping completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	this->numDOF = model_nodes.node_count * 6;  // 6 DOF per nodes (3 Translation + 3 Rotation)

	// Create Global Stiffness Matrix
	Eigen::MatrixXd globalStiffnessMatrix = Eigen::MatrixXd::Zero(this->numDOF, this->numDOF);
	Eigen::MatrixXd globalMassMatrix = Eigen::MatrixXd::Zero(this->numDOF, this->numDOF);

	get_global_stiffness_and_mass_matrix(globalStiffnessMatrix,
		globalMassMatrix,
		model_trielements,
		model_quadelements,
		material_list);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global stiffness and Mass matrix completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Create Global DOF Matrix

	this->reducedDOF = 0;
	this->globalDOFMatrix = Eigen::VectorXi::Zero(this->numDOF);


	get_global_dof_matrix(this->globalDOFMatrix,
		model_nodes,
		node_cnst,
		this->numDOF,
		this->reducedDOF);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global DOF completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	//____________________________________________________________________________________________________________________
	// Create Reduced Global Mass and stiffness matrix
	// Reduced Global stiffness matrix
	Eigen::MatrixXd reduced_globalStiffnessMatrix(this->reducedDOF, this->reducedDOF);
	reduced_globalStiffnessMatrix.setZero();

	// Reduced Global Mass matrix
	Eigen::MatrixXd reduced_globalMassMatrix(this->reducedDOF, this->reducedDOF);
	reduced_globalMassMatrix.setZero();


	get_reduced_global_matrices(reduced_globalStiffnessMatrix,
		reduced_globalMassMatrix,
		globalStiffnessMatrix,
		globalMassMatrix,
		globalDOFMatrix,
		numDOF,
		reducedDOF);


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global stiffness, Global Point mass matrices are reduced at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	//___________________________________________________________________________________________________________________
	// Solve the generalized eigenvalue problem: [K] * [phi] = lambda * [M] * [phi]
	Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> eigenSolver;
	eigenSolver.compute(reduced_globalStiffnessMatrix, reduced_globalMassMatrix);


	if (eigenSolver.info() != Eigen::Success)
	{
		// Eigenvalue problem failed to converge
		std::cout << "Eigenvalue problem failed to converge !!!!! " << std::endl;
		return;
	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen value problem solved at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	// Get the eigenvalues and eigenvectors_reduced
	Eigen::VectorXd eigenvalues = eigenSolver.eigenvalues().real(); // Eigenvalues
	Eigen::MatrixXd eigenvectors_reduced = eigenSolver.eigenvectors().real(); // Eigenvectors

	// sort the eigen value and eigen vector (ascending)
	sort_eigen_values_vectors(eigenvalues, eigenvectors_reduced, reducedDOF);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen values and Eigen vectors are sorted at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	// Normailize eigen vectors
	normalize_eigen_vectors(eigenvectors_reduced, reducedDOF);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen vectors are normalized at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	//____________________________________________________________________________________________________________________
	// Convert the reduced transformed eigenvectors to eigen vectors for the whole model (including the nodes with supports)
	global_eigenvectors.resize(numDOF, reducedDOF);
	global_eigenvectors.setZero();

	get_globalized_eigen_vector_matrix(global_eigenvectors, eigenvectors_reduced, globalDOFMatrix, numDOF, reducedDOF);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen vectors globalized at " << stopwatch_elapsed_str.str() << " secs" << std::endl;



	//_____________________________________________________________________________________________
	// Calculate the effective mass participation factor & Cummulative effective mass participation factor
	// effective mass participation factor = percentage of the system mass that participates in a particular mode

	Eigen::VectorXd participation_factor(reducedDOF);
	participation_factor.setZero();

	get_modal_participation_factor(participation_factor,
		globalMassMatrix,
		global_eigenvectors_transformed,
		numDOF,
		reducedDOF);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal Mass of Participation Factor completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;



	//____________________________________________________________________________________________________________________
	// Store the results

	// Clear the modal results
	number_of_modes = 0; // Number of modes
	mode_result_str.clear(); // Result string list
	m_eigenvalues.clear(); // Eigen values
	m_eigenvectors.clear(); // Eigen vectors

	// Add the eigen values and eigen vectors
	for (int i = 0; i < reducedDOF; i++)
	{
		std::vector<double> eigen_vec; // Eigen vectors of all nodes (including constrainded)

		for (int j = 0; j < numDOF; j++)
		{
			eigen_vec.push_back(global_eigenvectors_transformed.coeff(j, i));
		}

		// Add to the Eigen values storage
		m_eigenvalues.insert({ i, eigenvalues.coeff(i) });

		// Add to the Eigen vectors storage 
		m_eigenvectors.insert({ i, eigen_vec });

		// Frequency
		double nat_freq = std::sqrt(eigenvalues.coeff(i)) / (2.0 * m_pi);

		// Modal results
		std::stringstream ss, mf;
		ss << std::fixed << std::setprecision(3) << nat_freq;
		mf << std::fixed << std::setprecision(3) << participation_factor.coeff(i);

		// Add to the string list
		mode_result_str.push_back("Mode " + std::to_string(i + 1) + " = " + ss.str() + " Hz , Modal mass = " + mf.str());
	}

	number_of_modes = reducedDOF; // total number of modes


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen values/ vectors stored at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	is_modal_analysis_complete = true;


	//_____________________________________________________________________________________________

	// Add the modal analysis results to node & element
	// Clear the modal node and modal element results
	modal_result_nodes.clear_results();
	modal_result_trielements.clear_results();
	modal_result_quadelements.clear_results();

	map_modal_analysis_results(model_nodes,
		model_trielements,
		model_quadelements,
		modal_result_nodes,
		modal_result_trielements,
		modal_result_quadelements);


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal analysis results maped to nodes and elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	//____________________________________________________________________________________________________________________
	// Modal Decomposition
	// Create modal matrices
	reduced_modalMass.resize(reducedDOF);
	reduced_modalStiff.resize(reducedDOF);

	get_modal_matrices(reduced_modalMass,
		reduced_modalStiff,
		eigenvectors_reduced,
		reduced_globalMassMatrix,
		reduced_globalStiffnessMatrix,
		reducedDOF);


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal mass and stiffness storage completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	std::cout << "Modal analysis complete" << std::endl;

	// end

}


void modal_analysis_solver::get_global_stiffness_and_mass_matrix(Eigen::MatrixXd& globalStiffnessMatrix, 
	Eigen::MatrixXd& globalMassMatrix,
	const elementtri_list_store& model_trielements, 
	const elementquad_list_store& model_quadelements, 
	std::unordered_map<int, material_data>& material_list)
{


	// Create element stiffness matrix for tri shell elements
	triCKZ_element triCKZ;
	triCKZ.init();

	for (const auto& tri_elem_m : model_trielements.elementtriMap)
	{
		elementtri_store tri_elem = tri_elem_m.second;

		// Get the 3 node ids of the triangle element
		int tri_id1 = tri_elem.nd1->node_id;
		int tri_id2 = tri_elem.nd2->node_id;
		int tri_id3 = tri_elem.nd3->node_id;

		int material_id = tri_elem.material_id;

		material_data mat_data = material_list[material_id];

		triCKZ.set_triCKZ_element_stiffness_matrix(tri_elem.nd1->node_pt.x, tri_elem.nd1->node_pt.y, tri_elem.nd1->node_pt.z,
			tri_elem.nd2->node_pt.x, tri_elem.nd2->node_pt.y, tri_elem.nd2->node_pt.z,
			tri_elem.nd3->node_pt.x, tri_elem.nd3->node_pt.y, tri_elem.nd3->node_pt.z,
			mat_data.shell_thickness, mat_data.material_density,
			mat_data.material_youngsmodulus, mat_data.poissons_ratio);

		Eigen::MatrixXd trielement_stiffness_matrix = triCKZ.get_element_stiffness_matrix();
		Eigen::MatrixXd trielement_mass_matrix = triCKZ.get_element_mass_matrix();


		// Map to the global stiffness and mass matrix
		// Get the Node matrix index from the node map
		int n_index1 = this->nodeid_map[tri_id1]; // get the ordered map of the Triangle ID 1
		int n_index2 = this->nodeid_map[tri_id2]; // get the ordered map of the Triangle ID 2
		int n_index3 = this->nodeid_map[tri_id3]; // get the ordered map of the Triangle ID 3

		// Map to the global stiffness matrix
		globalStiffnessMatrix.block<6, 6>(n_index1 * 6, n_index1 * 6) += trielement_stiffness_matrix.block<6, 6>(0, 0);
		globalStiffnessMatrix.block<6, 6>(n_index1 * 6, n_index2 * 6) += trielement_stiffness_matrix.block<6, 6>(0, 6);
		globalStiffnessMatrix.block<6, 6>(n_index1 * 6, n_index3 * 6) += trielement_stiffness_matrix.block<6, 6>(0, 12);

		globalStiffnessMatrix.block<6, 6>(n_index2 * 6, n_index1 * 6) += trielement_stiffness_matrix.block<6, 6>(6, 0);
		globalStiffnessMatrix.block<6, 6>(n_index2 * 6, n_index2 * 6) += trielement_stiffness_matrix.block<6, 6>(6, 6);
		globalStiffnessMatrix.block<6, 6>(n_index2 * 6, n_index3 * 6) += trielement_stiffness_matrix.block<6, 6>(6, 12);

		globalStiffnessMatrix.block<6, 6>(n_index3 * 6, n_index1 * 6) += trielement_stiffness_matrix.block<6, 6>(12, 0);
		globalStiffnessMatrix.block<6, 6>(n_index3 * 6, n_index2 * 6) += trielement_stiffness_matrix.block<6, 6>(12, 6);
		globalStiffnessMatrix.block<6, 6>(n_index3 * 6, n_index3 * 6) += trielement_stiffness_matrix.block<6, 6>(12, 12);

		// Map to the global mass matrix
		globalMassMatrix.block<6, 6>(n_index1 * 6, n_index1 * 6) += trielement_mass_matrix.block<6, 6>(0, 0);
		globalMassMatrix.block<6, 6>(n_index1 * 6, n_index2 * 6) += trielement_mass_matrix.block<6, 6>(0, 6);
		globalMassMatrix.block<6, 6>(n_index1 * 6, n_index3 * 6) += trielement_mass_matrix.block<6, 6>(0, 12);

		globalMassMatrix.block<6, 6>(n_index2 * 6, n_index1 * 6) += trielement_mass_matrix.block<6, 6>(6, 0);
		globalMassMatrix.block<6, 6>(n_index2 * 6, n_index2 * 6) += trielement_mass_matrix.block<6, 6>(6, 6);
		globalMassMatrix.block<6, 6>(n_index2 * 6, n_index3 * 6) += trielement_mass_matrix.block<6, 6>(6, 12);

		globalMassMatrix.block<6, 6>(n_index3 * 6, n_index1 * 6) += trielement_mass_matrix.block<6, 6>(12, 0);
		globalMassMatrix.block<6, 6>(n_index3 * 6, n_index2 * 6) += trielement_mass_matrix.block<6, 6>(12, 6);
		globalMassMatrix.block<6, 6>(n_index3 * 6, n_index3 * 6) += trielement_mass_matrix.block<6, 6>(12, 12);

	}


	// Create element stiffness matrix for quad shell elements
	quadMITC4_element quadMIT4;
	quadMIT4.init();

	for (const auto& quad_elem_m : model_quadelements.elementquadMap)
	{
		elementquad_store quad_elem = quad_elem_m.second;

		// Get the 4 node ids of the quadrilateral element 
		int quad_id1 = quad_elem.nd1->node_id;
		int quad_id2 = quad_elem.nd2->node_id;
		int quad_id3 = quad_elem.nd3->node_id;
		int quad_id4 = quad_elem.nd4->node_id;

		int material_id = quad_elem.material_id;

		material_data mat_data = material_list[material_id];

		quadMIT4.set_quadMITC4_element_stiffness_matrix(quad_elem.nd1->node_pt.x, quad_elem.nd1->node_pt.y, quad_elem.nd1->node_pt.z,
			quad_elem.nd2->node_pt.x, quad_elem.nd2->node_pt.y, quad_elem.nd2->node_pt.z,
			quad_elem.nd3->node_pt.x, quad_elem.nd3->node_pt.y, quad_elem.nd3->node_pt.z,
			quad_elem.nd4->node_pt.x, quad_elem.nd4->node_pt.y, quad_elem.nd4->node_pt.z,
			mat_data.shell_thickness, mat_data.material_density,
			mat_data.material_youngsmodulus, mat_data.poissons_ratio);


		Eigen::MatrixXd quadelement_stiffness_matrix = quadMIT4.get_element_stiffness_matrix();
		Eigen::MatrixXd quadelement_mass_matrix = quadMIT4.get_element_mass_matrix();


		// Map to the global stiffness and mass matrix
		// Get the Node matrix index from the node map
		int n_index1 = this->nodeid_map[quad_id1]; // get the ordered map of the Quadrilateral ID 1
		int n_index2 = this->nodeid_map[quad_id2]; // get the ordered map of the Quadrilateral ID 2
		int n_index3 = this->nodeid_map[quad_id3]; // get the ordered map of the Quadrilateral ID 3
		int n_index4 = this->nodeid_map[quad_id4]; // get the ordered map of the Quadrilateral ID 4


		// Map to the global stiffness matrix
		globalStiffnessMatrix.block<6, 6>(n_index1 * 6, n_index1 * 6) += quadelement_stiffness_matrix.block<6, 6>(0, 0);
		globalStiffnessMatrix.block<6, 6>(n_index1 * 6, n_index2 * 6) += quadelement_stiffness_matrix.block<6, 6>(0, 6);
		globalStiffnessMatrix.block<6, 6>(n_index1 * 6, n_index3 * 6) += quadelement_stiffness_matrix.block<6, 6>(0, 12);
		globalStiffnessMatrix.block<6, 6>(n_index1 * 6, n_index4 * 6) += quadelement_stiffness_matrix.block<6, 6>(0, 18);

		globalStiffnessMatrix.block<6, 6>(n_index2 * 6, n_index1 * 6) += quadelement_stiffness_matrix.block<6, 6>(6, 0);
		globalStiffnessMatrix.block<6, 6>(n_index2 * 6, n_index2 * 6) += quadelement_stiffness_matrix.block<6, 6>(6, 6);
		globalStiffnessMatrix.block<6, 6>(n_index2 * 6, n_index3 * 6) += quadelement_stiffness_matrix.block<6, 6>(6, 12);
		globalStiffnessMatrix.block<6, 6>(n_index2 * 6, n_index4 * 6) += quadelement_stiffness_matrix.block<6, 6>(6, 18);

		globalStiffnessMatrix.block<6, 6>(n_index3 * 6, n_index1 * 6) += quadelement_stiffness_matrix.block<6, 6>(12, 0);
		globalStiffnessMatrix.block<6, 6>(n_index3 * 6, n_index2 * 6) += quadelement_stiffness_matrix.block<6, 6>(12, 6);
		globalStiffnessMatrix.block<6, 6>(n_index3 * 6, n_index3 * 6) += quadelement_stiffness_matrix.block<6, 6>(12, 12);
		globalStiffnessMatrix.block<6, 6>(n_index3 * 6, n_index4 * 6) += quadelement_stiffness_matrix.block<6, 6>(12, 18);

		globalStiffnessMatrix.block<6, 6>(n_index4 * 6, n_index1 * 6) += quadelement_stiffness_matrix.block<6, 6>(18, 0);
		globalStiffnessMatrix.block<6, 6>(n_index4 * 6, n_index2 * 6) += quadelement_stiffness_matrix.block<6, 6>(18, 6);
		globalStiffnessMatrix.block<6, 6>(n_index4 * 6, n_index3 * 6) += quadelement_stiffness_matrix.block<6, 6>(18, 12);
		globalStiffnessMatrix.block<6, 6>(n_index4 * 6, n_index4 * 6) += quadelement_stiffness_matrix.block<6, 6>(18, 18);


		// Map to the global mass matrix
		globalMassMatrix.block<6, 6>(n_index1 * 6, n_index1 * 6) += quadelement_mass_matrix.block<6, 6>(0, 0);
		globalMassMatrix.block<6, 6>(n_index1 * 6, n_index2 * 6) += quadelement_mass_matrix.block<6, 6>(0, 6);
		globalMassMatrix.block<6, 6>(n_index1 * 6, n_index3 * 6) += quadelement_mass_matrix.block<6, 6>(0, 12);
		globalMassMatrix.block<6, 6>(n_index1 * 6, n_index4 * 6) += quadelement_mass_matrix.block<6, 6>(0, 18);

		globalMassMatrix.block<6, 6>(n_index2 * 6, n_index1 * 6) += quadelement_mass_matrix.block<6, 6>(6, 0);
		globalMassMatrix.block<6, 6>(n_index2 * 6, n_index2 * 6) += quadelement_mass_matrix.block<6, 6>(6, 6);
		globalMassMatrix.block<6, 6>(n_index2 * 6, n_index3 * 6) += quadelement_mass_matrix.block<6, 6>(6, 12);
		globalMassMatrix.block<6, 6>(n_index2 * 6, n_index4 * 6) += quadelement_mass_matrix.block<6, 6>(6, 18);

		globalMassMatrix.block<6, 6>(n_index3 * 6, n_index1 * 6) += quadelement_mass_matrix.block<6, 6>(12, 0);
		globalMassMatrix.block<6, 6>(n_index3 * 6, n_index2 * 6) += quadelement_mass_matrix.block<6, 6>(12, 6);
		globalMassMatrix.block<6, 6>(n_index3 * 6, n_index3 * 6) += quadelement_mass_matrix.block<6, 6>(12, 12);
		globalMassMatrix.block<6, 6>(n_index3 * 6, n_index4 * 6) += quadelement_mass_matrix.block<6, 6>(12, 18);

		globalMassMatrix.block<6, 6>(n_index4 * 6, n_index1 * 6) += quadelement_mass_matrix.block<6, 6>(18, 0);
		globalMassMatrix.block<6, 6>(n_index4 * 6, n_index2 * 6) += quadelement_mass_matrix.block<6, 6>(18, 6);
		globalMassMatrix.block<6, 6>(n_index4 * 6, n_index3 * 6) += quadelement_mass_matrix.block<6, 6>(18, 12);
		globalMassMatrix.block<6, 6>(n_index4 * 6, n_index4 * 6) += quadelement_mass_matrix.block<6, 6>(18, 18);

	}
	// globalStiffnessMatrix & globalMassMatrix

}




void modal_analysis_solver::get_global_dof_matrix(Eigen::VectorXi& globalDOFMatrix,
	const nodes_list_store& model_nodes,
	const nodecnst_list_store& node_cnst,
	const int& numDOF,
	int& reducedDOF)
{

	// Create global DOF Matrix
	for (auto& nd_m : model_nodes.nodeMap)
	{
		// Get the node data
		node_store nd = nd_m.second;
		int n_index = this->nodeid_map[nd.node_id]; // get the ordered map of the node ID

		if (node_cnst.ndcnstMap.find(nd.node_id) != node_cnst.ndcnstMap.end())
		{
			// Nodes have constraints
			nodecnst_data cd = node_cnst.ndcnstMap.at(nd.node_id);

			if (cd.constraint_type == 0)
			{
				// Fixed End (0.0 = Fixed)

				globalDOFMatrix.coeffRef((n_index * 6) + 0) = 0; // Translation x
				globalDOFMatrix.coeffRef((n_index * 6) + 1) = 0; // Translation y
				globalDOFMatrix.coeffRef((n_index * 6) + 2) = 0; // Translation z

				globalDOFMatrix.coeffRef((n_index * 6) + 3) = 0; // Rotation x
				globalDOFMatrix.coeffRef((n_index * 6) + 4) = 0; // Rotation y
				globalDOFMatrix.coeffRef((n_index * 6) + 5) = 0; // Rotation z
			}
			else if (cd.constraint_type == 1)
			{
				// Pin end

				globalDOFMatrix.coeffRef((n_index * 6) + 0) = 0; // Translation x
				globalDOFMatrix.coeffRef((n_index * 6) + 1) = 0; // Translation y
				globalDOFMatrix.coeffRef((n_index * 6) + 2) = 0; // Translation z

				globalDOFMatrix.coeffRef((n_index * 6) + 3) = 1; // Rotation x is Free
				globalDOFMatrix.coeffRef((n_index * 6) + 4) = 1; // Rotation y is Free
				globalDOFMatrix.coeffRef((n_index * 6) + 5) = 1; // Rotation z is Free

				reducedDOF = reducedDOF + 3;
			}
		}
		else
		{
			// Nodes doesnt have Constraint (1.0 = Free)

			globalDOFMatrix.coeffRef((n_index * 6) + 0) = 1; // Translation x
			globalDOFMatrix.coeffRef((n_index * 6) + 1) = 1; // Translation y
			globalDOFMatrix.coeffRef((n_index * 6) + 2) = 1; // Translation z

			globalDOFMatrix.coeffRef((n_index * 6) + 3) = 1; // Rotation x is Free
			globalDOFMatrix.coeffRef((n_index * 6) + 4) = 1; // Rotation y is Free
			globalDOFMatrix.coeffRef((n_index * 6) + 5) = 1; // Rotation z is Free

			reducedDOF = reducedDOF + 6;
		}
	}
	// globalDOFMatrix complete

}



void modal_analysis_solver::get_reduced_global_matrices(Eigen::MatrixXd& reduced_globalStiffnessMatrix,
	Eigen::MatrixXd& reduced_globalMassMatrix,
	const Eigen::MatrixXd& globalStiffnessMatrix,
	const Eigen::MatrixXd& globalMassMatrix,
	const Eigen::VectorXi& globalDOFMatrix,
	const int& numDOF,
	const int& reducedDOF)
{

	// Curtailment of Global stiffness and Global Point mass matrix based on DOF
	// Get the reduced global stiffness matrix
	int r = 0;
	int s = 0;

	// Loop throug the Degree of freedom of indices
	for (int i = 0; i < numDOF; i++)
	{
		if (globalDOFMatrix.coeff(i) == 0)
		{
			// constrained row index, so skip
			continue;
		}
		else
		{
			s = 0;
			for (int j = 0; j < numDOF; j++)
			{
				if (globalDOFMatrix.coeff(j) == 0)
				{
					// constrained column index, so skip
					continue;
				}
				else
				{
					// Get the reduced matrices
					reduced_globalStiffnessMatrix.coeffRef(r, s) = globalStiffnessMatrix.coeff(i, j);
					reduced_globalMassMatrix.coeffRef(r, s) = globalMassMatrix.coeff(i, j);
					s++;
				}
			}
			r++;
		}
	}
	// reduction complete

}



void modal_analysis_solver::sort_eigen_values_vectors(Eigen::VectorXd& eigenvalues,
	Eigen::MatrixXd& eigenvectors,
	const int& m_size)
{
	int p = 0;
	int q = 0;
	int i = 0;

	double swap_temp = 0.0;

	// sort the eigen value and eigen vector (ascending)
	for (p = 0; p < m_size; p++)
	{
		for (q = p + 1; q < m_size; q++)
		{
			if (eigenvalues(p) > eigenvalues(q))
			{
				swap_temp = eigenvalues(p);
				eigenvalues(p) = eigenvalues(q);
				eigenvalues(q) = swap_temp;

				for (i = 0; i < m_size; i++)
				{
					swap_temp = eigenvectors(i, p);
					eigenvectors(i, p) = eigenvectors(i, q);
					eigenvectors(i, q) = swap_temp;
				}
			}
		}
	}

}


void modal_analysis_solver::normalize_eigen_vectors(Eigen::MatrixXd& eigenvectors,
	const int& m_size)
{
	// Normalize eigen vectors
	int p = 0;
	int q = 0;

	// loop throught each column
	for (p = 0; p < m_size; p++)
	{
		double max_modal_vector = 0.0;

		// Loop through each row
		for (q = 0; q < m_size; q++)
		{
			if (std::abs(eigenvectors(q, p)) > max_modal_vector)
			{
				// Max modal vector in the column (for particular mode)
				max_modal_vector = std::abs(eigenvectors(q, p));
			}
		}

		// Normalize the column using maximum modal vector
		for (q = 0; q < m_size; q++)
		{
			eigenvectors(q, p) = eigenvectors(q, p) / max_modal_vector;

			// Round the eigen vectors to 6 digit precision after normalizing
			eigenvectors(q, p) = std::round(eigenvectors(q, p) * 1000000) / 1000000;
		}
	}

}


void modal_analysis_solver::get_globalized_eigen_vector_matrix(Eigen::MatrixXd& global_eigenvectors,
	const Eigen::MatrixXd& reduced_eigenvectors,
	const Eigen::VectorXi& globalDOFMatrix,
	const int& numDOF,
	const int& reducedDOF)
{
	// Global eigen vector Matrix
	// Loop throug the Degree of freedom of indices

	// J loops through number of modes (along the column)
	for (int j = 0; j < reducedDOF; j++)
	{
		int s = 0;
		// i loops through the number of nodes (along the row)
		for (int i = 0; i < numDOF; i++)
		{
			if (globalDOFMatrix.coeff(i) == 0)
			{
				// constrained row index, so Displacement is Zero
				global_eigenvectors.coeffRef(i, j) = 0;
			}
			else
			{
				// Un constrained row index, so Displacement is Zero
				global_eigenvectors.coeffRef(i, j) = reduced_eigenvectors.coeff(s, j);
				s++;
			}
		}
	}

}




void modal_analysis_solver::get_modal_participation_factor(Eigen::VectorXd& participation_factor,
	const Eigen::MatrixXd& globalPointMassMatrix,
	const Eigen::MatrixXd& global_eigenvectors_transformed,
	const int& numDOF,
	const int& reducedDOF)
{

	// Global Participation Factor

	// Influence vector
	Eigen::VectorXd influence_vector(numDOF);
	influence_vector.setOnes();


	double temp_modal_mass = 0.0;
	double temp_distribution_factor = 0.0;
	double temp_participation_factor = 0.0;
	double total_participation_factor = 0.0;

	double total_mass = influence_vector.transpose() * globalPointMassMatrix * influence_vector;

	for (int i = 0; i < reducedDOF; i++)
	{
		// Get the nth Mode (Column)
		Eigen::VectorXd eigen_vector_i = global_eigenvectors_transformed.col(i);

		// Modal Mass
		temp_modal_mass = eigen_vector_i.transpose() * globalPointMassMatrix * eigen_vector_i;

		// Force distribution factor
		temp_distribution_factor = eigen_vector_i.transpose() * globalPointMassMatrix * influence_vector;

		// Add to the Participation factor
		temp_participation_factor = std::pow(temp_distribution_factor, 2) / temp_modal_mass;

		participation_factor.coeffRef(i) = temp_participation_factor;

		// Cummulative participation factor
		total_participation_factor += temp_participation_factor;

	}


}



void modal_analysis_solver::map_modal_analysis_results(const nodes_list_store& model_nodes,
	const elementtri_list_store& model_trielements,
	const elementquad_list_store& model_quadelements,
	rslt_nodes_list_store& modal_result_nodes,
	rslt_elementtri_list_store& modal_result_trielements,
	rslt_elementquad_list_store& modal_result_quadelements)
{

	// Map the results to modal_result_nodes

	for (auto& nd_m : model_nodes.nodeMap)
	{
		int node_id = nd_m.first;
		glm::vec3 node_pt = nd_m.second.node_pt;
		int matrix_index = nodeid_map[node_id];

		// Modal analysis results
		std::vector<glm::vec3> node_modal_displ;
		std::vector<double> node_modal_displ_magnitude;


		for (int i = 0; i < number_of_modes; i++)
		{
			// Get the mode result list
			std::vector<double> globalEigenVector = m_eigenvectors[i];

			// get the appropriate modal displacement of this particular node
			glm::vec3 modal_displ = glm::vec3(globalEigenVector[(matrix_index * 6) + 0],
				globalEigenVector[(matrix_index * 6) + 1],
				globalEigenVector[(matrix_index * 6) + 2]);

			double displ_magnitude = glm::length(modal_displ);


			// add to modal result of this node
			node_modal_displ.push_back(modal_displ);
			node_modal_displ_magnitude.push_back(std::abs(displ_magnitude));
		}

		// Create the modal analysis result node
		modal_result_nodes.add_result_node(node_id, node_pt, node_modal_displ, node_modal_displ_magnitude);
	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped to model nodes at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	//____________________________________________________________________________________________________________________
	// Map the results to modal_result_trielements and modal_result_quadelements

	// Add the modal tri element results
	for (const auto& tri_elem_m : model_trielements.elementtriMap)
	{
		elementtri_store tri_elem = tri_elem_m.second;

		// Get the 3 node ids of the triangle element
		int tri_id1 = tri_elem.nd1->node_id;
		int tri_id2 = tri_elem.nd2->node_id;
		int tri_id3 = tri_elem.nd3->node_id;

		modal_result_trielements.add_rslt_elementtriangle(tri_elem.tri_id,
			&modal_result_nodes.rslt_nodeMap[tri_id1],
			&modal_result_nodes.rslt_nodeMap[tri_id2],
			&modal_result_nodes.rslt_nodeMap[tri_id3]);

	}


	// Add the modal quad element results
	for (const auto& quad_elem_m : model_quadelements.elementquadMap)
	{
		elementquad_store quad_elem = quad_elem_m.second;

		// Get the 4 node ids of the quadrilateral element 
		int quad_id1 = quad_elem.nd1->node_id;
		int quad_id2 = quad_elem.nd2->node_id;
		int quad_id3 = quad_elem.nd3->node_id;
		int quad_id4 = quad_elem.nd4->node_id;

		modal_result_quadelements.add_rslt_elementquadrilateral(quad_elem.quad_id,
			&modal_result_nodes.rslt_nodeMap[quad_id1],
			&modal_result_nodes.rslt_nodeMap[quad_id2],
			&modal_result_nodes.rslt_nodeMap[quad_id3],
			&modal_result_nodes.rslt_nodeMap[quad_id4]);

	}
	
	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped to model elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	//____________________________________________________________________________________________________________________
	// Mapping ends

}



void modal_analysis_solver::get_modal_matrices(Eigen::VectorXd& reduced_modalMass,
	Eigen::VectorXd& reduced_modalStiff,
	const Eigen::MatrixXd& reduced_eigenvectors,
	const Eigen::MatrixXd& reduced_globalMassMatrix,
	const Eigen::MatrixXd& reduced_globalStiffnessMatrix,
	const int& reducedDOF)
{
	// Get the modal matrices
	Eigen::MatrixXd modalMassMatrix(reducedDOF, reducedDOF);
	modalMassMatrix = reduced_eigenvectors.transpose() * reduced_globalMassMatrix * reduced_eigenvectors;

	// Create modal stiffness matrix
	Eigen::MatrixXd modalStiffMatrix(reducedDOF, reducedDOF);
	modalStiffMatrix = reduced_eigenvectors.transpose() * reduced_globalStiffnessMatrix * reduced_eigenvectors;

	// Create the modal vectors
	reduced_modalMass = modalMassMatrix.diagonal();
	reduced_modalStiff = modalStiffMatrix.diagonal();

}
