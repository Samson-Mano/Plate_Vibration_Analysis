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










	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal analysis complete at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


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

		// Get the 3 triangle ids
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

		// Get the 4 quadrilateral ids
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


}
