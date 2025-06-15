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


	this->node_count = model_nodes.node_count;


	this->matrix_size = 0;


	// Create element stiffness matrix
	triCKZ_element triCKZ;
	triCKZ.init();

	for (const auto& tri_elem_m : model_trielements.elementtriMap)
	{
		elementtri_store tri_elem = tri_elem_m.second;

		// Find the 3 triangle ids
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

		if (tri_elem.tri_id == 1)
		{
			// Print the element stiffness matrix for testing

			 // Open output file
			std::ofstream outFile("tri_element_stiffness_matrix.txt");

			if (outFile.is_open())
			{
				// Define formatting: high precision, fixed-point notation, aligned columns
				Eigen::IOFormat FullPrecisionFmt(
					Eigen::FullPrecision,      // use full precision
					0,                         // don't align columns
					"\t",                      // coefficient separator (tab)
					"\n",                      // row separator
					"", "", "", "");           // prefix/suffix

				outFile << "Element Stiffness Matrix for Triangle ID " << tri_elem.tri_id << ":\n";
				outFile << trielement_stiffness_matrix.format(FullPrecisionFmt);
				outFile << "\n";

				outFile << "Element Mass Matrix for Triangle ID " << tri_elem.tri_id << ":\n";
				outFile << trielement_mass_matrix.format(FullPrecisionFmt);

				outFile.close();
				std::cout << "Stiffness matrix written to tri_element_stiffness_matrix.txt\n";



			}
			else
			{
				std::cerr << "Unable to open output file for writing.\n";
			}

		}

	}






	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal analysis complete at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


}



