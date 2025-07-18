#include "geom_store.h"

geom_store::geom_store()
{
	// Empty Constructor
}

geom_store::~geom_store()
{
	// Empty Destructor
}

void geom_store::init(modal_analysis_window* modal_solver_window,
	pulse_analysis_window* pulse_solver_window,
	options_window* op_window,
	node_load_window* nd_load_window,
	constraint_window* nd_cnst_window,
	inlcondition_window* nd_inlcond_window,
	material_window* mat_window)
{
	// Initialize
	// Initialize the geometry parameters
	geom_param.init();

	// Intialize the selection rectangle
	selection_rectangle.init(&geom_param);

	is_geometry_set = false;

	// Initialize the solvers
	modal_solver.clear_results();
	pulse_solver.clear_results();


	// Add the window pointers
	this->op_window = op_window; // Option window
	this->nd_load_window = nd_load_window; // Node Load window
	this->nd_cnst_window = nd_cnst_window; // Node constraint window
	this->mat_window = mat_window; // Material window
	this->nd_inlcond_window = nd_inlcond_window; // Node initial condition window

	// Add the solver window pointers
	this->modal_solver_window = modal_solver_window; // Modal Analysis Solver window
	this->pulse_solver_window = pulse_solver_window; // Pulse Analysis Solver window
}

void geom_store::fini()
{
	// Deinitialize
	is_geometry_set = false;
}

void geom_store::import_model(std::ifstream& input_file)
{

	// Create stopwatch
	Stopwatch_events stopwatch;
	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Reading of raw data input started" << std::endl;

	// Read the Raw Data
	// Read the entire file into a string
	std::string file_contents((std::istreambuf_iterator<char>(input_file)),
		std::istreambuf_iterator<char>());

	// Split the string into lines
	std::istringstream iss(file_contents);
	std::string line;
	std::vector<std::string> data_lines;
	while (std::getline(iss, line))
	{
		data_lines.push_back(line);
	}

	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Lines loaded at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	int j = 0, i = 0;

	// Initialize the mesh data
	this->mesh_data.init(&geom_param);
	this->mesh_modal_rslt_data.init(&geom_param);
	this->mesh_pulse_rslt_data.init(&geom_param);

	// Initialize the model items
	this->model_nodes.init(&geom_param, &this->mesh_data);
	this->model_trielements.init(&geom_param, &this->mesh_data);
	this->model_quadelements.init(&geom_param, &this->mesh_data);

	// Node loads, initial conditions and constraints
	this->node_loads.init(&geom_param);
	this->node_inldispl.init(&geom_param);
	this->node_inlvelo.init(&geom_param);
	this->node_cnst.init(&geom_param);

	// Re-initialize the result elements
	this->modal_result_nodes.init(&geom_param, &this->mesh_modal_rslt_data);
	this->modal_result_trielements.init(&geom_param, &this->mesh_modal_rslt_data);
	this->modal_result_quadelements.init(&geom_param, &this->mesh_modal_rslt_data);

	this->pulse_result_nodes.init(&geom_param, &this->mesh_pulse_rslt_data);
	this->pulse_result_trielements.init(&geom_param, &this->mesh_pulse_rslt_data);
	this->pulse_result_quadelements.init(&geom_param, &this->mesh_pulse_rslt_data);

	// Re-initialized the analysis window
	this->modal_solver_window->init();
	this->pulse_solver_window->init();

	// Re-Initialize the solver
	modal_solver.clear_results();
	pulse_solver.clear_results();

	int node_count = 0;


	// Set the initial conditions

	this->node_inldispl.set_zero_condition(0);
	this->node_inlvelo.set_zero_condition(1);
	this->node_loads.set_zero_condition(0);


	//________________________________________ Create the model

	//Node Point list
	std::vector<glm::vec3> node_pts_list;
	bool is_material_exists = false;

	j = 0;
	// Process the lines
	while (j < data_lines.size())
	{
		std::istringstream iss(data_lines[j]);

		std::string inpt_type;
		char comma;
		iss >> inpt_type;

		if (inpt_type == "*NODE")
		{
			// Nodes
			while (j < data_lines.size())
			{
				std::istringstream nodeIss(data_lines[j + 1]);

				// Vector to store the split values
				std::vector<std::string> splitValues;

				// Split the string by comma
				std::string token;
				while (std::getline(nodeIss, token, ','))
				{
					splitValues.push_back(token);
				}

				if (static_cast<int>(splitValues.size()) != 4)
				{
					break;
				}

				int node_id = std::stoi(splitValues[0]); // node ID
				double x = geom_parameters::roundToSixDigits(std::stod(splitValues[1])); // Node coordinate x
				double y = geom_parameters::roundToSixDigits(std::stod(splitValues[2])); // Node coordinate y
				double z = geom_parameters::roundToSixDigits(std::stod(splitValues[3])); // Node coordinate z

				glm::vec3 node_pt = glm::vec3(x, y, z);
				node_pts_list.push_back(node_pt);

				// Add the nodes
				this->model_nodes.add_node(node_id, node_pt);
				j++;
			}

			stopwatch_elapsed_str.str("");
			stopwatch_elapsed_str << stopwatch.elapsed();
			std::cout << "Nodes read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
		}

		if (inpt_type == "*ELEMENT,TYPE=S3")
		{
			// Triangle Element
			while (j < data_lines.size())
			{
				std::istringstream elementIss(data_lines[j + 1]);

				// Vector to store the split values
				std::vector<std::string> splitValues;

				// Split the string by comma
				std::string token;
				while (std::getline(elementIss, token, ','))
				{
					splitValues.push_back(token);
				}

				if (static_cast<int>(splitValues.size()) != 4)
				{
					break;
				}

				int tri_id = std::stoi(splitValues[0]); // triangle ID
				int nd1 = std::stoi(splitValues[1]); // Node id 1
				int nd2 = std::stoi(splitValues[2]); // Node id 2
				int nd3 = std::stoi(splitValues[3]); // Node id 3

				// Add the Triangle Elements
				this->model_trielements.add_elementtriangle(tri_id, &model_nodes.nodeMap[nd1], &model_nodes.nodeMap[nd2],
					&model_nodes.nodeMap[nd3]);
				j++;
			}


			stopwatch_elapsed_str.str("");
			stopwatch_elapsed_str << stopwatch.elapsed();
			std::cout << "Triangle Elements read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
		}


		if (inpt_type == "*ELEMENT,TYPE=S4")
		{
			// Quad Element
			while (j < data_lines.size())
			{
				std::istringstream elementIss(data_lines[j + 1]);

				// Vector to store the split values
				std::vector<std::string> splitValues;

				// Split the string by comma
				std::string token;
				while (std::getline(elementIss, token, ','))
				{
					splitValues.push_back(token);
				}

				if (static_cast<int>(splitValues.size()) != 5)
				{
					break;
				}

				int quad_id = std::stoi(splitValues[0]); // Quadrilateral ID
				int nd1 = std::stoi(splitValues[1]); // Node id 1
				int nd2 = std::stoi(splitValues[2]); // Node id 2
				int nd3 = std::stoi(splitValues[3]); // Node id 3
				int nd4 = std::stoi(splitValues[4]); // Node id 4

				// Add the Triangle Elements
				this->model_quadelements.add_elementquadrilateral(quad_id, &model_nodes.nodeMap[nd1], &model_nodes.nodeMap[nd2],
					&model_nodes.nodeMap[nd3], &model_nodes.nodeMap[nd4]);
				j++;
			}


			stopwatch_elapsed_str.str("");
			stopwatch_elapsed_str << stopwatch.elapsed();
			std::cout << "Quadrilateral Elements read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
		}

		if (inpt_type == "*MATERIAL_DATA")
		{
			is_material_exists = true;
			mat_window->material_list.clear(); // Clear the existing material list

			std::map<int, std::vector<int>> tri_material_map;
			std::map<int, std::vector<int>> quad_material_map;

			// Material data
			while (j < data_lines.size())
			{
				std::istringstream materialIss(data_lines[j + 1]);

				// Vector to store the split values
				std::vector<std::string> splitValues;

				// Split the string by comma
				std::string token;
				while (std::getline(materialIss, token, ','))
				{
					splitValues.push_back(token);
				}

				const int numValues = static_cast<int>(splitValues.size());

				if (numValues == 7)
				{
					material_data temp_material;

					temp_material.material_id = std::stoi(splitValues[0]); // Material ID
					temp_material.material_name = splitValues[1].substr(4); // Material name
					temp_material.material_youngsmodulus = std::stod(splitValues[2]); // material youngs modulus
					temp_material.material_shearmodulus = std::stod(splitValues[3]); // material shear modulus
					temp_material.material_density = std::stod(splitValues[4]); // material density
					temp_material.shell_thickness = std::stod(splitValues[5]); // shell thickness
					temp_material.poissons_ratio = std::stod(splitValues[6]); // poissons ratio

					// Add the Material data to material data store
					// Add to materail list
					// mat_window->material_list[temp_material.material_id] = temp_material;

					// Emplace does the following
					// If the key exists : does nothing(does not overwrite).
						// If the key doesn't exist: constructs the key-value pair in-place.
						// Avoids unnecessary copying(especially if using std::move or in - place construction).

					mat_window->material_list.emplace(temp_material.material_id, temp_material);

				}
				else if (numValues == 3)
				{
					int elm_id = std::stoi(splitValues[0]); // Element ID
					int elm_type = std::stoi(splitValues[1]); // Element type 1 = Triangle, 2 = Quadrilateral
					int material_id = std::stoi(splitValues[2]); // Material ID

					if (elm_type == 1)
					{
						// Add the material ID map to Triangle element id
						tri_material_map[material_id].push_back(elm_id);
						// this->model_trielements.update_material({ elm_id }, material_id);

					}
					else if (elm_type == 2)
					{
						// Add the material ID map to Quadrilateral element id
						quad_material_map[material_id].push_back(elm_id);

						// this->model_quadelements.update_material({elm_id}, material_id);
					}
				}
				else
				{
					// Apply all materials at once
					for (const auto& [mat_id, tri_ids] : tri_material_map)
					{
						this->model_trielements.update_material(tri_ids, mat_id);
					}

					for (const auto& [mat_id, quad_ids] : quad_material_map)
					{
						this->model_quadelements.update_material(quad_ids, mat_id);
					}

					stopwatch_elapsed_str.str("");
					stopwatch_elapsed_str << stopwatch.elapsed();
					std::cout << "Material data read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

					break;
				}

				j++;
			}
		}


		if (inpt_type == "*CONSTRAINT_DATA")
		{
			// Constraint data
			while (j < data_lines.size())
			{
				std::istringstream cnstdataIss(data_lines[j + 1]);

				// Vector to store the split values
				std::vector<std::string> splitValues;

				// Split the string by comma
				std::string token;
				while (std::getline(cnstdataIss, token, ','))
				{
					splitValues.push_back(token);
				}

				if (static_cast<int>(splitValues.size()) != 8)
				{
					break;
				}

				int cnst_node_id = std::stoi(splitValues[0]); // Constraint node id
				glm::vec3 ndcnst_loc = glm::vec3(std::stod(splitValues[1]), std::stod(splitValues[2]), std::stod(splitValues[3])); // Constraint location
				glm::vec3 ndcnst_normal = glm::vec3(std::stod(splitValues[4]), std::stod(splitValues[5]), std::stod(splitValues[6])); // Constraint normal
				int cnst_type = std::stoi(splitValues[7]); // Constraint type

				// Add the Constraint data
				this->node_cnst.add_nodeconstraint(cnst_node_id, ndcnst_loc, ndcnst_normal, cnst_type);

				j++;
			}


			stopwatch_elapsed_str.str("");
			stopwatch_elapsed_str << stopwatch.elapsed();
			std::cout << "Constraint data read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

		}

		if (inpt_type == "*LOAD_DATA")
		{

			std::map<int, load_data> nd_loadset_data;

			while (j < data_lines.size())
			{
				std::istringstream loaddataIss(data_lines[j + 1]);

				// Vector to store the split values
				std::vector<std::string> splitValues;

				// Split the string by comma
				std::string token;
				while (std::getline(loaddataIss, token, ','))
				{
					splitValues.push_back(token);
				}

				if (static_cast<int>(splitValues.size()) != 11)
				{
					break;
				}



				int load_set_id = std::stoi(splitValues[0]); // Load set id
				int load_node_ids = std::stoi(splitValues[1]); // Load node id
				glm::vec3 load_loc = glm::vec3(std::stod(splitValues[2]), std::stod(splitValues[3]), std::stod(splitValues[4])); // Load location
				glm::vec3 load_normal = glm::vec3(std::stod(splitValues[5]), std::stod(splitValues[6]), std::stod(splitValues[7])); // Load normal
				double load_value = std::stod(splitValues[8]); // Load amplitude 
				double load_start_time = std::stod(splitValues[9]); // Load start time
				double load_end_time = std::stod(splitValues[10]); // Load end time

				auto& load_entry = nd_loadset_data[load_set_id];
				load_entry.node_ids.push_back(load_node_ids);
				load_entry.load_locs.push_back(load_loc);
				load_entry.load_normals.push_back(load_normal);

				if (static_cast<int>(load_entry.node_ids.size()) == 1)
				{
					load_entry.load_value = load_value;
					load_entry.load_start_time = load_start_time;
					load_entry.load_end_time = load_end_time;

				}

				j++;
			}


			// Add to the main load storage
			for (auto& ld_m : nd_loadset_data)
			{
				load_data ld = ld_m.second;

				this->node_loads.add_loads(ld.node_ids, ld.load_locs, ld.load_normals,
					ld.load_start_time, ld.load_end_time, ld.load_value);

			}

			stopwatch_elapsed_str.str("");
			stopwatch_elapsed_str << stopwatch.elapsed();
			std::cout << "Load data read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

		}

		if (inpt_type == "*INITIAL_CONDITION_DATA")
		{

			while (j < data_lines.size())
			{
				std::istringstream inldataIss(data_lines[j + 1]);

				// Vector to store the split values
				std::vector<std::string> splitValues;

				// Split the string by comma
				std::string token;
				while (std::getline(inldataIss, token, ','))
				{
					splitValues.push_back(token);
				}

				if (static_cast<int>(splitValues.size()) != 9)
				{
					break;
				}

				int inl_node_id = std::stoi(splitValues[0]); // Constraint node id
				glm::vec3 inlcond_loc = glm::vec3(std::stod(splitValues[1]), std::stod(splitValues[2]), std::stod(splitValues[3])); // Initial condition location
				glm::vec3 inlcond_normal = glm::vec3(std::stod(splitValues[4]), std::stod(splitValues[5]), std::stod(splitValues[6])); // Initial condition normal
				double inl_amplitude_z = std::stod(splitValues[7]); // Initial condition value
				int inlcond_type = std::stoi(splitValues[8]); // Initial condition type

				// Add the Initial condition data
				if (inlcond_type == 0)
				{
					// Initial displacement
					this->node_inldispl.add_inlcondition(inl_node_id, inlcond_loc, inlcond_normal, inl_amplitude_z);

				}
				else if (inlcond_type == 1)
				{
					// Initial velocity
					this->node_inlvelo.add_inlcondition(inl_node_id, inlcond_loc, inlcond_normal, inl_amplitude_z);

				}

				j++;
			}

			stopwatch_elapsed_str.str("");
			stopwatch_elapsed_str << stopwatch.elapsed();
			std::cout << "Initial condition data read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

		}

		// Iterate line
		j++;
	}

	// Input read failed??
	if (model_nodes.node_count < 2 || (model_trielements.elementtri_count + model_quadelements.elementquad_count) < 1)
	{
		is_geometry_set = false;
		std::cerr << "Input error !!" << std::endl;
		return;
	}


	// Set the mesh point normal
	this->mesh_data.set_mesh_node_normals();

	// Set the mesh wire frame
	this->mesh_data.set_mesh_wireframe();

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Mesh wireframe created at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	// No material is assigned in the model (Add a default material)
	if (is_material_exists == false || static_cast<int>(mat_window->material_list.size()) == 0)
	{
		// add a default material to the material list
		material_data default_material;
		default_material.material_id = 0; // Get the material id
		default_material.material_name = "Default material"; //Default material name
		default_material.material_youngsmodulus = 2.07 * std::pow(10, 5); //  MPa
		default_material.material_shearmodulus = 0.80 * std::pow(10, 5); //  MPa
		default_material.material_density = 7.83 * std::pow(10, -9); // tons/mm3
		default_material.shell_thickness = 10.0; // mm
		default_material.poissons_ratio = 0.3;

		// Add to materail list
		mat_window->material_list.clear();
		mat_window->material_list[default_material.material_id] = default_material;

		// Add default material id to the elements
		std::vector<int> selected_tri_elm_ids;
		std::vector<int> selected_quad_elm_ids;

		for (auto it = this->model_trielements.elementtriMap.begin(); it != this->model_trielements.elementtriMap.end(); ++it)
		{
			selected_tri_elm_ids.push_back(it->second.tri_id);
		}

		this->model_trielements.update_material(selected_tri_elm_ids, default_material.material_id);

		for (auto it = this->model_quadelements.elementquadMap.begin(); it != this->model_quadelements.elementquadMap.end(); ++it)
		{
			selected_quad_elm_ids.push_back(it->second.quad_id);
		}

		this->model_quadelements.update_material(selected_quad_elm_ids, default_material.material_id);

	}


	// Geometry is loaded
	is_geometry_set = true;

	// Set the boundary of the geometry
	std::pair<glm::vec3, glm::vec3> result = geom_parameters::findMinMaxXY(node_pts_list);
	this->geom_param.min_b = result.first;
	this->geom_param.max_b = result.second;
	this->geom_param.geom_bound = geom_param.max_b - geom_param.min_b;

	// Set the center of the geometry
	this->geom_param.center = geom_parameters::findGeometricCenter(node_pts_list);

	// Set the geometry
	update_model_matrix();
	update_model_zoomfit();

	// Set the geometry buffers
	this->mesh_data.set_buffer();

	// Set the constraints buffer
	this->node_loads.set_buffer();
	this->node_inldispl.set_buffer();
	this->node_inlvelo.set_buffer();
	this->node_cnst.set_buffer();

	// Do Not Set the result object buffers

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Model read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
}


void geom_store::export_model(std::ofstream& output_file)
{
	Stopwatch_events stopwatch;
	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Writing of model started" << std::endl;

	// Comment on model data
	output_file << "**" << std::endl;
	output_file << "**" << std::endl;
	output_file << "**Template:  Plate Vibration" << std::endl;
	output_file << "**" << std::endl;


	// Write all the nodes
	output_file << "*NODE" << std::endl;

	for (const auto& nd_m : model_nodes.nodeMap)
	{
		// Print the node details
		const node_store nd = nd_m.second;

		output_file << nd.node_id << ",\t\t"
			<< nd.node_pt.x << ",\t\t"
			<< nd.node_pt.y << ",\t\t"
			<< nd.node_pt.z << std::endl;

	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Nodes written at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	// Write all the elements
	// Write the triangle elements
	if (static_cast<int>(model_trielements.elementtriMap.size()) != 0)
	{
		output_file << "*ELEMENT,TYPE=S3" << std::endl;

		for (const auto& elm_tri_m : model_trielements.elementtriMap)
		{
			// Print the tri element details
			const elementtri_store elm_tri = elm_tri_m.second;

			output_file << elm_tri.tri_id << ",\t\t"
				<< elm_tri.nd1->node_id << ",\t\t"
				<< elm_tri.nd2->node_id << ",\t\t"
				<< elm_tri.nd3->node_id << std::endl;

		}

	}

	// Write the quadrilateral elements
	if (static_cast<int>(model_quadelements.elementquadMap.size()) != 0)
	{
		output_file << "*ELEMENT,TYPE=S4" << std::endl;

		for (const auto& elm_quad_m : model_quadelements.elementquadMap)
		{
			// Print the quad element details
			const elementquad_store elm_quad = elm_quad_m.second;

			output_file << elm_quad.quad_id << ",\t\t"
				<< elm_quad.nd1->node_id << ",\t\t"
				<< elm_quad.nd2->node_id << ",\t\t"
				<< elm_quad.nd3->node_id << ",\t\t"
				<< elm_quad.nd4->node_id << std::endl;

		}

	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Elements written at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	// Write the material data
	output_file << "*MATERIAL_DATA" << std::endl;

	for (const auto& mat_m : mat_window->material_list)
	{
		material_data mat = mat_m.second;

		output_file << mat.material_id << ",\t\t"
			<< mat.material_name << ",\t\t"
			<< mat.material_youngsmodulus << ",\t\t"
			<< mat.material_shearmodulus << ",\t\t"
			<< mat.material_density << ",\t\t"
			<< mat.shell_thickness << ",\t\t"
			<< mat.poissons_ratio << std::endl;

	}

	if (static_cast<int>(model_trielements.elementtriMap.size()) != 0)
	{

		for (const auto& elm_tri_m : model_trielements.elementtriMap)
		{
			// Print the tri element details
			const elementtri_store elm_tri = elm_tri_m.second;

			output_file << elm_tri.tri_id << ",\t\t"
				<< "1" << ",\t\t"
				<< elm_tri.material_id << std::endl;

		}

	}

	if (static_cast<int>(model_quadelements.elementquadMap.size()) != 0)
	{

		for (const auto& elm_quad_m : model_quadelements.elementquadMap)
		{
			// Print the quad element details
			const elementquad_store elm_quad = elm_quad_m.second;

			output_file << elm_quad.quad_id << ",\t\t"
				<< "2" << ",\t\t"
				<< elm_quad.material_id << std::endl;

		}

	}


	// Write the constraint data
	output_file << "*CONSTRAINT_DATA" << std::endl;

	for (const auto& cnst_m : node_cnst.ndcnstMap)
	{
		nodecnst_data cnst_d = cnst_m.second;

		output_file << cnst_d.node_id << ",\t\t"
			<< cnst_d.ndcnst_loc.x << ",\t\t"
			<< cnst_d.ndcnst_loc.y << ",\t\t"
			<< cnst_d.ndcnst_loc.z << ",\t\t"
			<< cnst_d.ndcnst_normals.x << ",\t\t"
			<< cnst_d.ndcnst_normals.y << ",\t\t"
			<< cnst_d.ndcnst_normals.z << ",\t\t"
			<< cnst_d.constraint_type << std::endl;

	}


	// Write the load data
	output_file << "*LOAD_DATA" << std::endl;

	for (const auto& nd_load : node_loads.loadMap)
	{
		for (int i = 0; i < static_cast<int>(nd_load.node_ids.size()); i++)
		{
			output_file << nd_load.load_set_id << ",\t\t"
				<< nd_load.node_ids[i] << ",\t\t"
				<< nd_load.load_locs[i].x << ",\t\t"
				<< nd_load.load_locs[i].y << ",\t\t"
				<< nd_load.load_locs[i].z << ",\t\t"
				<< nd_load.load_normals[i].x << ",\t\t"
				<< nd_load.load_normals[i].y << ",\t\t"
				<< nd_load.load_normals[i].z << ",\t\t"
				<< nd_load.load_value << ",\t\t"
				<< nd_load.load_start_time << ",\t\t"
				<< nd_load.load_end_time << std::endl;

		}

	}


	// Write the initial condition data
	output_file << "*INITIAL_CONDITION_DATA" << std::endl;

	for (const auto& nd_inl_displ_m : node_inldispl.inlcondMap)
	{
		// Node initial displacement data
		nodeinl_condition_data nd_inl_displ = nd_inl_displ_m.second;

		output_file << nd_inl_displ.node_id << ",\t\t"
			<< nd_inl_displ.inlcond_loc.x << ",\t\t"
			<< nd_inl_displ.inlcond_loc.y << ",\t\t"
			<< nd_inl_displ.inlcond_loc.z << ",\t\t"
			<< nd_inl_displ.inlcond_normals.x << ",\t\t"
			<< nd_inl_displ.inlcond_normals.y << ",\t\t"
			<< nd_inl_displ.inlcond_normals.z << ",\t\t"
			<< nd_inl_displ.inl_amplitude_z << ",\t\t"
			<< "0" << std::endl;

	}

	for (const auto& nd_inl_velo_m : node_inlvelo.inlcondMap)
	{
		// Node initial velocity data
		nodeinl_condition_data nd_inl_velo = nd_inl_velo_m.second;

		output_file << nd_inl_velo.node_id << ",\t\t"
			<< nd_inl_velo.inlcond_loc.x << ",\t\t"
			<< nd_inl_velo.inlcond_loc.y << ",\t\t"
			<< nd_inl_velo.inlcond_loc.z << ",\t\t"
			<< nd_inl_velo.inlcond_normals.x << ",\t\t"
			<< nd_inl_velo.inlcond_normals.y << ",\t\t"
			<< nd_inl_velo.inlcond_normals.z << ",\t\t"
			<< nd_inl_velo.inl_amplitude_z << ",\t\t"
			<< "1" << std::endl;

	}


	output_file << "*****" << std::endl;

	std::cout << "Model written " << std::endl;


}



void geom_store::update_WindowDimension(const int& window_width, const int& window_height)
{
	// Update the window dimension
	this->geom_param.window_width = window_width;
	this->geom_param.window_height = window_height;

	if (is_geometry_set == true)
	{
		// Update the model matrix
		update_model_matrix();
		// !! Zoom to fit operation during window resize is handled in mouse event class !!
	}
}


void geom_store::update_model_matrix()
{
	// Set the model matrix for the model shader
	// Find the scale of the model (with 0.9 being the maximum used)
	int max_dim = geom_param.window_width > geom_param.window_height ? geom_param.window_width : geom_param.window_height;

	double normalized_screen_width = 1.8f * (static_cast<double>(geom_param.window_width) / static_cast<double>(max_dim));
	double normalized_screen_height = 1.8f * (static_cast<double>(geom_param.window_height) / static_cast<double>(max_dim));

	// geom_param.rotateTranslation =   glm::mat4_cast(geom_param.default_transl);


	geom_param.geom_scale = std::min(normalized_screen_width / geom_param.geom_bound.x,
		normalized_screen_height / geom_param.geom_bound.y);

	// Translation
	glm::vec3 geom_translation = glm::vec3(-1.0f * (geom_param.max_b.x + geom_param.min_b.x) * 0.5f * geom_param.geom_scale,
		-1.0f * (geom_param.max_b.y + geom_param.min_b.y) * 0.5f * geom_param.geom_scale,
		0.0f);

	glm::mat4 g_transl = glm::translate(glm::mat4(1.0f), geom_translation);

	geom_param.modelMatrix = g_transl * glm::scale(glm::mat4(1.0f), glm::vec3(static_cast<float>(geom_param.geom_scale)));

	// Update the model matrix
	mesh_data.update_opengl_uniforms(true, false, true);

	//___________________
	node_loads.update_geometry_matrices(true, false, true);
	node_inldispl.update_geometry_matrices(true, false, true);
	node_inlvelo.update_geometry_matrices(true, false, true);
	node_cnst.update_geometry_matrices(true, false, true);

	// Update the analysis result objects
	mesh_modal_rslt_data.update_opengl_uniforms(true, false, true);
	mesh_pulse_rslt_data.update_opengl_uniforms(true, false, true);

}

void geom_store::update_model_zoomfit()
{
	if (is_geometry_set == false)
		return;

	// Set the pan translation matrix
	geom_param.panTranslation = glm::mat4(1.0f);

	// Rotation Matrix
	// geom_param.rotateTranslation = glm::mat4( glm::mat4_cast(0.4402697668541200f, 0.8215545196058330f, 0.2968766167094340f, -0.2075451231915790f));

	// Set the zoom scale
	geom_param.zoom_scale = 1.0f;

	// Update the zoom scale and pan translation
	mesh_data.update_opengl_uniforms(false, true, false);

	//___________________
	node_loads.update_geometry_matrices(false, true, false);
	node_inldispl.update_geometry_matrices(false, true, false);
	node_inlvelo.update_geometry_matrices(false, true, false);
	node_cnst.update_geometry_matrices(false, true, false);

	// Update the analysis result objects
	mesh_modal_rslt_data.update_opengl_uniforms(false, true, false);
	mesh_pulse_rslt_data.update_opengl_uniforms(false, true, false);

}

void geom_store::update_model_pan(glm::vec2& transl)
{
	if (is_geometry_set == false)
		return;

	// Pan the geometry
	geom_param.panTranslation = glm::mat4(1.0f);

	geom_param.panTranslation[0][3] = -1.0f * transl.x;
	geom_param.panTranslation[1][3] = transl.y;

	// Update the pan translation
	mesh_data.update_opengl_uniforms(false, true, false);

	//___________________
	node_loads.update_geometry_matrices(false, true, false);
	node_inldispl.update_geometry_matrices(false, true, false);
	node_inlvelo.update_geometry_matrices(false, true, false);
	node_cnst.update_geometry_matrices(false, true, false);

	// Update the analysis result objects
	mesh_modal_rslt_data.update_opengl_uniforms(false, true, false);
	mesh_pulse_rslt_data.update_opengl_uniforms(false, true, false);

}

void geom_store::update_model_rotate(glm::mat4& rotation_m)
{
	if (is_geometry_set == false)
		return;

	// Rotate the geometry
	geom_param.rotateTranslation = rotation_m;

	// Update the rotate translation
	mesh_data.update_opengl_uniforms(false, true, false);

	//___________________
	node_loads.update_geometry_matrices(false, true, false);
	node_inldispl.update_geometry_matrices(false, true, false);
	node_inlvelo.update_geometry_matrices(false, true, false);
	node_cnst.update_geometry_matrices(false, true, false);

	// Update the analysis result objects
	mesh_modal_rslt_data.update_opengl_uniforms(false, true, false);
	mesh_pulse_rslt_data.update_opengl_uniforms(false, true, false);

}


void geom_store::update_model_zoom(double& z_scale)
{
	if (is_geometry_set == false)
		return;

	// Zoom the geometry
	geom_param.zoom_scale = z_scale;

	// Update the Zoom
	mesh_data.update_opengl_uniforms(false, true, false);

	//___________________
	node_loads.update_geometry_matrices(false, true, false);
	node_inldispl.update_geometry_matrices(false, true, false);
	node_inlvelo.update_geometry_matrices(false, true, false);
	node_cnst.update_geometry_matrices(false, true, false);

	// Update the analysis result objects
	mesh_modal_rslt_data.update_opengl_uniforms(false, true, false);
	mesh_pulse_rslt_data.update_opengl_uniforms(false, true, false);

}

void geom_store::update_model_transperency(bool is_transparent)
{
	if (is_geometry_set == false)
		return;

	if (is_transparent == true)
	{
		// Set the transparency value
		geom_param.geom_transparency = 0.2f;
	}
	else
	{
		// remove transparency
		geom_param.geom_transparency = 1.0f;
	}

	// Update the model transparency
	mesh_data.update_opengl_uniforms(false, false, true);

	//___________________
	node_loads.update_geometry_matrices(false, false, true);
	node_inldispl.update_geometry_matrices(false, false, true);
	node_inlvelo.update_geometry_matrices(false, false, true);
	node_cnst.update_geometry_matrices(false, false, true);

	// Donot update result elements transparency

}

void geom_store::update_selection_rectangle(const glm::vec2& o_pt, const glm::vec2& c_pt,
	const bool& is_paint, const bool& is_select, const bool& is_rightbutton)
{
	// Draw the selection rectangle
	selection_rectangle.update_selection_rectangle(o_pt, c_pt, is_paint);

	// Selection commence (mouse button release)
	if (is_paint == false && is_select == true)
	{
		// Node Initial condition Window
		if (nd_inlcond_window->is_show_window == true)
		{
			// Selected Node Index
			std::vector<int> selected_node_ids = model_nodes.is_node_selected(o_pt, c_pt);
			nd_inlcond_window->add_to_node_list(selected_node_ids, is_rightbutton);
		}

		// Node Load Window
		if (nd_load_window->is_show_window == true)
		{
			// Selected Node Index
			std::vector<int> selected_node_ids = model_nodes.is_node_selected(o_pt, c_pt);
			nd_load_window->add_to_node_list(selected_node_ids, is_rightbutton);
		}

		// Node constraint Window
		if (nd_cnst_window->is_show_window == true)
		{
			// Selected Node Index
			std::vector<int> selected_node_ids = model_nodes.is_node_selected(o_pt, c_pt);
			nd_cnst_window->add_to_node_list(selected_node_ids, is_rightbutton);
		}

		// Material Assignment Window
		if (mat_window->is_show_window == true)
		{
			// Selected Element Index
			std::vector<int> selected_tri_elm_ids = model_trielements.is_tri_selected(o_pt, c_pt);
			std::vector<int> selected_quad_elm_ids = model_quadelements.is_quad_selected(o_pt, c_pt);

			mat_window->add_to_element_list(selected_tri_elm_ids, selected_quad_elm_ids, is_rightbutton);
		}

	}

}


void geom_store::paint_geometry()
{

	if (is_geometry_set == false)
		return;

	// Clean the back buffer and assign the new color to it
	glClear(GL_COLOR_BUFFER_BIT);

	// Paint the model
	paint_model();

	// Paint the results
	paint_model_results();

}

void geom_store::paint_model()
{
	if (modal_solver_window->is_show_window == true ||
		pulse_solver_window->is_show_window == true)
	{
		if (modal_solver_window->is_show_window == true &&
			modal_solver.is_modal_analysis_complete == true &&
			modal_solver_window->show_undeformed_model == false)
		{
			// Modal Analysis complete, window open and user turned off model view
			return;
		}
		//________________________________________________________________________________________
		if (pulse_solver_window->is_show_window == true && pulse_solver.is_pulse_analysis_complete == true &&
			pulse_solver_window->show_undeformed_model == false)
		{
			// Pulse analysis complete, window open and user turned off model view
			return;
		}
	}

	//______________________________________________
	// Paint the model
	if (op_window->is_show_modelelements == true)
	{
		// Show the model elements
		mesh_data.paint_static_mesh();

	}


	if (mat_window->is_show_window == true)
	{
		// Material assignment window is open
		paint_material_assign_operation();
	}


	// Paint the wiremesh
	if (op_window->is_show_modeledeges == true)
	{
		// Show the model edges
		mesh_data.paint_static_mesh_boundaries();
	}

	if (op_window->is_show_modelnodes == true)
	{
		// Show the model nodes
		mesh_data.paint_static_mesh_points();
	}



	if (op_window->is_show_meshnormals == true)
	{
		// Show the mesh normals
		mesh_data.paint_mesh_normals();
	}


	if (op_window->is_show_inlcondition == true)
	{
		// Show the node initial condition
		// Initial Displacement
		glPointSize(geom_param.selected_point_size);
		glLineWidth(geom_param.selected_line_width);

		node_inldispl.paint_inlcond();

		// Initial Velocity
		node_inlvelo.paint_inlcond();


		glPointSize(geom_param.point_size);
		glLineWidth(geom_param.line_width);
	}

	if (op_window->is_show_loads == true)
	{
		// Show the node Loads
		glLineWidth(geom_param.selected_line_width);

		node_loads.paint_loads();

		glLineWidth(geom_param.line_width);
	}

	if (op_window->is_show_constraint == true)
	{
		// Show the node Constraints
		glLineWidth(geom_param.selected_line_width);

		node_cnst.paint_constraint();

		glLineWidth(geom_param.line_width);
	}



	if (nd_inlcond_window->is_show_window == true)
	{
		// Initial condition window open
		paint_node_inlcond_operation();

	}

	if (nd_load_window->is_show_window == true)
	{
		// Node load window is open
		paint_node_load_operation();
	}


	if (nd_cnst_window->is_show_window == true)
	{
		// Node constraint window is open
		paint_node_constraint_operation();
	}


	if (nd_inlcond_window->is_show_window == true || nd_load_window->is_show_window == true ||
		nd_cnst_window->is_show_window == true || mat_window->is_show_window == true)
	{
		// Paint the selection rectangle (last)
		selection_rectangle.paint_selection_rectangle();
	}


}


void geom_store::paint_model_results()
{
	// Paint the results
	// Modal Analysis 
	paint_modal_analysis_results();

	// Pulse Analysis
	paint_pulse_analysis_results();

}


void geom_store::paint_modal_analysis_results()
{
	// Paint the modal analysis results
	// Closing sequence for the modal analysis window
	if (modal_solver_window->execute_modal_close == true)
	{
		// Execute the close sequence
		if (modal_solver.is_modal_analysis_complete == true)
		{
			// Pulse response analysis is complete
			update_model_transperency(false);
		}

		modal_solver_window->execute_modal_close = false;
	}

	// Check whether the modal analysis solver window is open or not
	if (modal_solver_window->is_show_window == false)
	{
		return;
	}

	// Paint the modal analysis results
	if (modal_solver.is_modal_analysis_complete == true)
	{
		// Change the buffer depending on the selected mode
		if (modal_solver_window->is_mode_selection_changed == true)
		{
			// Update the Drawing objects buffers (Depends on the selected)
			// mesh_modal_rslt_data.update_buffer(modal_solver_window->selected_modal_option);
			
			modal_solver_window->is_mode_selection_changed = false;
		}

		modal_result_nodes.update_modal_response(modal_solver_window->selected_modal_option, modal_solver_window->deformation_scale,
			std::abs(modal_solver_window->normailzed_defomation_scale));

		// Update the deflection scale
		geom_param.normalized_defl_scale = std::abs(modal_solver_window->normailzed_defomation_scale);
		geom_param.defl_scale = modal_solver_window->deformation_scale;

		// Update the deflection scale
		// mesh_modal_rslt_data.update_opengl_uniforms(false, false, false, false, false, true);

		// ______________________________________________________________________________________

		if (modal_solver_window->show_result_quads == true)
		{
			// Paint the modal tris/ quads 
			mesh_modal_rslt_data.paint_dynamic_mesh();

		}

		if (modal_solver_window->show_result_lines == true)
		{
			// Paint the modal lines (mesh boundaries)
			glLineWidth(geom_param.selected_line_width);

			mesh_modal_rslt_data.paint_dynamic_mesh_boundaries();

			glLineWidth(geom_param.line_width);
		}

		if (modal_solver_window->show_result_nodes == true)
		{
			// Paint the modal nodes
			// mesh_modal_rslt_data.paint_points();
		}
	}

	// Open sequence for the modal analysis window
	if (modal_solver_window->execute_modal_open == true)
	{
		// Execute the open sequence
		if (modal_solver.is_modal_analysis_complete == true)
		{
			// update the modal window list box
			modal_solver_window->mode_result_str = modal_solver.mode_result_str;

			// Set the buffer
			modal_solver_window->is_mode_selection_changed = true;

			// Modal analysis is already complete so set the transparency for the model
			update_model_transperency(true);
		}
		modal_solver_window->execute_modal_open = false;
	}

	// Modal Analysis 
	if (modal_solver_window->execute_modal_analysis == true)
	{
		// reset the result mesh data
		mesh_modal_rslt_data.clear_mesh();
		mesh_pulse_rslt_data.clear_mesh();

		// reset the frequency response and pulse response solution
		pulse_solver.clear_results();

		// Execute the Modal Analysis
		modal_solver.modal_analysis_start(model_nodes,
			model_trielements,
			model_quadelements,
			node_cnst,
			mat_window->material_list,
			modal_result_nodes,
			modal_result_trielements,
			modal_result_quadelements);


		// Check whether the modal analysis is complete or not
		if (modal_solver.is_modal_analysis_complete == true)
		{
			// update the modal window list box
			modal_solver_window->mode_result_str = modal_solver.mode_result_str;

			mesh_modal_rslt_data.set_mesh_wireframe();

			// Set the buffer (nodes, mesh boundaries, tria/ quad)
			mesh_modal_rslt_data.set_buffer(); // Set the node buffer

			std::cout << "Modal Analysis Complete" << std::endl;

			modal_solver_window->is_mode_selection_changed = true;

			// Modal analysis is already complete so set the transparency for the model
			update_model_transperency(true);
		}


		modal_solver_window->execute_modal_analysis = false;
	}

}


void geom_store::paint_pulse_analysis_results()
{
	//// Paint the pulse analysis results
	//// Check closing sequence for Pulse response analysis window
	//if (pulse_solver_window->execute_pulse_close == true)
	//{
	//	// Execute the close sequence
	//	if (pulse_solver.is_pulse_analysis_complete == true)
	//	{
	//		// Pulse response analysis is complete
	//		update_model_transperency(false);
	//	}

	//	pulse_solver_window->execute_pulse_close = false;
	//}

	//// Check whether the modal analysis solver window is open or not
	//if (pulse_solver_window->is_show_window == false)
	//{
	//	return;
	//}


	//// Paint the pulse analysis result
	//if (pulse_solver.is_pulse_analysis_complete == true)
	//{
	//	// Update the buffer of the selected
	//	mesh_pulse_rslt_data.update_buffer(pulse_solver_window->time_step);

	//	// Update the deflection scale
	//	geom_param.normalized_defl_scale = 1.0f;
	//	geom_param.defl_scale = pulse_solver_window->deformation_scale_max;

	//	// Update the deflection scale
	//	mesh_pulse_rslt_data.update_opengl_uniforms(false, false, false, false, false, true);

	//	// ______________________________________________________________________________________

	//	if (pulse_solver_window->show_result_quads == true)
	//	{
	//		// Paint the pulse quads 
	//		mesh_pulse_rslt_data.paint_triangles();
	//		mesh_pulse_rslt_data.paint_quadrilaterals();

	//	}


	//	if (pulse_solver_window->show_result_lines == true)
	//	{
	//		// Paint the pulse lines (mesh boundaries)
	//		mesh_pulse_rslt_data.paint_mesh_edges();

	//	}

	//	if (pulse_solver_window->show_result_nodes == true)
	//	{
	//		// Paint the pulse nodes
	//		mesh_pulse_rslt_data.paint_points();

	//	}

	//}


	if (pulse_solver_window->execute_pulse_open == true)
	{
		// Execute the open sequence
		if (modal_solver.is_modal_analysis_complete == false)
		{
			// Exit the window (when modal analysis is not complete)
			pulse_solver_window->is_show_window = false;
		}
		else
		{
			// Modal analysis Results
			// pulse_solver_window->number_of_modes = static_cast<int>(modal_solver.m_eigenvalues.size());
			// int modal_frequency_count = static_cast<int>(modal_solver.angular_freq_vector.size());
			pulse_solver_window->modal_first_frequency = modal_solver.angular_freq_vector.coeff(0) / (2.0 * m_pi);
			pulse_solver_window->modal_end_frequency = modal_solver.angular_freq_vector.coeff(modal_solver.number_of_modes - 1) / (2.0 * m_pi);
			pulse_solver_window->mode_result_str = modal_solver.mode_result_str;

			// Modal analysis is complete (check whether frequency response analysis is complete or not)
			if (pulse_solver.is_pulse_analysis_complete == true)
			{
				// Set the pulse response analysis result
				pulse_solver_window->time_interval_atrun = pulse_solver.time_interval;
				pulse_solver_window->time_step_count = pulse_solver.time_step_count;

				// Pulse response analysis is complete
				update_model_transperency(true);
			}

		}
		pulse_solver_window->execute_pulse_open = false;
	}

	if (pulse_solver_window->execute_pulse_analysis == true)
	{
		mesh_pulse_rslt_data.clear_mesh();

		// Execute the Pulse response Analysis
		pulse_solver.pulse_analysis_start(model_nodes,
			model_trielements,
			model_quadelements,
			node_loads,
			node_inldispl,
			node_inlvelo,
			mat_window->material_list,
			modal_solver,
			pulse_solver_window->total_simulation_time,
			pulse_solver_window->time_interval,
			pulse_solver_window->damping_ratio,
			pulse_solver_window->selected_pulse_option,
			pulse_result_nodes,
			pulse_result_trielements,
			pulse_result_quadelements);

		// Check whether the modal analysis is complete or not
		if (pulse_solver.is_pulse_analysis_complete == true)
		{
			// Set the pulse response analysis result
			pulse_solver_window->time_interval_atrun = pulse_solver.time_interval;
			pulse_solver_window->time_step_count = pulse_solver.time_step_count;

			mesh_pulse_rslt_data.set_mesh_wireframe();

			// Reset the buffers for pulse result nodes, lines and quads/ tris
			mesh_pulse_rslt_data.set_buffer();

			std::cout << "Pulse analysis complete " << std::endl;

			// Pulse response analysis is complete
			update_model_transperency(true);
		}
		pulse_solver_window->execute_pulse_analysis = false;
	}

}


void  geom_store::paint_node_load_operation()
{
	// Selection rectangle
	// selection_rectangle.paint_selection_rectangle();

	// Paint the selected nodes
	if (nd_load_window->is_selected_count == true)
	{
		glPointSize(geom_param.selected_point_size);
		mesh_data.paint_selected_points();

		// model_nodes.paint_selected_model_nodes();
		glPointSize(geom_param.point_size);
	}

	// Check whether the selection changed
	if (nd_load_window->is_selection_changed == true)
	{
		mesh_data.add_selected_points(nd_load_window->selected_nodes);
		nd_load_window->is_selection_changed = false;
	}

	// Apply the Node load
	if (nd_load_window->apply_nodal_load == true)
	{
		double load_amplitude = nd_load_window->load_amplitude; // load amplitude
		double load_start_time = nd_load_window->load_start_time; // load start time
		double load_end_time = nd_load_window->load_end_time; // load end time

		std::vector<glm::vec3> load_locs;
		std::vector<glm::vec3> load_normals;

		for (int& id : nd_load_window->selected_nodes)
		{
			// Add to the load location
			load_locs.push_back(model_nodes.nodeMap[id].node_pt);

			// Add to the load normal
			load_normals.push_back(mesh_data.get_mesh_node_normals(model_nodes.nodeMap[id].node_id));

		}

		// Add the loads
		node_loads.add_loads(nd_load_window->selected_nodes, load_locs, load_normals,
			load_start_time, load_end_time, load_amplitude);

		node_loads.set_buffer();

		// Load application ends
		nd_load_window->apply_nodal_load = false;

		// Remove the selection
		nd_load_window->selected_nodes.clear();
		nd_load_window->is_selection_changed = true;
	}

	// Delete all the Node load
	if (nd_load_window->delete_nodal_load == true)
	{
		for (int& id : nd_load_window->selected_nodes)
		{
			// Delete the loads
			node_loads.delete_load(id);
		}

		node_loads.set_buffer();

		// Load delete ends
		nd_load_window->delete_nodal_load = false;

		// Remove the selection
		nd_load_window->selected_nodes.clear();
		nd_load_window->is_selection_changed = true;
	}

}


void geom_store::paint_node_inlcond_operation()
{
	// Paint the node initial condition pre processing
		// Selection rectangle
	// selection_rectangle.paint_selection_rectangle();

	// Paint the selected nodes
	if (nd_inlcond_window->is_selected_count == true)
	{
		glPointSize(geom_param.selected_point_size);
		mesh_data.paint_selected_points();

		// model_nodes.paint_selected_model_nodes();
		glPointSize(geom_param.point_size);
	}

	// Check whether the selection changed
	if (nd_inlcond_window->is_selection_changed == true)
	{
		mesh_data.add_selected_points(nd_inlcond_window->selected_nodes);
		nd_inlcond_window->is_selection_changed = false;
	}

	// Apply the Node Initial Condition
	if (nd_inlcond_window->apply_nodal_inlcond == true)
	{

		for (int& id : nd_inlcond_window->selected_nodes)
		{
			// Add the initial condition

			if (nd_inlcond_window->selected_inl_option == 0)
			{
				// Intial displacement
				double initial_displacement_z = nd_inlcond_window->initial_displacement_z; // initial displacement z

				if (std::abs(initial_displacement_z) < 0.000001)
				{
					continue;
				}

				glm::vec3 node_pt = glm::vec3(model_nodes.nodeMap[id].node_pt.x,
					model_nodes.nodeMap[id].node_pt.y,
					model_nodes.nodeMap[id].node_pt.z);

				glm::vec3 inlcond_normals = mesh_data.get_mesh_node_normals(model_nodes.nodeMap[id].node_id);

				node_inldispl.add_inlcondition(id, node_pt, inlcond_normals, initial_displacement_z);

			}
			else if (nd_inlcond_window->selected_inl_option == 1)
			{
				// Initial velocity
				double initial_velocity_z = nd_inlcond_window->initial_velocity_z; // initial velocity z

				if (std::abs(initial_velocity_z) < 0.000001)
				{
					continue;
				}

				glm::vec3 node_pt = glm::vec3(model_nodes.nodeMap[id].node_pt.x,
					model_nodes.nodeMap[id].node_pt.y,
					model_nodes.nodeMap[id].node_pt.z);

				glm::vec3 inlcond_normals = mesh_data.get_mesh_node_normals(model_nodes.nodeMap[id].node_id);

				node_inlvelo.add_inlcondition(id, node_pt, inlcond_normals, initial_velocity_z);

			}

		}


		if (nd_inlcond_window->selected_inl_option == 0)
		{
			// Reset the buffer
			node_inldispl.set_buffer();

		}
		else if (nd_inlcond_window->selected_inl_option == 1)
		{
			// Reset the buffer
			node_inlvelo.set_buffer();

		}

		// initial condition application ends
		nd_inlcond_window->apply_nodal_inlcond = false;

		// Remove the selection
		nd_inlcond_window->selected_nodes.clear();
		nd_inlcond_window->is_selection_changed = true;
	}

	// Delete all the Node initial condition
	if (nd_inlcond_window->delete_nodal_inlcond == true)
	{
		for (int& id : nd_inlcond_window->selected_nodes)
		{
			// Delete the initial condition

			if (nd_inlcond_window->selected_inl_option == 0)
			{
				// Intial displacement
				double initial_displacement_z = nd_inlcond_window->initial_displacement_z; // initial displacement z

				node_inldispl.delete_inlcondition(id);

			}
			else if (nd_inlcond_window->selected_inl_option == 1)
			{
				// Initial velocity
				double initial_velocity_z = nd_inlcond_window->initial_velocity_z; // initial velocity z

				node_inlvelo.delete_inlcondition(id);
			}
		}


		if (nd_inlcond_window->selected_inl_option == 0)
		{
			// Reset the buffer
			node_inldispl.set_buffer();

		}
		else if (nd_inlcond_window->selected_inl_option == 1)
		{
			// Reset the buffer
			node_inlvelo.set_buffer();

		}


		// Initial condition delete ends
		nd_inlcond_window->delete_nodal_inlcond = false;

		// Remove the selection
		nd_inlcond_window->selected_nodes.clear();
		nd_inlcond_window->is_selection_changed = true;
	}
}


void geom_store::paint_node_constraint_operation()
{
	// Paint the node constratint pre processing
		// Selection rectangle
	// selection_rectangle.paint_selection_rectangle();

	// Paint the selected nodes
	if (nd_cnst_window->is_selected_count == true)
	{
		glPointSize(geom_param.selected_point_size);
		mesh_data.paint_selected_points();

		// model_nodes.paint_selected_model_nodes();
		glPointSize(geom_param.point_size);
	}

	// Check whether the selection changed
	if (nd_cnst_window->is_selection_changed == true)
	{
		mesh_data.add_selected_points(nd_cnst_window->selected_nodes);
		nd_cnst_window->is_selection_changed = false;
	}

	// Apply the Node constraints
	if (nd_cnst_window->apply_nodal_constraint == true)
	{

		for (int& id : nd_cnst_window->selected_nodes)
		{
			// Add the node constraint
			glm::vec3 node_pt = glm::vec3(model_nodes.nodeMap[id].node_pt.x,
				model_nodes.nodeMap[id].node_pt.y,
				model_nodes.nodeMap[id].node_pt.z);

			glm::vec3 cnst_normals = mesh_data.get_mesh_node_normals(model_nodes.nodeMap[id].node_id);


			// Add the node constraint
			node_cnst.add_nodeconstraint(id, node_pt, cnst_normals, nd_cnst_window->constraint_selectedOptionIndex);
		}

		// Reset the buffer
		node_cnst.set_buffer();

		// constraint application ends
		nd_cnst_window->apply_nodal_constraint = false;

		// Remove the selection
		nd_cnst_window->selected_nodes.clear();
		nd_cnst_window->is_selection_changed = true;
	}

	// Delete all the Node constraints
	if (nd_cnst_window->delete_nodal_constraint == true)
	{
		for (int& id : nd_cnst_window->selected_nodes)
		{
			// Delete the node constraints

			node_cnst.delete_nodeconstraint(id);

		}

		// Reset the buffer
		node_cnst.set_buffer();

		// Initial condition delete ends
		nd_cnst_window->delete_nodal_constraint = false;

		// Remove the selection
		nd_cnst_window->selected_nodes.clear();
		nd_cnst_window->is_selection_changed = true;

	}
}


void geom_store::paint_material_assign_operation()
{
	// Paint the material assignment pre processing
		// Selection rectangle
	// selection_rectangle.paint_selection_rectangle();

	// Paint the selected elements
	if (mat_window->is_selected_count == true)
	{
		// Paint the selected mesh
		this->mesh_data.paint_selected_mesh();

	}

	// Check whether the selection changed
	if (mat_window->is_selection_changed == true)
	{
		// Add the selected tris and selected quads
		this->mesh_data.add_selected_tris(mat_window->selected_tri_elements);
		this->mesh_data.add_selected_quads(mat_window->selected_quad_elements);

		mat_window->is_selection_changed = false;
	}

	// Material deleted
	if (mat_window->execute_delete_materialid != -1)
	{
		// Delete material
		// Execute delete material id
		model_trielements.execute_delete_material(mat_window->execute_delete_materialid);
		model_quadelements.execute_delete_material(mat_window->execute_delete_materialid);

		mat_window->execute_delete_materialid = -1;

		// Remove the selection
		mat_window->selected_tri_elements.clear();
		mat_window->selected_quad_elements.clear();

		mat_window->is_selection_changed = true;
	}

	// Apply the Element properties
	if (mat_window->apply_element_properties == true)
	{
		// Apply material properties to the selected triangle elements
		int material_id = mat_window->material_list[mat_window->selected_material_option].material_id; // get the material id

		model_trielements.update_material(mat_window->selected_tri_elements, material_id);
		model_quadelements.update_material(mat_window->selected_quad_elements, material_id);

		mat_window->apply_element_properties = false;

		// Remove the selection
		mat_window->selected_tri_elements.clear();
		mat_window->selected_quad_elements.clear();

		mat_window->is_selection_changed = true;
	}

	// Paint the material ID
	this->mesh_data.paint_mesh_materialids();

}






