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
	material_window* mat_window,
	new_model_window* md_window)
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
	this->md_window = md_window;
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

void geom_store::load_model(std::vector<std::string> data_lines)
{

	// Create stopwatch
	Stopwatch_events stopwatch;
	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Reading of raw data input started" << std::endl;

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


	// Set the loads, initial condition & constraints

	this->node_inldispl.set_zero_condition(0, 0);
	this->node_inlvelo.set_zero_condition(1, 0);
	this->node_loads.set_zero_condition(0);
	this->node_cnst.set_zero_condition(0);


	//________________________________________ Create the model

	//Node Point list
	std::vector<glm::vec3> node_pts_list;
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

	if (md_window->is_show_window == true)
	{
		// New Model Window
		if (md_window->execute_create_model == true)
		{
			// Load a model
			load_model(md_window->data_lines);

			md_window->execute_create_model = false;
		}

	}


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
	//// Paint the modal analysis results
	//// Closing sequence for the modal analysis window
	//if (modal_solver_window->execute_modal_close == true)
	//{
	//	// Execute the close sequence
	//	if (modal_solver.is_modal_analysis_complete == true)
	//	{
	//		// Pulse response analysis is complete
	//		update_model_transperency(false);
	//	}

	//	modal_solver_window->execute_modal_close = false;
	//}

	//// Check whether the modal analysis solver window is open or not
	//if (modal_solver_window->is_show_window == false)
	//{
	//	return;
	//}

	//// Paint the modal analysis results
	//if (modal_solver.is_modal_analysis_complete == true)
	//{
	//	// Change the buffer depending on the selected mode
	//	if (modal_solver_window->is_mode_selection_changed == true)
	//	{
	//		// Update the Drawing objects buffers (Depends on the selected)
	//		mesh_modal_rslt_data.update_buffer(modal_solver_window->selected_modal_option);

	//		modal_solver_window->is_mode_selection_changed = false;
	//	}

	//	// Update the deflection scale
	//	geom_param.normalized_defl_scale = std::abs(modal_solver_window->normailzed_defomation_scale);
	//	geom_param.defl_scale = modal_solver_window->deformation_scale;

	//	// Update the deflection scale
	//	mesh_modal_rslt_data.update_opengl_uniforms(false, false, false, false, false, true);

	//	// ______________________________________________________________________________________

	//	if (modal_solver_window->show_result_quads == true)
	//	{
	//		// Paint the modal tris/ quads 
	//		mesh_modal_rslt_data.paint_triangles();
	//		mesh_modal_rslt_data.paint_quadrilaterals();
	//	}

	//	if (modal_solver_window->show_result_lines == true)
	//	{
	//		// Paint the modal lines (mesh boundaries)
	//		mesh_modal_rslt_data.paint_mesh_edges();

	//	}

	//	if (modal_solver_window->show_result_nodes == true)
	//	{
	//		// Paint the modal nodes
	//		// mesh_modal_rslt_data.paint_points();
	//	}
	//}

	//// Open sequence for the modal analysis window
	//if (modal_solver_window->execute_modal_open == true)
	//{
	//	// Execute the open sequence
	//	if (modal_solver.is_modal_analysis_complete == true)
	//	{
	//		// update the modal window list box
	//		modal_solver_window->mode_result_str = modal_solver.mode_result_str;

	//		// Set the buffer
	//		modal_solver_window->is_mode_selection_changed = true;

	//		// Modal analysis is already complete so set the transparency for the model
	//		update_model_transperency(true);
	//	}
	//	modal_solver_window->execute_modal_open = false;
	//}

	//// Modal Analysis 
	//if (modal_solver_window->execute_modal_analysis == true)
	//{
	//	// reset the result mesh data
	//	mesh_modal_rslt_data.clear_mesh();
	//	mesh_pulse_rslt_data.clear_mesh();

	//	// reset the frequency response and pulse response solution
	//	pulse_solver.clear_results();

	//	// Execute the Modal Analysis
	//	modal_solver.modal_analysis_start(model_nodes,
	//		model_trielements,
	//		model_quadelements,
	//		mat_data,
	//		modal_result_nodes,
	//		modal_result_trielements,
	//		modal_result_quadelements);


	//	// Check whether the modal analysis is complete or not
	//	if (modal_solver.is_modal_analysis_complete == true)
	//	{
	//		// update the modal window list box
	//		modal_solver_window->mode_result_str = modal_solver.mode_result_str;

	//		mesh_modal_rslt_data.set_mesh_wireframe();

	//		// Set the buffer (nodes, mesh boundaries, tria/ quad)
	//		mesh_modal_rslt_data.set_buffer(); // Set the node buffer

	//		std::cout << "Modal Analysis Complete" << std::endl;

	//		modal_solver_window->is_mode_selection_changed = true;

	//		// Modal analysis is already complete so set the transparency for the model
	//		update_model_transperency(true);
	//	}


	//	modal_solver_window->execute_modal_analysis = false;
	//}

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


	//if (pulse_solver_window->execute_pulse_open == true)
	//{
	//	// Execute the open sequence
	//	if (modal_solver.is_modal_analysis_complete == false)
	//	{
	//		// Exit the window (when modal analysis is not complete)
	//		pulse_solver_window->is_show_window = false;
	//	}
	//	else
	//	{
	//		// Modal analysis Results
	//		// pulse_solver_window->number_of_modes = static_cast<int>(modal_solver.m_eigenvalues.size());
	//		pulse_solver_window->modal_first_frequency = modal_solver.angular_freq_vector.coeff(0) / (2.0 * m_pi);
	//		pulse_solver_window->modal_end_frequency = modal_solver.angular_freq_vector.coeff(modal_solver.matrix_size - 1) / (2.0 * m_pi);
	//		pulse_solver_window->mode_result_str = modal_solver.mode_result_str;

	//		// Modal analysis is complete (check whether frequency response analysis is complete or not)
	//		if (pulse_solver.is_pulse_analysis_complete == true)
	//		{
	//			// Set the pulse response analysis result
	//			pulse_solver_window->time_interval_atrun = pulse_solver.time_interval;
	//			pulse_solver_window->time_step_count = pulse_solver.time_step_count;

	//			// Pulse response analysis is complete
	//			update_model_transperency(true);
	//		}

	//	}
	//	pulse_solver_window->execute_pulse_open = false;
	//}

	//if (pulse_solver_window->execute_pulse_analysis == true)
	//{
	//	mesh_pulse_rslt_data.clear_mesh();

	//	// Execute the Pulse response Analysis
	//	pulse_solver.pulse_analysis_start(model_nodes,
	//		model_trielements,
	//		model_quadelements,
	//		node_loads,
	//		node_inldispl,
	//		node_inlvelo,
	//		mat_data,
	//		modal_solver,
	//		pulse_solver_window->total_simulation_time,
	//		pulse_solver_window->time_interval,
	//		pulse_solver_window->damping_ratio,
	//		pulse_solver_window->selected_pulse_option,
	//		pulse_result_nodes,
	//		pulse_result_trielements,
	//		pulse_result_quadelements);

	//	// Check whether the modal analysis is complete or not
	//	if (pulse_solver.is_pulse_analysis_complete == true)
	//	{
	//		// Set the pulse response analysis result
	//		pulse_solver_window->time_interval_atrun = pulse_solver.time_interval;
	//		pulse_solver_window->time_step_count = pulse_solver.time_step_count;

	//		mesh_pulse_rslt_data.set_mesh_wireframe();

	//		// Reset the buffers for pulse result nodes, lines and quads/ tris
	//		mesh_pulse_rslt_data.set_buffer();

	//		std::cout << "Pulse analysis complete " << std::endl;

	//		// Pulse response analysis is complete
	//		update_model_transperency(true);
	//	}
	//	pulse_solver_window->execute_pulse_analysis = false;
	//}

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
	//  model_trielements.paint_tri_material_id();



}






