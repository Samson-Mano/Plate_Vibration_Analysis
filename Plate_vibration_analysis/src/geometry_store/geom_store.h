#pragma once
#include "geom_parameters.h"

// File system
#include <fstream>
#include <sstream>
#include <iomanip>

// Window includes
#include "../tool_window/constraint_window.h"
#include "../tool_window/node_load_window.h"
#include "../tool_window/inlcondition_window.h"
#include "../tool_window/options_window.h"
#include "../tool_window/material_window.h"
#include "../tool_window/modal_analysis_window.h"
#include "../tool_window/pulse_analysis_window.h"

// Solver
#include "../fe_solver/modal_analysis_solver.h"
#include "../fe_solver/pulse_analysis_solver.h"

// FE Objects
#include "fe_objects/nodes_list_store.h"
// #include "fe_objects/elementline_list_store.h"
#include "fe_objects/elementtri_list_store.h"
#include "fe_objects/elementquad_list_store.h"
#include "fe_objects/nodeload_list_store.h"
#include "fe_objects/nodeinlcond_list_store.h"
#include "fe_objects/nodecnst_list_store.h"

// Geometry Objects
#include "geometry_objects/dynamic_selrectangle_store.h"
#include "geometry_objects/obj_mesh_data.h"

// FE Result Objects
#include "result_objects/rslt_nodes_list_store.h";
// #include "result_objects/rslt_elementline_list_store.h";
#include "result_objects/rslt_elementtri_list_store.h";
#include "result_objects/rslt_elementquad_list_store.h"



class geom_store
{
public: 
	const double m_pi = 3.14159265358979323846;
	bool is_geometry_set = false;

	// Main Variable to strore the geometry parameters
	geom_parameters geom_param;

	geom_store();
	~geom_store();

	void init(modal_analysis_window* modal_solver_window,
		pulse_analysis_window* pulse_solver_window,
		options_window* op_window,
		node_load_window* nd_load_window, 
		constraint_window* nd_cnst_window,
		inlcondition_window* nd_inlcond_window,
		material_window* mat_window);

	void fini();

	// Load the geometry
	void import_model(std::ifstream& input_file);
	void export_model(std::ofstream& output_file);

	// Functions to control the drawing area
	void update_WindowDimension(const int& window_width, const int& window_height);
	void update_model_matrix();
	void update_model_zoomfit();
	void update_model_pan(glm::vec2& transl);
	void update_model_rotate(glm::mat4& rotation_m);
	void update_model_zoom(double& z_scale);
	void update_model_transperency(bool is_transparent);

	// Function to paint the selection rectangle
	void update_selection_rectangle(const glm::vec2& o_pt, const glm::vec2& c_pt,
		const bool& is_paint, const bool& is_select, const bool& is_rightbutton);

	// Functions to paint the geometry and results
	void paint_geometry();
private:
	// Geometry objects
	dynamic_selrectangle_store selection_rectangle;
	obj_mesh_data mesh_data;
	obj_mesh_data mesh_modal_rslt_data;
	obj_mesh_data mesh_pulse_rslt_data;


	// FE Mesh objects
	nodes_list_store model_nodes;
	elementtri_list_store model_trielements;
	elementquad_list_store model_quadelements;

	// Material data
	material_data mat_data;

	// Node initial condition, loads & Constraints
	nodeload_list_store node_loads;
	nodeinlcond_list_store node_inldispl;
	nodeinlcond_list_store node_inlvelo;
	nodecnst_list_store node_cnst;

	// Modal analysis result 
	rslt_nodes_list_store modal_result_nodes;
	rslt_elementtri_list_store modal_result_trielements;
	rslt_elementquad_list_store modal_result_quadelements;

	// Pulse analysis result
	rslt_nodes_list_store pulse_result_nodes;
	rslt_elementtri_list_store pulse_result_trielements;
	rslt_elementquad_list_store pulse_result_quadelements;

	// Solver object
	modal_analysis_solver modal_solver;
	pulse_analysis_solver pulse_solver;

	// Window pointers
	options_window* op_window = nullptr;
	node_load_window* nd_load_window = nullptr;
	constraint_window* nd_cnst_window = nullptr;
	material_window* mat_window = nullptr;
	inlcondition_window* nd_inlcond_window = nullptr;

	// Analysis window
	modal_analysis_window* modal_solver_window = nullptr;
	pulse_analysis_window* pulse_solver_window = nullptr;

	void paint_model(); // Paint the model
	void paint_model_results(); // Paint the results

	//_____________________________________________________________________________________
	void paint_modal_analysis_results(); // Paint the modal analysis results
	void paint_pulse_analysis_results(); // Paint the pulse analysis results
	void paint_node_load_operation(); // Paint the node load window
	void paint_node_inlcond_operation(); // Paint the node initial condition window
	void paint_node_constraint_operation(); // Paint the node constraint window
	void paint_material_assign_operation(); // Paint the material assignment window


};

