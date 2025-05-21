#pragma once
#include <iostream>
#include "../geometry_store/geom_parameters.h"
#include "../../resources/ImGui/imgui.h"
#include "../../resources/ImGui/imgui_impl_glfw.h"
#include "../../resources/ImGui/imgui_impl_opengl3.h"


class constraint_window
{
public:
	bool is_show_window = false;
	bool is_selection_changed = false;
	bool is_selected_count = false;

	bool apply_nodal_constraint = false;// Apply nodal constraint
	bool delete_nodal_constraint = false; // Delete nodal constraint

	int constraint_selectedOptionIndex = 0;

	std::vector<int> selected_nodes;

	constraint_window();
	~constraint_window();
	void init(); 
	void render_window();
	void add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right);

private:


};