#pragma once
#include <iostream>
#include "../geometry_store/geom_parameters.h"
#include "../../resources/ImGui/imgui.h"
#include "../../resources/ImGui/imgui_impl_glfw.h"
#include "../../resources/ImGui/imgui_impl_opengl3.h"


class material_window
{
public:
	bool is_show_window = false;
	bool is_selection_changed = false;
	bool is_selected_count = false;
	bool apply_element_properties = false;
	int execute_delete_materialid = -1;

	int selected_material_option = 0;
	std::unordered_map<int, material_data> material_list;
	std::vector<int> selected_tri_elements;
	std::vector<int> selected_quad_elements;

	material_window();
	~material_window();
	void init();
	void render_window();
	void add_to_element_list(const std::vector<int>& selected_tri_elements, const std::vector<int>& selected_quad_elements, const bool& is_right);

private:
	int selected_list_option = 0;

	int get_unique_material_id();

};






