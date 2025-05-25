#pragma once
#include "nodes_list_store.h"


struct elementtri_store
{
	int tri_id = 0; // ID of the triangle element
	node_store* nd1 = nullptr; // node 1
	node_store* nd2 = nullptr; // node 2
	node_store* nd3 = nullptr; // node 3

	int material_id = -1;

};


class elementtri_list_store
{
public:
	unsigned int elementtri_count = 0;
	std::unordered_map<int, elementtri_store> elementtriMap; // Create an unordered_map to store Triangles with ID as key

	elementtri_list_store();
	~elementtri_list_store();
	void init(geom_parameters* geom_param_ptr, obj_mesh_data* mesh_data);
	void add_elementtriangle(int& tri_id, node_store* nd1, node_store* nd2, node_store* nd3);

	void update_material(const std::vector<int> selected_element_tris, const int& material_id);
	void execute_delete_material(const int& del_material_id);
	// void update_material_id_labels();

	std::vector<int> is_tri_selected(const glm::vec2& corner_pt1, const glm::vec2& corner_pt2);

private:
	geom_parameters* geom_param_ptr = nullptr;
	obj_mesh_data* mesh_data = nullptr;

};
