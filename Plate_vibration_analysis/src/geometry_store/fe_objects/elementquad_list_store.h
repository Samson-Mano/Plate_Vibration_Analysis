#pragma once
#include "elementline_list_store.h"
#include "../geometry_objects/quad_list_store.h"


struct elementquad_store
{
	int quad_id = 0; // ID of the quadrilateral element
	node_store* nd1 = nullptr; // node 1
	node_store* nd2 = nullptr; // node 2
	node_store* nd3 = nullptr; // node 3
	node_store* nd4 = nullptr; // node 4
};


class elementquad_list_store
{
public:
	unsigned int elementquad_count = 0;
	std::unordered_map<int, elementquad_store> elementquadMap; // Create an unordered_map to store Quadrilaterals with ID as key

	elementquad_list_store();
	~elementquad_list_store();
	void init(geom_parameters* geom_param_ptr, obj_mesh_data* mesh_data);
	void add_elementquadrilateral(int& quad_id, node_store* nd1, node_store* nd2, node_store* nd3, node_store* nd4);
	void add_selection_quadrilaterals(const std::vector<int>& selected_quad_element_ids);
	void update_material(const std::vector<int> selected_element_quads, const int& material_id);
	void execute_delete_material(const int& del_material_id);
	void update_material_id_labels();

	std::vector<int> is_quad_selected(const glm::vec2& corner_pt1, const glm::vec2& corner_pt2);

private:
	geom_parameters* geom_param_ptr = nullptr;
	obj_mesh_data* mesh_data = nullptr;

};
