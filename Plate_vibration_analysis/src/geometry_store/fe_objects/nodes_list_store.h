#pragma once
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <unordered_map>
#include "../geom_parameters.h"
#include "../geometry_objects/obj_mesh_data.h"

struct node_store
{
	int node_id = 0;
	glm::vec3 node_pt = glm::vec3(0);
	glm::vec3 node_color = glm::vec3(0);
};

class nodes_list_store
{
public:
	unsigned int node_count = 0;
	std::unordered_map<int, node_store> nodeMap; // Create an unordered_map to store nodes with ID as key
	double max_displacement = 0.0;
	double max_reaction_force = 0.0;

	nodes_list_store();
	~nodes_list_store();
	void init(geom_parameters* geom_param_ptr, obj_mesh_data* mesh_data);
	void add_node(int& node_id, glm::vec3& node_pt);

	std::vector<int> is_node_selected(const glm::vec2& corner_pt1, const glm::vec2& corner_pt2);

private:
	geom_parameters* geom_param_ptr = nullptr;
	obj_mesh_data* mesh_data = nullptr;
	
};
