#include "nodes_list_store.h"

nodes_list_store::nodes_list_store()
{
	// Empty constructor
}

nodes_list_store::~nodes_list_store()
{
	// Empty destructor
}

void nodes_list_store::init(geom_parameters* geom_param_ptr, obj_mesh_data* mesh_data)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;
	this->mesh_data = mesh_data;


	// Clear the nodes
	node_count = 0;
	nodeMap.clear();

}

void nodes_list_store::add_node(int& node_id, glm::vec3& node_pt)
{
	// Add the node to the list
	node_store temp_node;
	temp_node.node_id = node_id;
	temp_node.node_pt = node_pt;
	temp_node.node_color = geom_param_ptr->geom_colors.point_color;

	// Check whether the node_id is already there
	if (nodeMap.find(node_id) != nodeMap.end())
	{
		// Node ID already exist (do not add)
		return;
	}

	// Insert to the nodes
	nodeMap.insert({ node_id, temp_node });
	node_count++;

	//__________________________ Add the node points
	this->mesh_data->add_mesh_point(node_id, node_pt.x, node_pt.y, node_pt.z);


}


std::vector<int> nodes_list_store::is_node_selected(const glm::vec2& corner_pt1, const glm::vec2& corner_pt2)
{
	// Return the node id of node which is inside the rectangle
	// Covert mouse location to screen location
	int max_dim = geom_param_ptr->window_width > geom_param_ptr->window_height ? geom_param_ptr->window_width : geom_param_ptr->window_height;

	// Selected node list index;
	std::vector<int> selected_node_index;

	// Transform the mouse location to openGL screen coordinates
	// Corner Point 1
	glm::vec2 screen_cpt1 = glm::vec2(2.0f * ((corner_pt1.x - (geom_param_ptr->window_width * 0.5f)) / max_dim),
		2.0f * (((geom_param_ptr->window_height * 0.5f) - corner_pt1.y) / max_dim));

	// Corner Point 2
	glm::vec2 screen_cpt2 = glm::vec2(2.0f * ((corner_pt2.x - (geom_param_ptr->window_width * 0.5f)) / max_dim),
		2.0f * (((geom_param_ptr->window_height * 0.5f) - corner_pt2.y) / max_dim));


	// gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(node_position, 1.0);

	// Nodal location
	glm::mat4 scalingMatrix = glm::mat4(1.0) * static_cast<float>(geom_param_ptr->zoom_scale);
	scalingMatrix[3][3] = 1.0f;

	glm::mat4 viewMatrix = glm::transpose(geom_param_ptr->panTranslation) * geom_param_ptr->rotateTranslation * scalingMatrix;



	// Loop through all nodes in map
	for (auto it = nodeMap.begin(); it != nodeMap.end(); ++it)
	{
		const auto& node = it->second.node_pt;
		glm::vec4 finalPosition = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(node.x, node.y, node.z, 1.0f);

		double node_position_x = finalPosition.x;
		double node_position_y = finalPosition.y;

		// Check whether the point inside a rectangle
		if (geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, finalPosition) == true)
		{
			selected_node_index.push_back(it->first);
		}
	}

	// Return the node index find
	return selected_node_index;
}




