#include "elementquad_list_store.h"

elementquad_list_store::elementquad_list_store()
{
	// Empty Constructor
}

elementquad_list_store::~elementquad_list_store()
{
	// Empty Destructor
}

void elementquad_list_store::init(geom_parameters* geom_param_ptr, obj_mesh_data* mesh_data)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;
	this->mesh_data = mesh_data;


	// Clear the triangles
	elementquad_count = 0;
	elementquadMap.clear();

}

void elementquad_list_store::add_elementquadrilateral(int& quad_id, node_store* nd1, node_store* nd2, node_store* nd3, node_store* nd4)
{
	// Add the quadrilateral to the list
	elementquad_store temp_quad;
	temp_quad.quad_id = quad_id; // Quadrilateral ID
	temp_quad.nd1 = nd1;
	temp_quad.nd2 = nd2;
	temp_quad.nd3 = nd3;
	temp_quad.nd4 = nd4;

	// Check whether the node_id is already there
	if (elementquadMap.find(quad_id) != elementquadMap.end())
	{
		// Element ID already exist (do not add)
		return;
	}

	// Insert to the quadrilaterals
	elementquadMap.insert({ quad_id, temp_quad });
	elementquad_count++;

	//__________________________ Add the Quadrilaterals
	this->mesh_data->add_mesh_quads(quad_id, nd1->node_id, nd2->node_id, nd3->node_id, nd4->node_id);

}


void elementquad_list_store::update_material(const std::vector<int> selected_element_quads, const int& material_id)
{
	// Update the material ID
	for (const int& it : selected_element_quads)
	{
		elementquadMap[it].material_id = material_id;
	}

	// Update the material ID label
	this->mesh_data->update_quad_material_ids(selected_element_quads, material_id);
	
}


void elementquad_list_store::execute_delete_material(const int& del_material_id)
{
	// Update delete material
	std::vector<int> selected_element_quads; // element quads where the material is changed

	// Delete the material
	for (const auto& quad : elementquadMap)
	{
		int id = quad.first; // get the id
		if (elementquadMap[id].material_id == del_material_id)
		{
			// Delete material is removed and the material ID of that element to 0
			elementquadMap[id].material_id = 0;
			selected_element_quads.push_back(id);

		}
	}

	// Update the material ID label
	if (static_cast<int>(selected_element_quads.size()) != 0)
	{
		this->mesh_data->update_quad_material_ids(selected_element_quads, 0);
	}

}



std::vector<int> elementquad_list_store::is_quad_selected(const glm::vec2& corner_pt1, const glm::vec2& corner_pt2)
{
	// Return the node id of node which is inside the rectangle
	// Covert mouse location to screen location
	int max_dim = geom_param_ptr->window_width > geom_param_ptr->window_height ? geom_param_ptr->window_width : geom_param_ptr->window_height;

	// Selected quad list index;
	std::vector<int> selected_quad_index;

	// Transform the mouse location to openGL screen coordinates
	// Corner Point 1
	glm::vec2 screen_cpt1 = glm::vec2(2.0f * ((corner_pt1.x - (geom_param_ptr->window_width * 0.5f)) / max_dim),
		2.0f * (((geom_param_ptr->window_height * 0.5f) - corner_pt1.y) / max_dim));

	// Corner Point 2
	glm::vec2 screen_cpt2 = glm::vec2(2.0f * ((corner_pt2.x - (geom_param_ptr->window_width * 0.5f)) / max_dim),
		2.0f * (((geom_param_ptr->window_height * 0.5f) - corner_pt2.y) / max_dim));

	// Nodal location
	glm::mat4 scalingMatrix = glm::mat4(1.0) * static_cast<float>(geom_param_ptr->zoom_scale);
	scalingMatrix[3][3] = 1.0f;

	glm::mat4 viewMatrix = glm::transpose(geom_param_ptr->panTranslation) * geom_param_ptr->rotateTranslation * scalingMatrix;


	// Loop through all Quadrialaterals in map
	for (auto it = elementquadMap.begin(); it != elementquadMap.end(); ++it)
	{
		const glm::vec3& node_pt1 = it->second.nd1->node_pt; // Node pt 1
		const glm::vec3& node_pt2 = it->second.nd2->node_pt; // Node pt 2
		const glm::vec3& node_pt3 = it->second.nd3->node_pt; // Node pt 3
		const glm::vec3& node_pt4 = it->second.nd4->node_pt; // Node pt 4

		glm::vec3 md_pt_12 = geom_param_ptr->linear_interpolation3d(node_pt1, node_pt2, 0.50);
		glm::vec3 md_pt_23 = geom_param_ptr->linear_interpolation3d(node_pt2, node_pt3, 0.50);
		glm::vec3 md_pt_34 = geom_param_ptr->linear_interpolation3d(node_pt3, node_pt4, 0.50);
		glm::vec3 md_pt_41 = geom_param_ptr->linear_interpolation3d(node_pt4, node_pt1, 0.50);
		glm::vec3 quad_midpt = glm::vec3((node_pt1.x + node_pt2.x + node_pt3.x + node_pt4.x) / 4.0f,
			(node_pt1.y + node_pt2.y + node_pt3.y + node_pt4.y) / 4.0f, (node_pt1.z + node_pt2.z + node_pt3.z + node_pt4.z) / 4.0f);

		//______________________________
		glm::vec4 node_pt1_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(node_pt1.x, node_pt1.y, node_pt1.z, 1.0f);
		glm::vec4 node_pt2_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(node_pt2.x, node_pt2.y, node_pt2.z, 1.0f);
		glm::vec4 node_pt3_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(node_pt3.x, node_pt3.y, node_pt3.z, 1.0f);
		glm::vec4 node_pt4_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(node_pt4.x, node_pt4.y, node_pt4.z, 1.0f);
		glm::vec4 md_pt_12_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(md_pt_12.x, md_pt_12.y, md_pt_12.z, 1.0f);
		glm::vec4 md_pt_23_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(md_pt_23.x, md_pt_23.y, md_pt_23.z, 1.0f);
		glm::vec4 md_pt_34_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(md_pt_34.x, md_pt_34.y, md_pt_34.z, 1.0f);
		glm::vec4 md_pt_41_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(md_pt_41.x, md_pt_41.y, md_pt_41.z, 1.0f);
		glm::vec4 quad_midpt_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(quad_midpt.x, quad_midpt.y, quad_midpt.z, 1.0f);


		// Check whether the point inside a rectangle
		if (geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, node_pt1_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, node_pt2_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, node_pt3_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, node_pt4_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, md_pt_12_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, md_pt_23_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, md_pt_34_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, md_pt_41_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, quad_midpt_fp) == true)
		{
			selected_quad_index.push_back(it->first);
		}
	}

	// Return the Quad index find
	return selected_quad_index;
}





