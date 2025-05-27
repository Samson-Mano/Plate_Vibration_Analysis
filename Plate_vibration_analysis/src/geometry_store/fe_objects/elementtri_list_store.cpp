#include "elementtri_list_store.h"

elementtri_list_store::elementtri_list_store()
{
	// Empty constructor
}

elementtri_list_store::~elementtri_list_store()
{
	// Empty destructor
}

void elementtri_list_store::init(geom_parameters* geom_param_ptr, obj_mesh_data* mesh_data)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;
	this->mesh_data = mesh_data;

	// Clear the triangles
	elementtri_count = 0;
	elementtriMap.clear();
}

void elementtri_list_store::add_elementtriangle(int& tri_id, node_store* nd1, node_store* nd2, node_store* nd3)
{
	// Add the Triangle to the list
	elementtri_store temp_tri;
	temp_tri.tri_id = tri_id; // Triangle ID
	temp_tri.nd1 = nd1;
	temp_tri.nd2 = nd2;
	temp_tri.nd3 = nd3;

	// Check whether the node_id is already there
	if (elementtriMap.find(tri_id) != elementtriMap.end())
	{
		// Element ID already exist (do not add)
		return;
	}

	// Insert to the triangles
	elementtriMap.insert({ tri_id, temp_tri });
	elementtri_count++;

	//__________________________ Add the Triangle
	this->mesh_data->add_mesh_tris(tri_id, nd1->node_id, nd2->node_id, nd3->node_id);

}



void elementtri_list_store::update_material(const std::vector<int> selected_element_tris, const int& material_id)
{
	// Update the material ID
	for (const int& it : selected_element_tris)
	{
		elementtriMap[it].material_id = material_id;
	}

	// Update the material ID label
	this->mesh_data->update_tri_material_ids(selected_element_tris, material_id);

}



void elementtri_list_store::execute_delete_material(const int& del_material_id)
{
	// Update delete material
	std::vector<int> selected_element_tris; // element quads where the material is changed

	// Delete the material
	for (const auto& tri : elementtriMap)
	{
		int id = tri.first; // get the id
		if (elementtriMap[id].material_id == del_material_id)
		{
			// Delete material is removed and the material ID of that element to 0
			elementtriMap[id].material_id = 0;
			selected_element_tris.push_back(id);

		}
	}

	// Update the material ID label
	if (static_cast<int>(selected_element_tris.size()) != 0)
	{
		this->mesh_data->update_tri_material_ids(selected_element_tris, 0);
	}

}


std::vector<int> elementtri_list_store::is_tri_selected(const glm::vec2& corner_pt1, const glm::vec2& corner_pt2)
{
	// Return the node id of node which is inside the rectangle
	// Covert mouse location to screen location
	int max_dim = geom_param_ptr->window_width > geom_param_ptr->window_height ? geom_param_ptr->window_width : geom_param_ptr->window_height;

	// Selected triangle list index;
	std::vector<int> selected_tri_index;

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


	// Loop through all triangles in map
	for (auto it = elementtriMap.begin(); it != elementtriMap.end(); ++it)
	{
		const glm::vec3& node_pt1 = it->second.nd1->node_pt;
		const glm::vec3& node_pt2 = it->second.nd2->node_pt;
		const glm::vec3& node_pt3 = it->second.nd3->node_pt;
		glm::vec3 md_pt_12 = geom_param_ptr->linear_interpolation3d(node_pt1, node_pt2, 0.50);
		glm::vec3 md_pt_23 = geom_param_ptr->linear_interpolation3d(node_pt2, node_pt3, 0.50);
		glm::vec3 md_pt_31 = geom_param_ptr->linear_interpolation3d(node_pt3, node_pt1, 0.50);
		glm::vec3 tri_midpt = glm::vec3((node_pt1.x + node_pt2.x + node_pt3.x) * 0.33f, 
			(node_pt1.y + node_pt2.y + node_pt3.y) * 0.33f, (node_pt1.z + node_pt2.z + node_pt3.z) * 0.33f);

		//______________________________
		glm::vec4 node_pt1_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(node_pt1.x, node_pt1.y, node_pt1.z, 1.0f);
		glm::vec4 node_pt2_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(node_pt2.x, node_pt2.y, node_pt2.z, 1.0f);
		glm::vec4 node_pt3_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(node_pt3.x, node_pt3.y, node_pt3.z, 1.0f);
		glm::vec4 md_pt_12_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(md_pt_12.x, md_pt_12.y, md_pt_12.z, 1.0f);
		glm::vec4 md_pt_23_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(md_pt_23.x, md_pt_23.y, md_pt_23.z, 1.0f);
		glm::vec4 md_pt_31_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(md_pt_31.x, md_pt_31.y, md_pt_31.z, 1.0f);
		glm::vec2 tri_midpt_fp = geom_param_ptr->projectionMatrix * viewMatrix * geom_param_ptr->modelMatrix * glm::vec4(tri_midpt.x, tri_midpt.y, tri_midpt.z, 1.0f);



		// Check whether the point inside a rectangle
		if (geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, node_pt1_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, node_pt2_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, node_pt3_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, md_pt_12_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, md_pt_23_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, md_pt_31_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, tri_midpt_fp) == true)
		{
			selected_tri_index.push_back(it->first);
		}
	}

	// Return the tri index find
	return selected_tri_index;

}




