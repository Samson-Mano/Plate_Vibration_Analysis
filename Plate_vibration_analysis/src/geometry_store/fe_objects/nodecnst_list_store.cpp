#include "nodecnst_list_store.h"

nodecnst_list_store::nodecnst_list_store()
{
	// Empty constructor
}

nodecnst_list_store::~nodecnst_list_store()
{
	// Empty destructor
}

void nodecnst_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Set the geometry parameter for the points
	ndcnst_points.init(geom_param_ptr);
	ndcnst_lines.init(geom_param_ptr);

	ndcnstMap.clear();
	ndcnst_count = 0;

}

void nodecnst_list_store::set_zero_condition(int ndcnst_type, const int& model_type)
{
	this->ndcnst_type = ndcnst_type; // Constraint type 0 - Fixed, 1 - Pinned
	this->model_type = model_type; // Model type 0

	if (ndcnst_type == 0)
	{
		// Fixed constraint
		ndcnst_points.set_point_color(geom_param_ptr->geom_colors.inlcond_displ_color);
		ndcnst_lines.set_line_color(geom_param_ptr->geom_colors.inlcond_displ_color);

	}
	else
	{
		// Pinned constraint
		ndcnst_points.set_point_color(geom_param_ptr->geom_colors.inlcond_velo_color);
		ndcnst_lines.set_line_color(geom_param_ptr->geom_colors.inlcond_velo_color);
	}

}

void nodecnst_list_store::add_nodeconstraint(int& node_id, glm::vec3& ndcnst_loc, glm::vec3& ndcnst_normals)
{
	// Add the constraint to the particular node
	nodecnst_data temp_ndcnst_data;
	temp_ndcnst_data.node_id = node_id;
	temp_ndcnst_data.ndcnst_loc = ndcnst_loc;
	temp_ndcnst_data.ndcnst_normals = ndcnst_normals;

	// Insert the inital condition data to unordered map
	// Searching for node_id
	if (ndcnstMap.find(node_id) != ndcnstMap.end())
	{
		// Node is already have constraint
		// so remove the constraint
		ndcnstMap[node_id] = temp_ndcnst_data;

		return;
	}

	// Insert the constraint to nodes
	ndcnstMap.insert({ node_id, temp_ndcnst_data });
	ndcnst_count++;

}

void nodecnst_list_store::delete_nodeconstraint(int& node_id)
{
	// Delete the constraint in this node
	if (ndcnst_count != 0)
	{
		// Remove the constraint data to unordered map
		// Searching for node_id
		// Check there is already a constraint data in the found node
		if (ndcnstMap.find(node_id) != ndcnstMap.end())
		{
			// Node is already have constraint data
			// so remove the constraint data
			ndcnstMap.erase(node_id);

			// Update the buffer
			set_buffer();

			// adjust the constraint data count
			ndcnst_count--;
		}
	}


}

void nodecnst_list_store::set_buffer()
{
	// Reset the points based on the addition of new inl condition points
	ndcnst_points.clear_points();
	ndcnst_lines.clear_lines();


	// get the color
	glm::vec3 temp_color = glm::vec3(0);

	if (ndcnst_type == 0)
	{
		// Fixed constraint
		temp_color = geom_param_ptr->geom_colors.inlcond_displ_color;
	}
	else if (ndcnst_type == 1)
	{
		// Pinned constraint
		temp_color = geom_param_ptr->geom_colors.inlcond_velo_color;
	}

	//_________________________________________________________
	// Create the points
	int pt_id = 0;
	int ln_id = 0;

	for (auto& ndcnst_m : ndcnstMap)
	{
		nodecnst_data ndcnst = ndcnst_m.second;


		// constraint point amplitude
		double pt_amplitude = 10.0f * (geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale));

		// constraint point
		
		glm::vec3 cnst_pt_end = ndcnst.ndcnst_normals * static_cast<float>(pt_amplitude);

		// Add the end point
		ndcnst_points.add_point(pt_id, ndcnst.ndcnst_loc.x, ndcnst.ndcnst_loc.y, ndcnst.ndcnst_loc.z);

		pt_id++;

		// Add the end point
		ndcnst_points.add_point(pt_id, ndcnst.ndcnst_loc.x + cnst_pt_end.x,
			ndcnst.ndcnst_loc.y + cnst_pt_end.y, ndcnst.ndcnst_loc.z + cnst_pt_end.z);

		pt_id++;

	}

	// Add the lines
	ln_id = 0;

	for (int i = 0; i < static_cast<int>(pt_id / 2.0f); i++)
	{
		// Add the initial condition line
		point_store* pt1 = ndcnst_points.get_point((i * 2) + 0);
		point_store* pt2 = ndcnst_points.get_point((i * 2) + 1);

		glm::vec3 normal = glm::normalize(pt1->pt_coord() - pt2->pt_coord());

		ndcnst_lines.add_line(ln_id, pt1, pt2, normal);

		ln_id++;
	}



	ndcnst_points.set_buffer();
	ndcnst_lines.set_buffer();

}

void nodecnst_list_store::paint_constraint()
{
	// Paint the constraint lines
	ndcnst_lines.paint_static_lines();

}

void nodecnst_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_viewmatrix, bool set_transparency)
{
	// Update model openGL uniforms
	ndcnst_points.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, set_transparency);
	ndcnst_lines.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, set_transparency);

}
