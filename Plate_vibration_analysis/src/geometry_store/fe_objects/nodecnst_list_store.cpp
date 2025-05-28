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

	ndcnst_points.set_point_color(geom_param_ptr->geom_colors.constraint_color);
	ndcnst_lines.set_line_color(geom_param_ptr->geom_colors.constraint_color);

	ndcnstMap.clear();
	ndcnst_count = 0;

}



void nodecnst_list_store::add_nodeconstraint(int& node_id, glm::vec3& ndcnst_loc, glm::vec3& ndcnst_normals, int& constraint_type)
{
	// Add the constraint to the particular node
	nodecnst_data temp_ndcnst_data;
	temp_ndcnst_data.node_id = node_id;
	temp_ndcnst_data.ndcnst_loc = ndcnst_loc;
	temp_ndcnst_data.ndcnst_normals = ndcnst_normals;
	temp_ndcnst_data.constraint_type = constraint_type;

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

	//_________________________________________________________
	// Create the points
	int pt_id = 0;
	int ln_id = 0;

	for (auto& ndcnst_m : ndcnstMap)
	{
		nodecnst_data ndcnst = ndcnst_m.second;

		// Constraint location & Constraint normals
		glm::vec3 ndcnst_loc = ndcnst.ndcnst_loc;
		glm::vec3 normalized_cnst_loc = ndcnst.ndcnst_normals; // Normalize the load location
		
		//___________________________
		std::pair<glm::vec3, glm::vec3> uv_orthogonal = findOrthogonalVectors(normalized_cnst_loc);



		if (ndcnst.constraint_type == 0)
		{

			// Define the neck location of the arrow head
			glm::vec3 load_arrow_neck_top = ndcnst_loc + normalized_cnst_loc *
				static_cast<float>(1.0f * (geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale)); // 0

			// Define offset distances for arrow head points
			float arrow_head_offset = 1.0f * (geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale));


			// Calculate arrow head points (Top square)
			glm::vec3 load_arrow_pt1 = load_arrow_neck_top + glm::normalize(rotateVector(uv_orthogonal.first, normalized_cnst_loc, 45.0f)) * arrow_head_offset;
			glm::vec3 load_arrow_pt2 = load_arrow_neck_top + glm::normalize(rotateVector(uv_orthogonal.first, normalized_cnst_loc, 135.0f)) * arrow_head_offset;
			glm::vec3 load_arrow_pt3 = load_arrow_neck_top + glm::normalize(rotateVector(uv_orthogonal.first, normalized_cnst_loc, 225.0f)) * arrow_head_offset;
			glm::vec3 load_arrow_pt4 = load_arrow_neck_top + glm::normalize(rotateVector(uv_orthogonal.first, normalized_cnst_loc, 315.0f)) * arrow_head_offset;

			// Calculate arrow head points (Bot square)
			glm::vec3 load_arrow_pt5 = ndcnst_loc + glm::normalize(rotateVector(uv_orthogonal.first, normalized_cnst_loc, 45.0)) * arrow_head_offset;
			glm::vec3 load_arrow_pt6 = ndcnst_loc + glm::normalize(rotateVector(uv_orthogonal.first, normalized_cnst_loc, 135.0)) * arrow_head_offset;
			glm::vec3 load_arrow_pt7 = ndcnst_loc + glm::normalize(rotateVector(uv_orthogonal.first, normalized_cnst_loc, 225.0f)) * arrow_head_offset;
			glm::vec3 load_arrow_pt8 = ndcnst_loc + glm::normalize(rotateVector(uv_orthogonal.first, normalized_cnst_loc, 315.0f)) * arrow_head_offset;


			// Fixed constraint
			ndcnst_points.add_point(pt_id, load_arrow_pt1.x, load_arrow_pt1.y, load_arrow_pt1.z);
			pt_id++;

			ndcnst_points.add_point(pt_id, load_arrow_pt2.x, load_arrow_pt2.y, load_arrow_pt2.z);
			pt_id++;

			ndcnst_points.add_point(pt_id, load_arrow_pt3.x, load_arrow_pt3.y, load_arrow_pt3.z);
			pt_id++;

			ndcnst_points.add_point(pt_id, load_arrow_pt4.x, load_arrow_pt4.y, load_arrow_pt4.z);
			pt_id++;

			ndcnst_points.add_point(pt_id, load_arrow_pt5.x, load_arrow_pt5.y, load_arrow_pt5.z);
			pt_id++;

			ndcnst_points.add_point(pt_id, load_arrow_pt6.x, load_arrow_pt6.y, load_arrow_pt6.z);
			pt_id++;

			ndcnst_points.add_point(pt_id, load_arrow_pt7.x, load_arrow_pt7.y, load_arrow_pt7.z);
			pt_id++;

			ndcnst_points.add_point(pt_id, load_arrow_pt8.x, load_arrow_pt8.y, load_arrow_pt8.z);
			pt_id++;

			point_store* pt1 = ndcnst_points.get_point(pt_id - 8);
			point_store* pt2 = ndcnst_points.get_point(pt_id - 7);
			point_store* pt3 = ndcnst_points.get_point(pt_id - 6);
			point_store* pt4 = ndcnst_points.get_point(pt_id - 5);
			point_store* pt5 = ndcnst_points.get_point(pt_id - 4);
			point_store* pt6 = ndcnst_points.get_point(pt_id - 3);
			point_store* pt7 = ndcnst_points.get_point(pt_id - 2);
			point_store* pt8 = ndcnst_points.get_point(pt_id - 1);

			// Vertical lines
			ndcnst_lines.add_line(ln_id, pt1, pt5, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt2, pt6, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt3, pt7, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt4, pt8, normalized_cnst_loc);
			ln_id++;

			// top square
			ndcnst_lines.add_line(ln_id, pt1, pt2, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt2, pt3, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt3, pt4, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt4, pt1, normalized_cnst_loc);
			ln_id++;

			// bot square
			ndcnst_lines.add_line(ln_id, pt5, pt6, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt6, pt7, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt7, pt8, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt8, pt5, normalized_cnst_loc);
			ln_id++;

		}
		else if (ndcnst.constraint_type == 1)
		{
			// Define the neck location of the arrow head
			glm::vec3 load_arrow_neck_top = ndcnst_loc + normalized_cnst_loc *
				static_cast<float>(1.0f * (geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale)); // 0

			// Define offset distances for arrow head points
			float arrow_head_offset = 1.0f * (geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale));


			// Calculate arrow head points (Top triangle)
			glm::vec3 load_arrow_pt1 = load_arrow_neck_top + glm::normalize(uv_orthogonal.first) * arrow_head_offset;
			glm::vec3 load_arrow_pt2 = load_arrow_neck_top + glm::normalize(rotateVector(uv_orthogonal.first, normalized_cnst_loc, 120.0f)) * arrow_head_offset;
			glm::vec3 load_arrow_pt3 = load_arrow_neck_top + glm::normalize(rotateVector(uv_orthogonal.first, normalized_cnst_loc, 240.0f)) * arrow_head_offset;

			// Pinned constraint
			ndcnst_points.add_point(pt_id, ndcnst_loc.x, ndcnst_loc.y, ndcnst_loc.z);
			pt_id++;

			ndcnst_points.add_point(pt_id, load_arrow_pt1.x, load_arrow_pt1.y, load_arrow_pt1.z);
			pt_id++;

			ndcnst_points.add_point(pt_id, load_arrow_pt2.x, load_arrow_pt2.y, load_arrow_pt2.z);
			pt_id++;

			ndcnst_points.add_point(pt_id, load_arrow_pt3.x, load_arrow_pt3.y, load_arrow_pt3.z);
			pt_id++;

			point_store* pt1 = ndcnst_points.get_point(pt_id - 4);
			point_store* pt2 = ndcnst_points.get_point(pt_id - 3);
			point_store* pt3 = ndcnst_points.get_point(pt_id - 2);
			point_store* pt4 = ndcnst_points.get_point(pt_id - 1);


			ndcnst_lines.add_line(ln_id, pt1, pt2, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt1, pt3, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt1, pt4, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt2, pt3, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt3, pt4, normalized_cnst_loc);
			ln_id++;

			ndcnst_lines.add_line(ln_id, pt4, pt2, normalized_cnst_loc);
			ln_id++;

		}

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


std::pair<glm::vec3, glm::vec3> nodecnst_list_store::findOrthogonalVectors(const glm::vec3& v)
{
	// Step 1: Choose an arbitrary vector u
	// For example, set one component to zero and others to non-zero values
	glm::vec3 u = glm::vec3(1.0f, 0.0f, 0.0f);
	if (v.y != 0.0f || v.z != 0.0f)
	{
		u = glm::vec3(0.0f, 1.0f, 0.0f);
	}

	// Step 2: Compute the cross product of v and u
	glm::vec3 w = glm::cross(v, u);

	// Step 3: Normalize w
	w = glm::normalize(w);

	// Step 4: Compute the cross product of v and w to find the second orthogonal vector
	glm::vec3 u_prime = glm::cross(v, w);

	return std::make_pair(w, u_prime);
}


glm::vec3 nodecnst_list_store::rotateVector(const glm::vec3& v, const glm::vec3& axis, float angleDegrees)
{

	float angleRadians = glm::radians(angleDegrees); // Convert angle to radians

	glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), angleRadians, axis);
	return glm::vec3(rotationMatrix * glm::vec4(v, 1.0f));
}







