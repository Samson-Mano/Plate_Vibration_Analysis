#include "nodeload_list_store.h"

nodeload_list_store::nodeload_list_store()
{
	// Empty constructor
}

nodeload_list_store::~nodeload_list_store()
{
	// Empty destructor
}

void nodeload_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// load_value_labels.init(geom_param_ptr);

	// Set the geometry parameter for the lines showing loads
	load_points.init(geom_param_ptr);
	load_lines.init(geom_param_ptr);
	
	load_points.set_point_color(geom_param_ptr->geom_colors.load_color);
	load_lines.set_line_color(geom_param_ptr->geom_colors.load_color);


	// Clear the loads
	load_count = 0;
	load_max = 0.0;
	loadMap.clear();
	all_load_ids.clear();
}

void nodeload_list_store::set_zero_condition(const int& model_type)
{
	this->model_type = model_type; // Model type 0 - Circular, 1,2,3 Rectangular


}

void nodeload_list_store::add_loads(std::vector<int>& node_ids, std::vector<glm::vec3>& load_locs, std::vector<glm::vec3>& load_normals, double& load_start_time,
	double& load_end_time, double& load_value)
{
	load_data temp_load;
	temp_load.load_set_id = get_unique_load_id(all_load_ids); // Load id
	temp_load.node_ids = node_ids; // id of the line its applied to
	temp_load.load_locs = load_locs; // Load location
	temp_load.load_normals = load_normals;
	temp_load.load_start_time = load_start_time; // Load start time
	temp_load.load_end_time = load_end_time; // Load end time
	temp_load.load_value = load_value; // Load value

	// Insert the load to line
	loadMap.push_back(temp_load);
	all_load_ids.push_back(temp_load.load_set_id); // Add to the id vector
	load_count++;
}

void nodeload_list_store::delete_load(int& node_id)
{
	if (load_count == 0)
	{
		return;
	}

	// Delete all the loads in the node
	std::vector<int> delete_load_index;
	int ld_index = 0;

	for (auto& ld : loadMap)
	{
		// Check whether the load's nodeID is the nodeID
		for (auto& nd_id : ld.node_ids)
		{
			if (nd_id == node_id)
			{
				// Add to the delete load ID
				delete_load_index.push_back(ld_index);

				// Delete the load set id
				// Delete the iD from the load ids
				auto it = std::find(all_load_ids.begin(), all_load_ids.end(), ld.load_set_id);
				all_load_ids.erase(it);


				break;
			}
		}

		ld_index++;
	}

	//____________________________________

	 // Iterate over the delete indices vector and erase elements from the original vector
	for (int index : delete_load_index)
	{
		if (index >= 0 && index < loadMap.size())
		{
			loadMap.erase(loadMap.begin() + index);

			// Reduce the load count
			load_count--;
		}
	}

}


void nodeload_list_store::set_buffer()
{
	// Set the buffer for Loads

	// Set the load max
	// Load Max
	load_max = 0.0;
	// Set the load lables
	// load_value_labels.clear_labels();

	// Find the load maximum
	for (auto& load : loadMap)
	{
		if (load_max < std::abs(load.load_value))
		{
			load_max = std::abs(load.load_value);
		}
		//__________________________________________________________________________

		//if (load.show_load_label == true)
		//{
		//	std::stringstream ss;
		//	ss << std::fixed << std::setprecision(geom_param_ptr->load_precision) << std::abs(load.load_value);

		//	glm::vec3 temp_color = geom_param_ptr->geom_colors.load_color;
		//	std::string	temp_str = "(" + std::to_string(load.node_id) + ") " + ss.str();
		//	double load_angle_rad = 0.0f;

		//	bool is_load_val_above = false;
		//	if (load.load_value < 0)
		//	{
		//		is_load_val_above = true;
		//	}

		//	load_value_labels.add_text(temp_str, load.load_loc, glm::vec3(0), temp_color, load_angle_rad, is_load_val_above, false);
		//}
	}

	// load_value_labels.set_buffer();

	//__________________________________________________________________________
	// Get the total individual load count from load set count
	total_load_count = 0;

	for (auto& ld : loadMap)
	{
		total_load_count = total_load_count + static_cast<int>(ld.node_ids.size());
	}


	load_points.clear_points();
	load_lines.clear_lines();


	//_________________________________________________________
	// Create the points
	int pt_id = 0;
	int ln_id = 0;

	for (auto& ld : loadMap)
	{
		int load_sign = ld.load_value > 0 ? 1 : -1;
		double load_val = ld.load_value;

		int i = 0;

		for (auto& ld_loc : ld.load_locs)
		{
			glm::vec3 load_loc = ld_loc;

			// Rotate the corner points
			glm::vec3 normalized_load_loc = ld.load_normals[i]; // Normalize the load location
			i++;

			glm::vec3 load_arrow_startpt = load_loc + normalized_load_loc *
				static_cast<float>(0.0f * load_sign * (geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale)); // 0
			glm::vec3 load_arrow_endpt = load_loc + normalized_load_loc *
				static_cast<float>(20.0f * (load_val / load_max) * (geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale)); // 1

			//___________________________
			std::pair<glm::vec3, glm::vec3> uv_orthogonal = findOrthogonalVectors(normalized_load_loc);

			// Define the neck location of the arrow head
			glm::vec3 load_arrow_neck = load_loc + normalized_load_loc *
				static_cast<float>(5.0f * load_sign * (geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale)); // 0

			// Define offset distances for arrow head points
			float arrow_head_offset = 2.0f * (geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale));


			// Calculate arrow head points
			glm::vec3 load_arrow_pt1 = load_arrow_neck + glm::normalize(uv_orthogonal.first) * arrow_head_offset;

			glm::vec3 load_arrow_pt2 = load_arrow_neck + glm::normalize(rotateVector(uv_orthogonal.first, normalized_load_loc, 120.0f)) * arrow_head_offset;

			glm::vec3 load_arrow_pt3 = load_arrow_neck + glm::normalize(rotateVector(uv_orthogonal.first, normalized_load_loc, 240.0f)) * arrow_head_offset;

			// Add the load start point
			load_points.add_point(pt_id, load_arrow_startpt.x, load_arrow_startpt.y, load_arrow_startpt.z);
			pt_id++;

			// Add the load end point
			load_points.add_point(pt_id, load_arrow_endpt.x, load_arrow_endpt.y, load_arrow_endpt.z);
			pt_id++;

			// Add the arrow head pt1, pt2 and pt3
			load_points.add_point(pt_id, load_arrow_pt1.x, load_arrow_pt1.y, load_arrow_pt1.z);
			pt_id++;

			load_points.add_point(pt_id, load_arrow_pt2.x, load_arrow_pt2.y, load_arrow_pt2.z);
			pt_id++;

			load_points.add_point(pt_id, load_arrow_pt3.x, load_arrow_pt3.y, load_arrow_pt3.z);
			pt_id++;


			// Add the load lines

			point_store* ld_start_pt = load_points.get_point(pt_id - 5);
			point_store* ld_end_pt = load_points.get_point(pt_id - 4);
			point_store* ld_arrow_pt1 = load_points.get_point(pt_id - 3);
			point_store* ld_arrow_pt2 = load_points.get_point(pt_id - 2);
			point_store* ld_arrow_pt3 = load_points.get_point(pt_id - 1);

			glm::vec3 normal = glm::normalize(ld_end_pt->pt_coord() - ld_start_pt->pt_coord());

			// Load main line
			load_lines.add_line(ln_id, ld_start_pt, ld_end_pt, normal);
			ln_id++;

			// Load arrow head line 1
			load_lines.add_line(ln_id, ld_start_pt, ld_arrow_pt1, normal);
			ln_id++;

			// Load arrow head line 2
			load_lines.add_line(ln_id, ld_start_pt, ld_arrow_pt2, normal);
			ln_id++;

			// Load arrow head line 3
			load_lines.add_line(ln_id, ld_start_pt, ld_arrow_pt3, normal);
			ln_id++;

			// Load arrow top triangle 1
			load_lines.add_line(ln_id, ld_arrow_pt1, ld_arrow_pt2, normal);
			ln_id++;

			// Load arrow top trianlge 2
			load_lines.add_line(ln_id, ld_arrow_pt2, ld_arrow_pt3, normal);
			ln_id++;

			// Load arrow top triangle 3
			load_lines.add_line(ln_id, ld_arrow_pt3, ld_arrow_pt1, normal);
			ln_id++;

		}

	}
		
	load_points.set_buffer();
	load_lines.set_buffer();

}

void nodeload_list_store::paint_loads()
{
	// Paint the load visualization
	load_lines.paint_static_lines();

}

void nodeload_list_store::paint_load_labels()
{
	// Paint load labels
	// load_value_labels.paint_text();
}

void nodeload_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_viewmatrix, bool set_transparency)
{

	load_points.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, set_transparency);
	load_lines.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, set_transparency);

}


int nodeload_list_store::get_unique_load_id(std::vector<int>& all_ids)
{
	// Return the unique Load id
	if (all_ids.size() != 0)
	{
		int i;
		std::sort(all_ids.begin(), all_ids.end());

		// Find if any of the nodes are missing in an ordered int
		for (i = 0; i < all_ids.size(); i++)
		{
			if (all_ids[i] != i)
			{
				return i;
			}
		}

		// no node id is missing in an ordered list so add to the end
		return static_cast<int>(all_ids.size());
	}
	return 0;
}


std::pair<glm::vec3, glm::vec3> nodeload_list_store::findOrthogonalVectors(const glm::vec3& v)
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


glm::vec3 nodeload_list_store::rotateVector(const glm::vec3& v, const glm::vec3& axis, float angleDegrees)
{

	float angleRadians = glm::radians(angleDegrees); // Convert angle to radians

	glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), angleRadians, axis);
	return glm::vec3(rotationMatrix * glm::vec4(v, 1.0f));
}






