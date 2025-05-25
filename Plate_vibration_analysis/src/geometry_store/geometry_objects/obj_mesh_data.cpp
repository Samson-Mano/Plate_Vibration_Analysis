#include "obj_mesh_data.h"

obj_mesh_data::obj_mesh_data()
{
	// Empty constructor
}

obj_mesh_data::~obj_mesh_data()
{
	// Empty destructor
}

void obj_mesh_data::init(geom_parameters* geom_param_ptr)
{
	// Initialize the geometry objects of the mesh
	half_edge_count = 0;

	// Delete dynamically allocated memory
	for (auto ptr : mesh_half_edges)
	{
		delete ptr;
	}

	mesh_half_edges.clear(); // clear the half edge data


	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Nodes
	this->mesh_points.init(geom_param_ptr);
	this->selected_mesh_points.init(geom_param_ptr);
	this->selected_mesh_points.set_point_color(geom_param_ptr->geom_colors.selection_color);

	// Mesh boundaries & mesh normals
	this->mesh_boundaries.init(geom_param_ptr);
	this->mesh_normals.init(geom_param_ptr);

	// Mesh data
	this->mesh_tris.init(geom_param_ptr);
	this->selected_mesh_tris.init(geom_param_ptr);
	this->selected_mesh_tris.set_tri_color(geom_param_ptr->geom_colors.selection_color, 1.0f);

	this->mesh_quads.init(geom_param_ptr);
	this->selected_mesh_quads.init(geom_param_ptr);
	this->selected_mesh_quads.set_quad_color(geom_param_ptr->geom_colors.selection_color, 1.0f);

	// Mesh material ID data
	this->mesh_tri_material_ids.init(geom_param_ptr);
	this->mesh_quad_material_ids.init(geom_param_ptr);

}

void obj_mesh_data::add_mesh_point(const int& point_id, const double& x_coord, const double& y_coord, const double& z_coord)
{
	// add the points
	this->mesh_points.add_point(point_id, x_coord, y_coord, z_coord);

}

void obj_mesh_data::add_selected_points(const std::vector<int>& selected_point_id)
{
	// Add the selected points
	selected_mesh_points.clear_points();

	// Selected points id
	int id = 0;
	for (auto& pt_id : selected_point_id)
	{
		// get the node point
		point_store* pt = this->mesh_points.get_point(pt_id);

		selected_mesh_points.add_point(id, pt->x_coord, pt->y_coord, pt->z_coord);
		id++;
	}

	selected_mesh_points.set_buffer();

}

void obj_mesh_data::add_selected_tris(const std::vector<int>& selected_tri_id)
{
	// Add the selected tris
	selected_mesh_tris.clear_triangles();

	// Selected tris id
	int id = 0;
	for (auto& tri_id : selected_tri_id)
	{
		// get the tri
		tri_store* tri = this->mesh_tris.get_triangle(tri_id);

		selected_mesh_tris.add_tri(id, tri->edge1, tri->edge2, tri->edge3);
		id++;
	}

	selected_mesh_tris.set_buffer();

}

void obj_mesh_data::add_selected_quads(const std::vector<int>& selected_quad_id)
{
	// Add the selected quads
	selected_mesh_quads.clear_quadrilaterals();

	// Selected quads id
	int id = 0;
	for (auto& quad_id : selected_quad_id)
	{
		// get the quad
		tri_store* tri123 = this->mesh_quads.get_quadrilateral_face123(quad_id);
		tri_store* tri341 = this->mesh_quads.get_quadrilateral_face341(quad_id);

		selected_mesh_quads.add_quad(id, tri123->edge1, tri123->edge2, tri123->edge3, tri341->edge1, tri341->edge2, tri341->edge3);
		id++;
	}

	selected_mesh_quads.set_buffer();

}


void obj_mesh_data::add_mesh_tris(const int& tri_id, const int& point_id1, const int& point_id2, const int& point_id3)
{

	//    2____3 
	//    |   /  
	//    | /    
	//    1      

	// Add the half triangle of the quadrilaterals
	// Add three half edges
	int line_id1, line_id2, line_id3;

	// Add edge 1
	line_id1 = add_half_edge(point_id1, point_id2);

	// Point 1 or point 2 not found
	if (line_id1 == -1)
		return;

	// Add edge 2
	line_id2 = add_half_edge(point_id2, point_id3);

	// Point 3 not found
	if (line_id2 == -1)
	{
		mesh_half_edges.pop_back(); // remove the last item which is edge 1
		half_edge_count--;
		return;
	}


	// Add edge 3
	line_id3 = add_half_edge(point_id3, point_id1);


	//________________________________________
	// Add the mesh triangles
	this->mesh_tris.add_tri(tri_id, mesh_half_edges[line_id1],
		mesh_half_edges[line_id2],
		mesh_half_edges[line_id3]);


	// Set the half edges next line
	mesh_half_edges[line_id1]->next_line = mesh_half_edges[line_id2];
	mesh_half_edges[line_id2]->next_line = mesh_half_edges[line_id3];
	mesh_half_edges[line_id3]->next_line = mesh_half_edges[line_id1];

	// Set the half edge face data
	tri_store* temp_tri = this->mesh_tris.get_triangle(tri_id);

	mesh_half_edges[line_id1]->face = temp_tri;
	mesh_half_edges[line_id2]->face = temp_tri;
	mesh_half_edges[line_id3]->face = temp_tri;


	//_______________________________________________________________________________________________________
	// Add a text for material ID
	glm::vec3 nd_pt1 = temp_tri->edge1->start_pt->pt_coord();
	glm::vec3 nd_pt2 = temp_tri->edge2->start_pt->pt_coord();
	glm::vec3 nd_pt3 = temp_tri->edge3->start_pt->pt_coord();

	// Calculate the midpoint of the triangle
	glm::vec3 tri_mid_pt = glm::vec3((nd_pt1.x + nd_pt2.x + nd_pt3.x) * 0.33333f,
		(nd_pt1.y + nd_pt2.y + nd_pt3.y) * 0.33333f,
		(nd_pt1.z + nd_pt2.z + nd_pt3.z) * 0.33333f);

	// Add the material ID
	glm::vec3	temp_str_color = geom_parameters::get_standard_color(0);
	std::string	temp_str = " M = " + std::to_string(0);
	mesh_tri_material_ids.add_text(tri_id, temp_str, tri_mid_pt, temp_str_color);


}

void obj_mesh_data::add_mesh_quads(const int& quad_id, const int& point_id1, const int& point_id2, const int& point_id3, const int& point_id4)
{

	//    2____3     2____3      3
	//    |   /|     |   /     / |  
	//    | /  |     | /     /   |   
	//    1____4     1      1____4   

	// Add the quadrilaterals
	// Add the 1st half triangle of the quadrilaterals
	// Add three half edges
	int line_id1, line_id2, line_id3;

	// Add edge 1
	line_id1 = add_half_edge(point_id1, point_id2);

	// Add edge 2
	line_id2 = add_half_edge(point_id2, point_id3);

	// Add edge 3
	line_id3 = add_half_edge(point_id3, point_id1);

	// Set the half edges next line
	mesh_half_edges[line_id1]->next_line = mesh_half_edges[line_id2];
	mesh_half_edges[line_id2]->next_line = mesh_half_edges[line_id3];
	mesh_half_edges[line_id3]->next_line = mesh_half_edges[line_id1];


	// Add the 2nd half triangle of the quadrilaterals
	// Add three half edges
	int line_id4, line_id5, line_id6;

	// Add edge 4
	line_id4 = add_half_edge(point_id3, point_id4);

	// Add edge 5
	line_id5 = add_half_edge(point_id4, point_id1);

	// Add edge 6
	line_id6 = add_half_edge(point_id1, point_id3);


	// Set the half edges next line
	mesh_half_edges[line_id4]->next_line = mesh_half_edges[line_id5];
	mesh_half_edges[line_id5]->next_line = mesh_half_edges[line_id6];
	mesh_half_edges[line_id6]->next_line = mesh_half_edges[line_id4];


	//________________________________________
	// Add the mesh quadrilaterals
	this->mesh_quads.add_quad(quad_id, mesh_half_edges[line_id1],
		mesh_half_edges[line_id2],
		mesh_half_edges[line_id3],
		mesh_half_edges[line_id4],
		mesh_half_edges[line_id5],
		mesh_half_edges[line_id6]);

	// Set the half edge face data 1st Half triangle of the quadrilateral
	tri_store* temp_tri123 = this->mesh_quads.get_quadrilateral_face123(quad_id);

	mesh_half_edges[line_id1]->face = temp_tri123;
	mesh_half_edges[line_id2]->face = temp_tri123;
	mesh_half_edges[line_id3]->face = temp_tri123;

	// Set the half edge face data 2nd Half triangle of the quadrilateral
	tri_store* temp_tri341 = this->mesh_quads.get_quadrilateral_face341(quad_id);

	mesh_half_edges[line_id4]->face = temp_tri341;
	mesh_half_edges[line_id5]->face = temp_tri341;
	mesh_half_edges[line_id6]->face = temp_tri341;


	//_______________________________________________________________________________________________________
	// Add a text for material ID
	glm::vec3 nd_pt1 = temp_tri123->edge1->start_pt->pt_coord();
	glm::vec3 nd_pt2 = temp_tri123->edge2->start_pt->pt_coord();
	glm::vec3 nd_pt3 = temp_tri341->edge1->start_pt->pt_coord();
	glm::vec3 nd_pt4 = temp_tri341->edge2->start_pt->pt_coord();

	// Calculate the midpoint of the triangle
	glm::vec3 quad_mid_pt = glm::vec3((nd_pt1.x + nd_pt2.x + nd_pt3.x + nd_pt4.x) * 0.25f,
		(nd_pt1.y + nd_pt2.y + nd_pt3.y + nd_pt4.y) * 0.25f,
		(nd_pt1.z + nd_pt2.z + nd_pt3.z + nd_pt4.z) * 0.25f);

	// Add the material ID
	glm::vec3 temp_str_color = geom_parameters::get_standard_color(0);
	std::string	temp_str = " M = " + std::to_string(0);
	mesh_quad_material_ids.add_text(quad_id, temp_str, quad_mid_pt, temp_str_color);


}


void obj_mesh_data::set_mesh_wireframe()
{
	// Step 1: Set the twin of half edges
	for (int i = 0; i < static_cast<int>(mesh_half_edges.size()); i++)
	{
		// If the twin_line is already set, continue to the next half-edge
		if (mesh_half_edges[i]->twin_line != nullptr)
			continue;

		// Get the start and end points of the current half-edge
		point_store* start_pt = mesh_half_edges[i]->start_pt;
		point_store* end_pt = mesh_half_edges[i]->end_pt;

		// Iterate through the remaining half-edges to find the twin
		for (int j = i + 1; j < static_cast<int>(mesh_half_edges.size()); j++)
		{
			// Check if the current half-edge has the same start and end points as the twin
			if (mesh_half_edges[j]->start_pt == end_pt && mesh_half_edges[j]->end_pt == start_pt)
			{
				// Set the twin_line for both half-edges
				mesh_half_edges[i]->twin_line = mesh_half_edges[j];
				mesh_half_edges[j]->twin_line = mesh_half_edges[i];
				break; // Break the loop since twin has been found
			}
		}
	}

	//_________________________________________________________________________________________________
	// Step 2: Set the mesh boundaries & mesh normals
	mesh_boundaries.clear_lines();
	mesh_normals.clear_lines();

	std::set<std::pair<int, int>> seenLines;
	int line_id = 0;

	// Triangles
	for (const auto& tri : this->mesh_tris.triMap)
	{
		// Edge 1
		set_mesh_edge(tri->edge1, line_id, seenLines);

		// Edge 2
		set_mesh_edge(tri->edge2, line_id, seenLines);

		// Edge 3
		set_mesh_edge(tri->edge3, line_id, seenLines);

		// Set the mesh normal
		set_mesh_normal_vector(tri);
	}

	// Quadrilaterals
	for (const auto& quad : this->mesh_quads.quadMap)
	{
		// Edge 1
		set_mesh_edge(quad->tri123->edge1, line_id, seenLines);

		// Edge 2
		set_mesh_edge(quad->tri123->edge2, line_id, seenLines);

		// Edge 3
		set_mesh_edge(quad->tri341->edge1, line_id, seenLines);

		// Edge 4
		set_mesh_edge(quad->tri341->edge2, line_id, seenLines);

		// Set the mesh normal
		set_mesh_normal_vector(quad);
	}

}


void obj_mesh_data::set_mesh_node_normals()
{
	for (const auto& mesh_pt_id_m : mesh_points.pointId_Map)
	{
		// Get the normals
		std::vector<glm::vec3> connected_mesh_normals;
		int mesh_pt_id = mesh_pt_id_m.first;
		int mesh_pt_index = mesh_pt_id_m.second;

		// Triangle mesh
		for (const auto& tri : this->mesh_tris.triMap)
		{
			// find the trianlge connected to the mesh point
			if (tri->edge1->start_pt->point_id == mesh_pt_id ||
				tri->edge2->start_pt->point_id == mesh_pt_id ||
				tri->edge3->start_pt->point_id == mesh_pt_id)
			{
				connected_mesh_normals.push_back(tri->face_normal);
			}

		}

		// Quadrilateral mesh
		for (const auto& quad : this->mesh_quads.quadMap)
		{
			// find the quadrilateral connected to the mesh point
			if (quad->tri123->edge1->start_pt->point_id == mesh_pt_id ||
				quad->tri123->edge2->start_pt->point_id == mesh_pt_id ||
				quad->tri341->edge1->start_pt->point_id == mesh_pt_id ||
				quad->tri341->edge2->start_pt->point_id == mesh_pt_id)
			{
				connected_mesh_normals.push_back(quad->face_normal);
			}

		}

		// Average and normalize all the normals
		glm::vec3 pt_normal = geom_param_ptr->average_normal(connected_mesh_normals);

		// Set the mesh normals
		this->mesh_points.pointMap[mesh_pt_index].pt_normal = pt_normal;
	}

}


glm::vec3 obj_mesh_data::get_mesh_node_normals(const int& point_id)
{
	// get the mesh normal of node point
	return	this->mesh_points.pointMap[mesh_points.pointId_Map[point_id]].pt_normal;

}


void obj_mesh_data::set_mesh_edge(line_store* edge, int& line_id, std::set<std::pair<int, int>>& seenLines)
{
	// Ensure the start point ID is smaller than the end point ID
	int smaller_id = std::min(edge->start_pt->point_id, edge->end_pt->point_id);
	int larger_id = std::max(edge->start_pt->point_id, edge->end_pt->point_id);
	std::pair<int, int> lineEndpoint_ids = std::make_pair(smaller_id, larger_id);

	// If the line is not already seen in the opposite direction, add it to the unique lines
	if (seenLines.find(lineEndpoint_ids) == seenLines.end())
	{
		glm::vec3 line_normal = glm::vec3(0);

		// get the light and right face normal
		glm::vec3 left_face_normal = edge->face->face_normal;

		if (edge->twin_line != nullptr)
		{
			glm::vec3 right_face_normal = edge->twin_line->face->face_normal;
			// Compute the average normal only if twin_line is not nullptr
			line_normal = glm::normalize(left_face_normal + right_face_normal);
		}
		else
		{
			// Handle the case where twin_line is nullptr
			line_normal = glm::normalize(left_face_normal);
		}

		edge->line_normal = line_normal;

		// Create the mesh boundary
		mesh_boundaries.add_line(line_id,
			mesh_points.get_point(smaller_id),
			mesh_points.get_point(larger_id), line_normal);

		line_id++;

		// Add to the seen lines
		seenLines.insert(lineEndpoint_ids);
	}
}

void obj_mesh_data::set_mesh_normal_vector(tri_store* tri)
{
	// Get the geometric center of the triangle
	point_store* pt1 = new point_store;
	pt1->x_coord = tri->geom_center.x;
	pt1->y_coord = tri->geom_center.y;
	pt1->z_coord = tri->geom_center.z;

	// Get the face normal of the triangle
	point_store* pt2 = new point_store;
	pt2->x_coord = tri->geom_center.x + 10.0 * tri->face_normal.x;
	pt2->y_coord = tri->geom_center.y + 10.0 * tri->face_normal.y;
	pt2->z_coord = tri->geom_center.z + 10.0 * tri->face_normal.z;

	int normal_id = mesh_normals.line_count;
	mesh_normals.add_line(normal_id, pt1, pt2, tri->face_normal);

}


void obj_mesh_data::set_mesh_normal_vector(quad_store* quad)
{
	// Get the geometric center of the triangle
	point_store* pt1 = new point_store;
	pt1->x_coord = quad->geom_center.x;
	pt1->y_coord = quad->geom_center.y;
	pt1->z_coord = quad->geom_center.z;

	// Get the face normal of the triangle
	point_store* pt2 = new point_store;
	pt2->x_coord = quad->geom_center.x + 10.0 * quad->face_normal.x;
	pt2->y_coord = quad->geom_center.y + 10.0 * quad->face_normal.y;
	pt2->z_coord = quad->geom_center.z + 10.0 * quad->face_normal.z;

	int normal_id = mesh_normals.line_count;
	mesh_normals.add_line(normal_id, pt1, pt2, quad->face_normal);

}


void obj_mesh_data::update_mesh_point(const int& point_id, const double& x_coord, const double& y_coord, const double& z_coord)
{
	// Update the point with new - coordinates
	this->mesh_points.update_point(point_id, x_coord, y_coord, z_coord);

}


void obj_mesh_data::update_tri_material_ids(const std::vector<int>& selected_tri_id, const int& material_id)
{
	// Update the material IDs of triangle elements
	for (auto& tri_id : selected_tri_id)
	{
		// get the tri
		tri_store* tri = this->mesh_tris.get_triangle(tri_id);

		glm::vec3 nd_pt1 = tri->edge1->start_pt->pt_coord();
		glm::vec3 nd_pt2 = tri->edge2->start_pt->pt_coord();
		glm::vec3 nd_pt3 = tri->edge3->start_pt->pt_coord();

		// Calculate the midpoint of the triangle
		glm::vec3 tri_mid_pt = glm::vec3((nd_pt1.x + nd_pt2.x + nd_pt3.x) * 0.33333f,
			(nd_pt1.y + nd_pt2.y + nd_pt3.y) * 0.33333f,
			(nd_pt1.z + nd_pt2.z + nd_pt3.z) * 0.33333f);

		// Add the material ID
		glm::vec3 temp_str_color = geom_parameters::get_standard_color(material_id);
		std::string	temp_str = " 0 = " + std::to_string(material_id);
		mesh_tri_material_ids.add_text(tri_id, temp_str, tri_mid_pt, temp_str_color);

	}

	// Set the buffer for the labels
	mesh_tri_material_ids.update_buffer();

}


void obj_mesh_data::update_quad_material_ids(const std::vector<int>& selected_quad_id, const int& material_id)
{
	// Update the material IDs of quadrilateral elements

	for (auto& quad_id : selected_quad_id)
	{
		// get the quad
		tri_store* tri123 = this->mesh_quads.get_quadrilateral_face123(quad_id);
		tri_store* tri341 = this->mesh_quads.get_quadrilateral_face341(quad_id);


		glm::vec3 nd_pt1 = tri123->edge1->start_pt->pt_coord();
		glm::vec3 nd_pt2 = tri123->edge2->start_pt->pt_coord();
		glm::vec3 nd_pt3 = tri341->edge1->start_pt->pt_coord();
		glm::vec3 nd_pt4 = tri341->edge2->start_pt->pt_coord();

		// Calculate the midpoint of the triangle
		glm::vec3 quad_mid_pt = glm::vec3((nd_pt1.x + nd_pt2.x + nd_pt3.x + nd_pt4.x) * 0.25f,
			(nd_pt1.y + nd_pt2.y + nd_pt3.y + nd_pt4.y) * 0.25f,
			(nd_pt1.z + nd_pt2.z + nd_pt3.z + nd_pt4.z) * 0.25f);

		glm::vec3 temp_str_color = geom_parameters::get_standard_color(material_id);
		std::string	temp_str = " 0 = " + std::to_string(material_id);
		mesh_quad_material_ids.update_text(quad_id, temp_str, quad_mid_pt, temp_str_color);

	}

	// Set the buffer for the labels
	mesh_quad_material_ids.update_buffer();

}



//void obj_mesh_data::update_mesh_buffer()
//{
//	// Update the mesh point buffer
//	this->mesh_points.update_buffer();
//	this->mesh_boundaries.update_buffer();
//
//}

void obj_mesh_data::update_mesh_color(const glm::vec3& point_color, const glm::vec3& line_color, const glm::vec3& tri_color, const double& transparency)
{
	// Set the color of the mesh
	this->mesh_points.set_point_color(point_color);
	this->mesh_boundaries.set_line_color(line_color);
	this->mesh_tris.set_tri_color(tri_color, transparency);
	this->mesh_quads.set_quad_color(tri_color, transparency);

}

void obj_mesh_data::set_buffer()
{
	// Set the buffer
	this->mesh_points.set_buffer();

	// Mesh boundaries & mesh normals
	this->mesh_normals.set_buffer();
	this->mesh_boundaries.set_buffer();

	// Mesh data
	this->mesh_tris.set_buffer();
	this->mesh_quads.set_buffer();

	// Mesh material labels
	this->mesh_tri_material_ids.set_buffer();
	this->mesh_quad_material_ids.set_buffer();

}

void obj_mesh_data::clear_mesh()
{
	// Clear the mesh
		// Half edge
	half_edge_count = 0;

	// Delete dynamically allocated memory
	for (auto ptr : mesh_half_edges)
	{
		delete ptr;
	}

	mesh_half_edges.clear(); // clear the half edges

	// Nodes
	mesh_points.clear_points();
	selected_mesh_points.clear_points();

	// Mesh boundaries & mesh normals
	mesh_normals.clear_lines();
	mesh_boundaries.clear_lines();

	// Mesh data
	mesh_tris.clear_triangles();
	mesh_quads.clear_quadrilaterals();

	// Mesh material labels
	mesh_tri_material_ids.clear_texts();
	mesh_quad_material_ids.clear_texts();

}


void obj_mesh_data::paint_static_mesh()
{
	// Paint the static mesh (mesh which are fixed)
		// Paint the mesh triangles
	this->mesh_tris.paint_static_triangles();
	this->mesh_quads.paint_static_quadrilaterals();

}


void obj_mesh_data::paint_static_mesh_boundaries()
{
	// Paint the mesh boundaries
	this->mesh_boundaries.paint_static_lines();

}


void obj_mesh_data::paint_static_mesh_points()
{
	// Paint the mesh points
	this->mesh_points.paint_static_points();

}

void obj_mesh_data::paint_dynamic_mesh()
{
	// Paint the dynamic mesh (mesh which are not-fixed but variable)
		// Paint the mesh triangles
	this->mesh_tris.paint_dynamic_triangles();
	this->mesh_quads.paint_dynamic_quadrilaterals();

}

void obj_mesh_data::paint_dynamic_mesh_boundaries()
{
	// Paint the mesh lines
	this->mesh_boundaries.paint_dynamic_lines();

}

void obj_mesh_data::paint_dynamic_mesh_points()
{
	// Paint the mesh points
	this->mesh_points.paint_dynamic_points();

}

void obj_mesh_data::paint_selected_points()
{
	// Paint the selected points
	this->selected_mesh_points.paint_static_points();

}

void obj_mesh_data::paint_selected_mesh()
{
	// Paint the selected tris and quds
	this->selected_mesh_tris.paint_static_triangles();
	this->selected_mesh_quads.paint_static_quadrilaterals();

}


void obj_mesh_data::paint_mesh_normals()
{
	// Paint the mesh normals
	this->mesh_normals.paint_static_lines();

}

void obj_mesh_data::paint_mesh_materialids()
{
	// Paint the mesh material IDs
	this->mesh_tri_material_ids.paint_static_texts();
	this->mesh_quad_material_ids.paint_static_texts();

}

void obj_mesh_data::update_opengl_uniforms(bool set_modelmatrix, bool set_viewmatrix, bool set_transparency)
{
	// Update the openGl uniform matrices
	this->mesh_quads.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, false); // do not use default transparency 
	this->mesh_tris.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, false); // do not use default transparency 
	this->selected_mesh_quads.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, false); // do not use default transparency 
	this->selected_mesh_tris.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, false); // do not use default transparency 

	this->mesh_normals.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, set_transparency);
	this->mesh_boundaries.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, set_transparency);

	this->selected_mesh_points.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, set_transparency);
	this->mesh_points.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, set_transparency);

	this->mesh_tri_material_ids.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, set_transparency);
	this->mesh_quad_material_ids.update_opengl_uniforms(set_modelmatrix, set_viewmatrix, set_transparency);

}


int obj_mesh_data::add_half_edge(const int& startPt_id, const int& endPt_id)
{
	// Add the Half edge
	line_store* temp_edge = new line_store;
	temp_edge->line_id = half_edge_count;
	temp_edge->start_pt = this->mesh_points.get_point(startPt_id);
	temp_edge->end_pt = this->mesh_points.get_point(endPt_id);

	// Add to the Half edge list
	mesh_half_edges.push_back(temp_edge);

	// Iterate
	half_edge_count++;

	return (half_edge_count - 1); // return the index of last addition
}
