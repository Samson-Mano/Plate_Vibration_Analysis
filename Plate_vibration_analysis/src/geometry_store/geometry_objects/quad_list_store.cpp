#include "quad_list_store.h"

quad_list_store::quad_list_store()
{
	// Empty constructor
}

quad_list_store::~quad_list_store()
{
	// Empty destructor
}

void quad_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Create the quadrilateral shader
	std::filesystem::path shadersPath = geom_param_ptr->resourcePath;

	quad_shader.create_shader((shadersPath.string() + "/resources/shaders/mesh_vert_shader.vert").c_str(),
		(shadersPath.string() + "/resources/shaders/mesh_frag_shader.frag").c_str());

	quad_shader.setUniform("vertexColor", geom_param_ptr->geom_colors.triangle_color);


	// Delete all the quadrilaterals
	clear_quadrilaterals();
}

void quad_list_store::add_quad(const int& quad_id, line_store* edge1, line_store* edge2, 
	line_store* edge3, line_store* edge4, 
	line_store* edge5, line_store* edge6)
{
	// Add to the list
	quad_store* temp_quad = new quad_store;

	temp_quad->quad_id = quad_id;

	//___________________________________________________________________________________________
	// Half triangle 1
	tri_store* temp_tri123 = new tri_store;

	temp_tri123->tri_id = quad_id;
	temp_tri123->edge1 = edge1;
	temp_tri123->edge2 = edge2;
	temp_tri123->edge3 = edge3;

	// Find the normal of the face
	temp_tri123->face_normal = geom_param_ptr->get_face_normal(edge1->start_pt->pt_coord(),
		edge2->start_pt->pt_coord(),
		edge3->start_pt->pt_coord());

	// Find the geometric center of the face
	temp_tri123->geom_center = geom_param_ptr->findGeometricCenter(edge1->start_pt->pt_coord(),
		edge2->start_pt->pt_coord(),
		edge3->start_pt->pt_coord());

	temp_quad->tri123 = temp_tri123;

	//___________________________________________________________________________________________
	// Half triangle 2
	tri_store* temp_tri341 = new tri_store;

	temp_tri341->tri_id = quad_id;
	temp_tri341->edge1 = edge4;
	temp_tri341->edge2 = edge5;
	temp_tri341->edge3 = edge6;

	// Find the normal of the face
	temp_tri341->face_normal = geom_param_ptr->get_face_normal(edge4->start_pt->pt_coord(),
		edge5->start_pt->pt_coord(),
		edge6->start_pt->pt_coord());

	// Find the geometric center of the face
	temp_tri341->geom_center = geom_param_ptr->findGeometricCenter(edge4->start_pt->pt_coord(),
		edge5->start_pt->pt_coord(),
		edge6->start_pt->pt_coord());

	temp_quad->tri341 = temp_tri341;

	// Find the normal of the face
	temp_quad->face_normal = glm::normalize(temp_tri123->face_normal + temp_tri341->face_normal);

	// Find the geometric center of the face
	temp_quad->geom_center = (temp_tri123->geom_center + temp_tri341->geom_center) * 0.5f;


	quadMap.push_back(temp_quad);

	// Add to the quad id map
	quadId_Map.insert({ quad_id, quad_count });

	// Iterate the quad count
	quad_count++;

}

tri_store* quad_list_store::get_quadrilateral_face123(const int& quad_id)
{
	// Check whether quad_id exists?
	auto it = quadId_Map.find(quad_id);

	if (it != quadId_Map.end())
	{
		// quad id exists
		// return the address of the triangle
		return quadMap[it->second]->tri123;
	}
	else
	{
		// id not found
		return nullptr;
	}

}

tri_store* quad_list_store::get_quadrilateral_face341(const int& quad_id)
{
	// Check whether quad_id exists?
	auto it = quadId_Map.find(quad_id);

	if (it != quadId_Map.end())
	{
		// quad id exists
		// return the address of the triangle
		return quadMap[it->second]->tri341;
	}
	else
	{
		// id not found
		return nullptr;
	}

}

void quad_list_store::set_buffer()
{

	unsigned int quad_indices_count = 6 * quad_count; // 6 indices to form a quadrilateral ( 3 + 3 triangles)
	unsigned int* quad_vertex_indices = new unsigned int[quad_indices_count];

	unsigned int quad_i_index = 0;

	// Set the quad vertices
	for (auto& quad : quadMap)
	{
		// Add quadrilateral buffers
		get_quad_index_buffer(quad_vertex_indices, quad_i_index);
	}

	VertexBufferLayout quad_pt_layout;
	quad_pt_layout.AddFloat(3);  // Node center
	quad_pt_layout.AddFloat(3);  // Node normal
	quad_pt_layout.AddFloat(1);  // Is Dynamic data
	quad_pt_layout.AddFloat(1);  // Normalized deflection scale


	// Define the tri vertices of the model for a node 6 * (3 position + 3 normal + 2 dynamic data)   
	const unsigned int quad_vertex_count = 6 * 8 * quad_count;
	unsigned int quad_vertex_size = quad_vertex_count * sizeof(float); // Size of the node_vertex

	// Create the quadrilateral buffers
	quad_buffer.CreateDynamicBuffers(quad_vertex_size, quad_vertex_indices, quad_indices_count, quad_pt_layout);

	// Delete the dynamic array
	delete[] quad_vertex_indices;

	update_buffer();

}

void quad_list_store::set_quad_color(const glm::vec3& quad_color, const double& transparency)
{
	// Set the quad color
	quad_shader.setUniform("vertexColor", quad_color);
	quad_shader.setUniform("vertexTransparency", static_cast<float>(transparency));

}


void quad_list_store::paint_static_quadrilaterals()
{
	// Paint all the quadrilaterals
	quad_shader.Bind();
	quad_buffer.Bind();
	is_dynamic = 0.0;
	glDrawElements(GL_TRIANGLES, (6 * quad_count), GL_UNSIGNED_INT, 0);
	quad_buffer.UnBind();
	quad_shader.UnBind();

}


void quad_list_store::paint_dynamic_quadrilaterals()
{
	// Paint all the quadrilaterals
	quad_shader.Bind();
	quad_buffer.Bind();

	// Update the tri buffer data for dynamic drawing
	is_dynamic = 1.0;
	update_buffer();

	glDrawElements(GL_TRIANGLES, (6 * quad_count), GL_UNSIGNED_INT, 0);
	quad_buffer.UnBind();
	quad_shader.UnBind();

}


void quad_list_store::update_buffer()
{

	// Define the tri vertices of the model for a point 6 * (3 position + 3 normal + 2 dynamic data)     
	const unsigned int quad_vertex_count = 6 * 8 * quad_count;
	float* quad_vertices = new float[quad_vertex_count];

	unsigned int quad_v_index = 0;

	// Set the quad vertex buffers
	for (auto& quad : quadMap)
	{
		// Add vertex buffers
		get_quad_vertex_buffer(quad, quad_vertices, quad_v_index);
	}

	unsigned int quad_vertex_size = quad_vertex_count * sizeof(float); // Size of the quad_vertex

	// Update the buffer
	quad_buffer.UpdateDynamicVertexBuffer(quad_vertices, quad_vertex_size);

	// Delete the dynamic vertices array
	delete[] quad_vertices;


}


void quad_list_store::clear_quadrilaterals()
{
	// Delete all the quadrilaterals
	quad_count = 0;

	quadMap.clear();
	quadId_Map.clear();

}

void quad_list_store::update_opengl_uniforms(bool set_modelmatrix, bool set_viewmatrix, bool set_transparency)
{
	if (set_modelmatrix == true)
	{
		// set the transparency
		quad_shader.setUniform("vertexTransparency", static_cast<float>(geom_param_ptr->geom_transparency));

		// set the projection matrix
		quad_shader.setUniform("projectionMatrix", geom_param_ptr->projectionMatrix, false);

		// set the model matrix
		quad_shader.setUniform("modelMatrix", geom_param_ptr->modelMatrix, false);

	}

	if (set_viewmatrix == true)
	{
		glm::mat4 scalingMatrix = glm::mat4(1.0) * static_cast<float>(geom_param_ptr->zoom_scale);
		scalingMatrix[3][3] = 1.0f;

		glm::mat4 viewMatrix = glm::transpose(geom_param_ptr->panTranslation) * geom_param_ptr->rotateTranslation * scalingMatrix;

		// set the pan translation
		quad_shader.setUniform("viewMatrix", viewMatrix, false);
	}


	if (set_transparency == true)
	{
		// set the alpha transparency  static_cast<float>(geom_param_ptr->geom_transparency)
		quad_shader.setUniform("vertexTransparency", static_cast<float>(geom_param_ptr->geom_transparency));
	}

}


void quad_list_store::get_quad_vertex_buffer(quad_store* quad, float* quad_vertices, unsigned int& quad_v_index)
{
	// Get the three node buffer for the shader
	// Point 1
	// Point location
	quad_vertices[quad_v_index + 0] = quad->tri123->edge1->start_pt->x_coord;
	quad_vertices[quad_v_index + 1] = quad->tri123->edge1->start_pt->y_coord;
	quad_vertices[quad_v_index + 2] = quad->tri123->edge1->start_pt->z_coord;

	// Point normal
	quad_vertices[quad_v_index + 3] = quad->face_normal.x;
	quad_vertices[quad_v_index + 4] = quad->face_normal.y;
	quad_vertices[quad_v_index + 5] = quad->face_normal.z;

	// Is dynamic point
	quad_vertices[quad_v_index + 6] = is_dynamic;

	// Normalized deflection scale
	quad_vertices[quad_v_index + 7] = quad->tri123->edge1->start_pt->normalized_defl_scale;

	// Iterate
	quad_v_index = quad_v_index + 8;

	// Point 2
	// Point location
	quad_vertices[quad_v_index + 0] = quad->tri123->edge2->start_pt->x_coord;
	quad_vertices[quad_v_index + 1] = quad->tri123->edge2->start_pt->y_coord;
	quad_vertices[quad_v_index + 2] = quad->tri123->edge2->start_pt->z_coord;

	// Point normal
	quad_vertices[quad_v_index + 3] = quad->face_normal.x;
	quad_vertices[quad_v_index + 4] = quad->face_normal.y;
	quad_vertices[quad_v_index + 5] = quad->face_normal.z;

	// Is dynamic point
	quad_vertices[quad_v_index + 6] = is_dynamic;

	// Normalized deflection scale
	quad_vertices[quad_v_index + 7] = quad->tri123->edge2->start_pt->normalized_defl_scale;

	// Iterate
	quad_v_index = quad_v_index + 8;

	// Point 3
	// Point location
	quad_vertices[quad_v_index + 0] = quad->tri341->edge1->start_pt->x_coord;
	quad_vertices[quad_v_index + 1] = quad->tri341->edge1->start_pt->y_coord;
	quad_vertices[quad_v_index + 2] = quad->tri341->edge1->start_pt->z_coord;

	// Point normal
	quad_vertices[quad_v_index + 3] = quad->face_normal.x;
	quad_vertices[quad_v_index + 4] = quad->face_normal.y;
	quad_vertices[quad_v_index + 5] = quad->face_normal.z;

	// Is dynamic point
	quad_vertices[quad_v_index + 6] = is_dynamic;

	// Normalized deflection scale
	quad_vertices[quad_v_index + 7] = quad->tri341->edge1->start_pt->normalized_defl_scale;

	// Iterate
	quad_v_index = quad_v_index + 8;

	// Point 4
	// Point location
	quad_vertices[quad_v_index + 0] = quad->tri341->edge2->start_pt->x_coord;
	quad_vertices[quad_v_index + 1] = quad->tri341->edge2->start_pt->y_coord;
	quad_vertices[quad_v_index + 2] = quad->tri341->edge2->start_pt->z_coord;

	// Point normal
	quad_vertices[quad_v_index + 3] = quad->face_normal.x;
	quad_vertices[quad_v_index + 4] = quad->face_normal.y;
	quad_vertices[quad_v_index + 5] = quad->face_normal.z;

	// Is dynamic point
	quad_vertices[quad_v_index + 6] = is_dynamic;

	// Normalized deflection scale
	quad_vertices[quad_v_index + 7] = quad->tri341->edge2->start_pt->normalized_defl_scale;

	// Iterate
	quad_v_index = quad_v_index + 8;

}

void quad_list_store::get_quad_index_buffer(unsigned int* quad_vertex_indices, unsigned int& quad_i_index)
{
	//__________________________________________________________________________
	// Add the indices
	// Index 0 1 2 
	quad_vertex_indices[quad_i_index + 0] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 0;

	quad_vertex_indices[quad_i_index + 1] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 1;

	quad_vertex_indices[quad_i_index + 2] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 2;

	// Index 2 3 0 
	quad_vertex_indices[quad_i_index + 3] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 2;

	quad_vertex_indices[quad_i_index + 4] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 3;

	quad_vertex_indices[quad_i_index + 5] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 0;

	// Iterate
	quad_i_index = quad_i_index + 6;

}


