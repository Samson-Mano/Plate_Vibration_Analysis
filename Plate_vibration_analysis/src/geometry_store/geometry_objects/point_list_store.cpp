#include "point_list_store.h"

point_list_store::point_list_store()
{
	// Empty constructor
}

point_list_store::~point_list_store()
{
	// Empty destructor
}

void point_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Create the point shader
	std::filesystem::path shadersPath = geom_param_ptr->resourcePath;

	point_shader.create_shader((shadersPath.string() + "/resources/shaders/mesh_vert_shader.vert").c_str(),
		(shadersPath.string() + "/resources/shaders/mesh_frag_shader.frag").c_str());

	point_shader.setUniform("vertexColor", geom_param_ptr->geom_colors.point_color);

	// Delete all the points
	clear_points();
}

void point_list_store::add_point(const int& point_id, const double& x_coord, const double& y_coord, const double& z_coord)
{
	// Add to the list
	pointMap.insert({ point_count, {point_id, x_coord, y_coord, z_coord} });

	// Add to the point id map
	pointId_Map.insert({ point_id, point_count });

	// Iterate the point count
	point_count++;
}


void point_list_store::update_point(const int& point_id, const double& x_coord, const double& y_coord, const double& z_coord, 
	const double& normalized_defl_scale)
{
	// Get the point associated with the point_id
	point_store* pt = get_point(point_id);

	// Check if point was found
	if (pt != nullptr)
	{
		// Update the point's coordinates
		pt->x_coord = x_coord;
		pt->y_coord = y_coord;
		pt->z_coord = z_coord;

		pt->normalized_defl_scale = normalized_defl_scale;
	}
	else
	{
		// Handle the error: Point not found
		std::cerr << "Error: Point with ID " << point_id << " not found." << std::endl;
	}

}

point_store* point_list_store::get_point(const int& point_id)
{
	// Check whether point_id exists?
	auto it = pointId_Map.find(point_id);

	if (it != pointId_Map.end())
	{
		// Point id exists
		// return the address of the point
		return &pointMap[it->second];
	}
	else
	{
		// id not found
		return nullptr;
	}
}


void point_list_store::set_buffer()
{

	// Set the buffer for index
	unsigned int point_indices_count = 1 * point_count; // 1 indices to form a point
	unsigned int* point_vertex_indices = new unsigned int[point_indices_count];

	unsigned int point_i_index = 0;

	// Set the point index buffers
	for (auto& pt : pointMap)
	{
		// Add index buffer
		get_point_index_buffer(point_vertex_indices, point_i_index);
	}

	VertexBufferLayout node_layout;
	node_layout.AddFloat(3);  // Node center
	node_layout.AddFloat(3);  // Node normal
	node_layout.AddFloat(1);  // Is Dynamic data
	node_layout.AddFloat(1);  // Normalized deflection scale

	// Define the node vertices of the model for a node (3 position & 3 normal + 2 dynamic data) 
	const unsigned int point_vertex_count = 8 * point_count;
	unsigned int point_vertex_size = point_vertex_count * sizeof(float); // Size of the node_vertex

	// Create the point dynamic buffers
	point_buffer.CreateDynamicBuffers(point_vertex_size, point_vertex_indices, point_indices_count, node_layout);

	// Delete the dynamic index array
	delete[] point_vertex_indices;

	// Update the buffer
	update_buffer();

}


void point_list_store::set_point_color(const glm::vec3& point_color)
{
	// Set the point color
	point_shader.setUniform("vertexColor", point_color);

}


void point_list_store::paint_static_points()
{
	// Paint all the static points
	point_shader.Bind();
	point_buffer.Bind();
	is_dynamic = 0.0;
	glDrawElements(GL_POINTS, point_count, GL_UNSIGNED_INT, 0);
	point_buffer.UnBind();
	point_shader.UnBind();
}


void point_list_store::paint_dynamic_points()
{
	// Paint all the points
	point_shader.Bind();
	point_buffer.Bind();

	// Update the point buffer data for dynamic drawing
	is_dynamic = 1.0;
	update_buffer();

	glDrawElements(GL_POINTS, point_count, GL_UNSIGNED_INT, 0);
	point_buffer.UnBind();
	point_shader.UnBind();

}


void point_list_store::update_buffer()
{
	// Define the node vertices of the model for a node (3 position & 3 normal + 2 dynamic data)  
	const unsigned int point_vertex_count = 8 * point_count;
	float* point_vertices = new float[point_vertex_count];

	unsigned int point_v_index = 0;

	// Set the point vertex buffers
	for (auto& pt : pointMap)
	{
		// Add vertex buffers
		get_point_vertex_buffer(pt.second, point_vertices, point_v_index);
	}

	unsigned int point_vertex_size = point_vertex_count * sizeof(float); // Size of the node_vertex

	// Update the buffer
	point_buffer.UpdateDynamicVertexBuffer(point_vertices, point_vertex_size);

	// Delete the dynamic vertices array
	delete[] point_vertices;
}

void point_list_store::clear_points()
{
	// Delete all the points
	point_count = 0;
	pointId_Map.clear();
	pointMap.clear();
}

void point_list_store::update_opengl_uniforms(bool set_modelmatrix, bool set_viewmatrix, bool set_transparency)
{
	if (set_modelmatrix == true)
	{
		// set the transparency
		point_shader.setUniform("vertexTransparency", 1.0f);

		// set the projection matrix
		point_shader.setUniform("projectionMatrix", geom_param_ptr->projectionMatrix, false);

		// set the model matrix
		point_shader.setUniform("modelMatrix", geom_param_ptr->modelMatrix, false);

	}

	if (set_viewmatrix == true)
	{
		glm::mat4 scalingMatrix = glm::mat4(1.0) * static_cast<float>(geom_param_ptr->zoom_scale);
		scalingMatrix[3][3] = 1.0f;

		glm::mat4 viewMatrix = glm::transpose(geom_param_ptr->panTranslation) * geom_param_ptr->rotateTranslation * scalingMatrix;

		// set the view matrix
		point_shader.setUniform("viewMatrix", viewMatrix, false);
	}

	if (set_transparency == true)
	{
		// set the alpha transparency 
		point_shader.setUniform("vertexTransparency", static_cast<float>(geom_param_ptr->geom_transparency));

	}

}


void point_list_store::get_point_vertex_buffer(point_store& pt, float* point_vertices, unsigned int& point_v_index)
{
	// Get the node buffer for the shader
	// Point location
	point_vertices[point_v_index + 0] = pt.x_coord;
	point_vertices[point_v_index + 1] = pt.y_coord;
	point_vertices[point_v_index + 2] = pt.z_coord;

	// Point Normal (Normal to Geometry center)

	point_vertices[point_v_index + 3] = pt.pt_normal.x;
	point_vertices[point_v_index + 4] = pt.pt_normal.y;
	point_vertices[point_v_index + 5] = pt.pt_normal.z;

	point_vertices[point_v_index + 6] = is_dynamic;

	point_vertices[point_v_index + 7] = pt.normalized_defl_scale;

	// Iterate
	point_v_index = point_v_index + 8;

}


void point_list_store::get_point_index_buffer(unsigned int* point_indices, unsigned int& point_i_index)
{
	//__________________________________________________________________________
	// Add the indices
	point_indices[point_i_index] = point_i_index;

	point_i_index = point_i_index + 1;

}

