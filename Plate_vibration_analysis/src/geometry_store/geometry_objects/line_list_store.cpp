#include "line_list_store.h"

line_list_store::line_list_store()
{
	// Empty constructor
}

line_list_store::~line_list_store()
{
	// Empty destructor
}

void line_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Create the point shader
	std::filesystem::path shadersPath = geom_param_ptr->resourcePath;

	line_shader.create_shader((shadersPath.string() + "/resources/shaders/mesh_vert_shader.vert").c_str(),
		(shadersPath.string() + "/resources/shaders/mesh_frag_shader.frag").c_str());

	// Default color
	line_shader.setUniform("vertexColor", geom_param_ptr->geom_colors.edge_color);

	// Delete all the labels
	clear_lines();
}




void line_list_store::add_line(const int& line_id, point_store* start_pt, point_store* end_pt, const glm::vec3& line_normal)
{
	// Add to the list
	lineMap.insert({ line_count,  {line_id, start_pt,end_pt, nullptr,nullptr,nullptr , line_normal} });

	// Add to the line id map
	lineId_Map.insert({ line_id, line_count });

	// Iterate the line count
	line_count++;
}

line_store* line_list_store::get_line(const int& line_id)
{
	// Check whether line_id exists?
	auto it = lineId_Map.find(line_id);

	if (it != lineId_Map.end())
	{
		// return the address of the line
		// line id exists
		return &lineMap[it->second];
	}
	else
	{
		// id not found
		return nullptr;
	}
}


void line_list_store::set_buffer()
{

	// Set the buffer for index
	unsigned int line_indices_count = 2 * line_count; // 1 indices to form a point
	unsigned int* line_vertex_indices = new unsigned int[line_indices_count];

	unsigned int line_i_index = 0;

	// Set the line index buffers
	for (auto& ln : lineMap)
	{
		// Add index buffers
		get_line_index_buffer(line_vertex_indices, line_i_index);
	}

	VertexBufferLayout line_pt_layout;
	line_pt_layout.AddFloat(3);  // Node center
	line_pt_layout.AddFloat(3);  // Node normal
	line_pt_layout.AddFloat(1);  // Is Dynamic data
	line_pt_layout.AddFloat(1);  // Normalized deflection scale

	// Define the line vertices of the model for a point 2 * (3 position & 3 normal + 2 dynamic data) 
	const unsigned int line_vertex_count = 2 * 8 * line_count;
	unsigned int line_vertex_size = line_vertex_count * sizeof(float); // Size of the node_vertex

	// Create the line dynamic buffers
	line_buffer.CreateDynamicBuffers(line_vertex_size, line_vertex_indices, line_indices_count, line_pt_layout);

	// Delete the dynamic index array
	delete[] line_vertex_indices;

	// Update the buffer
	update_buffer();

}

void line_list_store::set_line_color(const glm::vec3& line_color)
{
	// Set the line color
	line_shader.setUniform("vertexColor", line_color);

}


void line_list_store::paint_static_lines()
{
	// Paint all the points
	line_shader.Bind();
	line_buffer.Bind();
	is_dynamic = 0.0;
	glDrawElements(GL_LINES, (2 * line_count), GL_UNSIGNED_INT, 0);
	line_buffer.UnBind();
	line_shader.UnBind();
}

void line_list_store::paint_dynamic_lines()
{
	// Paint all the points
	line_shader.Bind();
	line_buffer.Bind();

	// Update the line buffer data for dynamic drawing
	is_dynamic = 1.0;
	update_buffer();

	glDrawElements(GL_LINES, (2 * line_count), GL_UNSIGNED_INT, 0);
	line_buffer.UnBind();
	line_shader.UnBind();

}


void line_list_store::update_buffer()
{

	// Define the line vertices of the model for a point 2 * (3 position & 3 normal + 2 dynamic data) 
	const unsigned int line_vertex_count = 2 * 8 * line_count;
	float* line_vertices = new float[line_vertex_count];

	unsigned int line_v_index = 0;

	// Set the line vertex buffers
	for (auto& ln : lineMap)
	{
		// Add vertex buffers
		get_line_vertex_buffer(ln.second, line_vertices, line_v_index);
	}

	unsigned int line_vertex_size = line_vertex_count * sizeof(float); // Size of the line_vertex

	// Update the buffer
	line_buffer.UpdateDynamicVertexBuffer(line_vertices, line_vertex_size);

	// Delete the dynamic vertices array
	delete[] line_vertices;

}

void line_list_store::clear_lines()
{
	// Delete all the lines
	line_count = 0;
	lineMap.clear();
	lineId_Map.clear();

}

void line_list_store::update_opengl_uniforms(bool set_modelmatrix, bool set_viewmatrix, bool set_transparency)
{
	if (set_modelmatrix == true)
	{
		// set the transparency
		line_shader.setUniform("vertexTransparency", 1.0f);

		// set the projection matrix
		line_shader.setUniform("projectionMatrix", geom_param_ptr->projectionMatrix, false);

		// set the model matrix
		line_shader.setUniform("modelMatrix", geom_param_ptr->modelMatrix, false);
	}

	if (set_viewmatrix == true)
	{
		glm::mat4 scalingMatrix = glm::mat4(1.0) * static_cast<float>(geom_param_ptr->zoom_scale);
		scalingMatrix[3][3] = 1.0f;

		glm::mat4 viewMatrix = glm::transpose(geom_param_ptr->panTranslation) * geom_param_ptr->rotateTranslation * scalingMatrix;

		// set the pan translation
		line_shader.setUniform("viewMatrix", viewMatrix, false);

	}


	if (set_transparency == true)
	{
		// set the alpha transparency
		line_shader.setUniform("vertexTransparency", static_cast<float>(geom_param_ptr->geom_transparency));
	}

}

void line_list_store::get_line_vertex_buffer(line_store& ln, float* line_vertices, unsigned int& line_v_index)
{
	// Get the node buffer for the shader
	// Start Point
	// Point location
	line_vertices[line_v_index + 0] = ln.start_pt->x_coord;
	line_vertices[line_v_index + 1] = ln.start_pt->y_coord;
	line_vertices[line_v_index + 2] = ln.start_pt->z_coord;


	// Point normal
	line_vertices[line_v_index + 3] = ln.line_normal.x;
	line_vertices[line_v_index + 4] = ln.line_normal.y;
	line_vertices[line_v_index + 5] = ln.line_normal.z;

	// Is dynamic point
	line_vertices[line_v_index + 6] = is_dynamic;

	// Normalized deflection scale
	line_vertices[line_v_index + 7] = ln.start_pt->normalized_defl_scale;

	// Iterate
	line_v_index = line_v_index + 8;

	// End Point
	// Point location
	line_vertices[line_v_index + 0] = ln.end_pt->x_coord;
	line_vertices[line_v_index + 1] = ln.end_pt->y_coord;
	line_vertices[line_v_index + 2] = ln.end_pt->z_coord;

	// Point normal
	line_vertices[line_v_index + 3] = ln.line_normal.x;
	line_vertices[line_v_index + 4] = ln.line_normal.y;
	line_vertices[line_v_index + 5] = ln.line_normal.z;

	// Is dynamic point
	line_vertices[line_v_index + 6] = is_dynamic;

	// Normalized deflection scale
	line_vertices[line_v_index + 7] = ln.end_pt->normalized_defl_scale;

	// Iterate
	line_v_index = line_v_index + 8;

}

void line_list_store::get_line_index_buffer(unsigned int* line_vertex_indices, unsigned int& line_i_index)
{
	//__________________________________________________________________________
	// Add the indices
	// Index 1
	line_vertex_indices[line_i_index] = line_i_index;

	line_i_index = line_i_index + 1;

	// Index 2
	line_vertex_indices[line_i_index] = line_i_index;

	line_i_index = line_i_index + 1;

}


