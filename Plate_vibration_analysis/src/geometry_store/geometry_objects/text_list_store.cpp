#include "text_list_store.h"

text_list_store::text_list_store()
{
	// Empty constructor

}

text_list_store::~text_list_store()
{
	// Empty destructor

}

void text_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameter pointer
	this->geom_param_ptr = geom_param_ptr;

	// Create the label shader
	std::filesystem::path shadersPath = geom_param_ptr->resourcePath;

	text_shader.create_shader((shadersPath.string() + "/resources/shaders/text_vert_shader.vert").c_str(),
		(shadersPath.string() + "/resources/shaders/text_frag_shader.frag").c_str());

	// Set texture uniform variables
	text_shader.setUniform("u_Texture", 0);


	// Delete all the labels
	text_labels.clear();
	total_char_count = 0;
	text_count = 0;

}

void text_list_store::add_text(const int& label_id, std::string& label, glm::vec3& label_loc, glm::vec3& label_color)
{
	// Create a temporary element
	text_store temp_label;
	temp_label.label_id = label_id; // label id
	temp_label.label = label; // label string
	temp_label.label_loc = label_loc; // label location 
	temp_label.label_color = label_color; // label color

	temp_label.label_char_count = static_cast<unsigned int>(label.size());

	//___________________________________________________________________
	// Add to the list
	text_labels.insert({ label_id, temp_label });


	// Add to the char_count
	total_char_count = total_char_count + static_cast<unsigned int>(label.size());

	text_count++;
}



void text_list_store::update_text(const int& label_id, std::string& label, glm::vec3& label_loc, glm::vec3& label_color)
{
	// Find the label from id
	// Check whether the text_id is already there
	if (text_labels.find(label_id) != text_labels.end())
	{
		// text ID already exist proceed to update

		int text_char_count = static_cast<unsigned int>(text_labels[label_id].label.size());

		// Curtail the length of text
		if (static_cast<unsigned int>(label.size()) > text_char_count)
		{
			label = label.substr(0, text_char_count);
		}


		text_labels[label_id].label = label;
		text_labels[label_id].label_loc = label_loc;
		text_labels[label_id].label_color = label_color;

	}

}



void text_list_store::set_buffer()
{

	// 6 indices to form a quadrilateral (2 trianlges) which is used for texture mapping
	unsigned int label_indices_count = 6 * total_char_count;
	unsigned int* label_vertex_indices = new unsigned int[label_indices_count];

	unsigned int label_i_index = 0;


	for (int j = 0; j < text_labels.size(); j++)
	{
		for (int i = 0; text_labels[j].label[i] != '\0'; ++i)
		{
			// Create the index buffers for every individual character
			get_label_index_buffer(label_vertex_indices, label_i_index);
		}
	}

	// Create a layout
	VertexBufferLayout label_layout;
	label_layout.AddFloat(3);  // Label location
	label_layout.AddFloat(2);  // Character Position
	label_layout.AddFloat(2);  // Texture glyph coordinate
	label_layout.AddFloat(3);  // Label color

	// Define the label vertices of the model for the entire label
	// (4 vertex (to form a triangle) * ( 3 label location +  2 character position +  2 Texture Glyph coordinate + 3 label color) 
	const unsigned int label_vertex_count = 4 * 10 * total_char_count;
	unsigned int label_vertex_size = label_vertex_count * sizeof(float); // Size of the node_vertex

	// Create the text dynamic buffers
	text_buffer.CreateDynamicBuffers(label_vertex_size, label_vertex_indices, label_indices_count, label_layout);

	// Delete the dynamic index array
	delete[] label_vertex_indices;

	update_buffer();


}


void text_list_store::paint_static_texts()
{
	// Paint all the labels
	text_shader.Bind();
	text_buffer.Bind();

	glActiveTexture(GL_TEXTURE0);
	//// Bind the texture to the slot
	glBindTexture(GL_TEXTURE_2D, geom_param_ptr->main_font.textureID);

	glDrawElements(GL_TRIANGLES, 6 * total_char_count, GL_UNSIGNED_INT, 0);

	glBindTexture(GL_TEXTURE_2D, 0);

	text_buffer.UnBind();
	text_shader.UnBind();
}

void text_list_store::paint_dynamic_texts()
{
	// Paint all the labels
	text_shader.Bind();
	text_buffer.Bind();

	// Update the label buffer data for dynamic drawing
	update_buffer();

	glActiveTexture(GL_TEXTURE0);
	//// Bind the texture to the slot
	glBindTexture(GL_TEXTURE_2D, geom_param_ptr->main_font.textureID);

	glDrawElements(GL_TRIANGLES, 6 * total_char_count, GL_UNSIGNED_INT, 0);

	glBindTexture(GL_TEXTURE_2D, 0);

	text_buffer.UnBind();
	text_shader.UnBind();

}


void text_list_store::update_buffer()
{

	// Define the label vertices of the model for the entire label
	// (4 vertex (to form a triangle) * ( 3 label location +  2 character position +  2 Texture Glyph coordinate + 3 label color)  
	const unsigned int label_vertex_count = 4 * 10 * total_char_count;
	float* label_vertices = new float[label_vertex_count];

	unsigned int label_v_index = 0;

	for (int j = 0; j < text_labels.size(); j++)
	{
		// Set the label vertex buffers
		get_label_vertex_buffer(text_labels[j], label_vertices, label_v_index);
	}

	unsigned int label_vertex_size = label_vertex_count * sizeof(float); // Size of the tri_vertex

	// Update the buffer
	text_buffer.UpdateDynamicVertexBuffer(label_vertices, label_vertex_size);

	// Delete the dynamic vertices array
	delete[] label_vertices;

}


void text_list_store::clear_texts()
{
	// Delete all the labels
	// labels.clear();
	total_char_count = 0;

}

void text_list_store::update_opengl_uniforms(bool set_modelmatrix, bool set_viewmatrix, bool set_transparency)
{
	if (set_modelmatrix == true)
	{
		// set the transparency
		text_shader.setUniform("vertexTransparency", 1.0f);

		// set the projection matrix
		text_shader.setUniform("projectionMatrix", geom_param_ptr->projectionMatrix, false);

		// set the model matrix
		text_shader.setUniform("modelMatrix", geom_param_ptr->modelMatrix, false);

	}

	if (set_viewmatrix == true)
	{

		glm::mat4 scalingMatrix = glm::mat4(1.0) * static_cast<float>(geom_param_ptr->zoom_scale);
		scalingMatrix[3][3] = 1.0f;

		glm::mat4 viewMatrix = glm::transpose(geom_param_ptr->panTranslation) * geom_param_ptr->rotateTranslation * scalingMatrix;

		// set the pan translation
		text_shader.setUniform("viewMatrix", viewMatrix, false);
	}


	if (set_transparency == true)
	{
		// set the alpha transparency
		text_shader.setUniform("vertexTransparency", static_cast<float>(geom_param_ptr->geom_transparency));
	}

}

void text_list_store::get_label_vertex_buffer(text_store& txt, float* text_vertices, unsigned int& text_v_index)
{
	float font_scale = static_cast<float>(geom_param_ptr->font_size / geom_param_ptr->geom_scale);
	float x = 0; // To advance the character width

	for (int i = 0; txt.label[i] != '\0'; ++i)
	{
		// get the atlas information
		char ch = txt.label[i];

		Character ch_data = geom_param_ptr->main_font.ch_atlas[ch];

		float xpos = x + (ch_data.Bearing.x * font_scale);
		float ypos = (20.0 * font_scale) - (ch_data.Size.y - ch_data.Bearing.y) * font_scale;

		float w = ch_data.Size.x * font_scale;
		float h = ch_data.Size.y * font_scale;

		float margin = 0.000002f; // This value prevents the minor overlap with the next char when rendering

		// Point 1
		// Vertices [0,0] // 0th point

		// Label location
		text_vertices[text_v_index + 0] = txt.label_loc.x;
		text_vertices[text_v_index + 1] = txt.label_loc.y;
		text_vertices[text_v_index + 2] = txt.label_loc.z;
		
		// Char location
		text_vertices[text_v_index + 3] = xpos;
		text_vertices[text_v_index + 4] = ypos + h;

		// Texture Glyph coordinate
		text_vertices[text_v_index + 5] = ch_data.top_left.x + margin;
		text_vertices[text_v_index + 6] = ch_data.top_left.y;

		// Label color
		text_vertices[text_v_index + 7] = txt.label_color.x;
		text_vertices[text_v_index + 8] = txt.label_color.y;
		text_vertices[text_v_index + 9] = txt.label_color.z;


		// Iterate
		text_v_index = text_v_index + 10;

		//__________________________________________________________________________________________

		// Point 2
		// Vertices [0,1] // 1th point
	
		// Label location
		text_vertices[text_v_index + 0] = txt.label_loc.x;
		text_vertices[text_v_index + 1] = txt.label_loc.y;
		text_vertices[text_v_index + 2] = txt.label_loc.z;

		// Char location
		text_vertices[text_v_index + 3] = xpos;
		text_vertices[text_v_index + 4] = ypos;

		// Texture Glyph coordinate
		text_vertices[text_v_index + 5] = ch_data.top_left.x + margin;
		text_vertices[text_v_index + 6] = ch_data.bot_right.y;

		// Label color
		text_vertices[text_v_index + 7] = txt.label_color.x;
		text_vertices[text_v_index + 8] = txt.label_color.y;
		text_vertices[text_v_index + 9] = txt.label_color.z;


		// Iterate
		text_v_index = text_v_index + 10;

		//__________________________________________________________________________________________

		// Point 3
		// Vertices [1,1] // 2th point

		// Label location
		text_vertices[text_v_index + 0] = txt.label_loc.x;
		text_vertices[text_v_index + 1] = txt.label_loc.y;
		text_vertices[text_v_index + 2] = txt.label_loc.z;

		// Char location
		text_vertices[text_v_index + 3] = xpos + w;
		text_vertices[text_v_index + 4] = ypos;

		// Texture Glyph coordinate
		text_vertices[text_v_index + 5] = ch_data.bot_right.x - margin;
		text_vertices[text_v_index + 6] = ch_data.bot_right.y;

		// Label color
		text_vertices[text_v_index + 7] = txt.label_color.x;
		text_vertices[text_v_index + 8] = txt.label_color.y;
		text_vertices[text_v_index + 9] = txt.label_color.z;


		// Iterate
		text_v_index = text_v_index + 10;

		//__________________________________________________________________________________________

		// Point 4
		// Vertices [1,0] // 3th point
		
		// Label location
		text_vertices[text_v_index + 0] = txt.label_loc.x;
		text_vertices[text_v_index + 1] = txt.label_loc.y;
		text_vertices[text_v_index + 2] = txt.label_loc.z;

		// Char location
		text_vertices[text_v_index + 3] = xpos + w;
		text_vertices[text_v_index + 4] = ypos + h;

		// Texture Glyph coordinate
		text_vertices[text_v_index + 5] = ch_data.bot_right.x - margin;
		text_vertices[text_v_index + 6] = ch_data.top_left.y;

		// Label color
		text_vertices[text_v_index + 7] = txt.label_color.x;
		text_vertices[text_v_index + 8] = txt.label_color.y;
		text_vertices[text_v_index + 9] = txt.label_color.z;


		// Iterate
		text_v_index = text_v_index + 10;

		//__________________________________________________________________________________________
		x += (ch_data.Advance >> 6) * font_scale;

	}

}

void text_list_store::get_label_index_buffer(unsigned int* text_vertex_indices, unsigned int& text_i_index)
{

	// Fix the index buffers
	// Set the node indices
	unsigned int t_id = ((text_i_index / 6) * 4);

	// Triangle 0,1,2
	text_vertex_indices[text_i_index + 0] = t_id + 0;
	text_vertex_indices[text_i_index + 1] = t_id + 1;
	text_vertex_indices[text_i_index + 2] = t_id + 2;

	// Triangle 2,3,0
	text_vertex_indices[text_i_index + 3] = t_id + 2;
	text_vertex_indices[text_i_index + 4] = t_id + 3;
	text_vertex_indices[text_i_index + 5] = t_id + 0;

	// Increment
	text_i_index = text_i_index + 6;

}










