#pragma once
#include <iostream>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include "../geometry_buffers/gBuffers.h"
#include "../geometry_buffers/font_atlas.h"
#include "../geom_parameters.h"


struct text_store
{
	// Store the individual label
	int label_id = -1;
	std::string label = "";
	glm::vec3 label_loc = glm::vec3(0);
	int label_char_count = 0;

	double label_angle = 0.0;
	bool label_above_loc = false;

	// float total_label_width = 0.0f;
	// float total_label_height = 0.0f;
};


class text_list_store
{
	// Stores all the labels
public:
	unsigned int total_char_count = 0;
	unsigned int text_count = 0;
	std::unordered_map<int, text_store> text_labels;

	text_list_store();
	~text_list_store();

	void init(geom_parameters* geom_param_ptr);
	void add_text(const int& label_id, std::string& label, glm::vec3& label_loc,
		double label_angle, bool above_point);
	void update_text(const int& label_id, std::string& label, glm::vec3& label_loc);

	void set_buffer();
	void update_buffer();
	void set_text_color(const glm::vec3& text_color);

	void paint_static_texts();
	void paint_dynamic_texts();


	void clear_texts();
	void update_opengl_uniforms(bool set_modelmatrix, bool set_viewmatrix, bool set_transparency);
private:
	geom_parameters* geom_param_ptr = nullptr;

	gBuffers text_buffer;
	Shader text_shader;

	void get_label_vertex_buffer(text_store& txt, float* text_vertices, unsigned int& text_v_index);
		
	void get_label_index_buffer(unsigned int* text_vertex_indices, unsigned int& text_i_index);

	glm::vec3 rotate_pt(glm::vec3& rotate_about, glm::vec3 pt, double& rotation_angle);

};

