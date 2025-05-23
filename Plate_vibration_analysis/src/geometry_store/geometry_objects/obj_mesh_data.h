#pragma once
#include "point_list_store.h"
#include "line_list_store.h"
#include "tri_list_store.h"
#include "quad_list_store.h"
#include "text_list_store.h"

class obj_mesh_data
{
public:

	obj_mesh_data();
	~obj_mesh_data();

	void init(geom_parameters* geom_param_ptr);

	void add_mesh_point(const int& point_id,
		const double& x_coord,
		const double& y_coord,
		const double& z_coord);

	void add_selected_points(const std::vector<int>& selected_point_id);

	void add_selected_tris(const std::vector<int>& selected_tri_id);

	void add_selected_quads(const std::vector<int>& selected_quad_id);

	void add_mesh_tris(const int& tri_id,
		const int& point_id1,
		const int& point_id2,
		const int& point_id3);

	void add_mesh_quads(const int& quad_id,
		const int& point_id1,
		const int& point_id2,
		const int& point_id3,
		const int& point_id4);

	void update_mesh_point(const int& point_id,
		const double& x_coord,
		const double& y_coord, 
		const double& z_coord);

	void set_mesh_node_normals();

	void set_mesh_wireframe();

	glm::vec3 get_mesh_node_normals(const int& point_id);


	// void update_mesh_buffer();
	
	// glm::vec3 get_element_normal(const int& id, const int& type);


	void update_mesh_color(const glm::vec3& point_color, const glm::vec3& line_color, const glm::vec3& tri_color, const double& transparency);

	void set_buffer();

	void clear_mesh();

	void paint_static_mesh();
	void paint_static_mesh_boundaries();
	void paint_static_mesh_points();
	
	void paint_dynamic_mesh();
	void paint_dynamic_mesh_boundaries();
	void paint_dynamic_mesh_points();

	void paint_selected_points();
	void paint_selected_mesh();
	void paint_mesh_normals();

	void update_opengl_uniforms(bool set_modelmatrix, bool set_viewmatrix, bool set_transparency);

private:

	geom_parameters* geom_param_ptr = nullptr;

	point_list_store mesh_points;
	point_list_store selected_mesh_points;
	line_list_store mesh_boundaries;
	line_list_store mesh_normals;
	tri_list_store mesh_tris;
	tri_list_store selected_mesh_tris;
	quad_list_store mesh_quads;
	quad_list_store selected_mesh_quads;

	int half_edge_count = 0;
	std::vector<line_store*> mesh_half_edges; // All the Half edge data

	int add_half_edge(const int& startPt_id, const int& endPt_id);

	void set_mesh_edge(line_store* edge, int& line_id, std::set<std::pair<int, int>>& seenLines);

	void set_mesh_normal_vector(tri_store* tri);

	void set_mesh_normal_vector(quad_store* quad);

};