#pragma once
#include "nodes_list_store.h"
#include "elementline_list_store.h"

struct nodecnst_data
{
	int node_id = 0;
	glm::vec3 ndcnst_loc = glm::vec3(0);
	glm::vec3 ndcnst_normals = glm::vec3(0);

	int constraint_type = -1; // 0 - Fixed support, 1 - Pinned support

};


class nodecnst_list_store
{
public:
	const double epsilon = 0.000001;
	unsigned int ndcnst_count = 0;
	std::unordered_map<int, nodecnst_data> ndcnstMap;
	int model_type = 0; // 0,1 - Line, 2,3 - Circle

	nodecnst_list_store();
	~nodecnst_list_store();
	void init(geom_parameters* geom_param_ptr);
	void set_zero_condition(const int& model_type);
	void add_nodeconstraint(int& node_id, glm::vec3& ndcnst_loc, glm::vec3& ndcnst_normals, int& constraint_type);
	void delete_nodeconstraint(int& node_id);
	void set_buffer();
	void paint_constraint();
	
	void update_geometry_matrices(bool set_modelmatrix, bool set_viewmatrix, bool set_transparency);

private:

	geom_parameters* geom_param_ptr = nullptr;
	point_list_store ndcnst_points;
	line_list_store ndcnst_lines;

	std::pair<glm::vec3, glm::vec3> findOrthogonalVectors(const glm::vec3& v);

	glm::vec3 rotateVector(const glm::vec3& v, const glm::vec3& axis, float angleRadians);


};