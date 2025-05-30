#pragma once
#include "nodes_list_store.h"
#include "../geometry_objects/text_list_store.h"

struct load_data
{
	int load_set_id = 0; // Load id
	std::vector<int> node_ids; // ids of the node its applied to
	std::vector<glm::vec3> load_locs; // Load location
	std::vector<glm::vec3> load_normals; // Load normals

	double load_value = 0.0; // Load value
	double load_start_time = 0.0; // Load start time
	double load_end_time = 0.0; // Load end time
	// bool show_load_label = false;

};

class nodeload_list_store
{
public:
	const double epsilon = 0.000001;
	int load_count = 0;
	int total_load_count = 0;
	std::vector<load_data> loadMap;
	int model_type = 0; // 0 - Circle, 1,2,3 - Rectangle

	nodeload_list_store();
	~nodeload_list_store();
	void init(geom_parameters* geom_param_ptr);
	void set_zero_condition(const int& model_type);

	void add_loads(std::vector<int>& node_ids, std::vector<glm::vec3>& load_locs, std::vector<glm::vec3>& load_normals, double& load_start_time,
		double& load_end_time, double& load_value);
	void delete_load(int& node_id);
	void set_buffer();

	void paint_loads();
	void paint_load_labels();
	void update_geometry_matrices(bool set_modelmatrix, bool set_viewmatrix, bool set_transparency);

	int get_unique_load_id(std::vector<int>& all_ids);
private:
	geom_parameters* geom_param_ptr = nullptr;
	line_list_store load_lines;
	point_list_store load_points;

	// label_list_store load_value_labels;

	double load_max = 0.0;
	std::vector<int> all_load_ids;

	std::pair<glm::vec3, glm::vec3> findOrthogonalVectors(const glm::vec3& v);

	glm::vec3 rotateVector(const glm::vec3& v, const glm::vec3& axis, float angleRadians);

};
