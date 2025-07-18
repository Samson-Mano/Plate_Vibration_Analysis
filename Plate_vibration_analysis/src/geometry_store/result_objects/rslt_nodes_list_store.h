#pragma once
#include "../fe_objects/nodes_list_store.h"
#include "../geometry_objects/obj_mesh_data.h"

struct rslt_node_store
{
	int node_id = 0;
	glm::vec3 node_pt = glm::vec3(0);

	// results (x, y, z)
	std::vector<glm::vec3> node_displ;
	std::vector<double> node_displ_magnitude;
};

class rslt_nodes_list_store
{
public:
	unsigned int rslt_node_count = 0;
	std::unordered_map<int, rslt_node_store> rslt_nodeMap; // Create an unordered_map to store nodes with ID as key


	rslt_nodes_list_store();
	~rslt_nodes_list_store();
	void init(geom_parameters* geom_param_ptr, obj_mesh_data* mesh_data);
	void add_result_node(int& node_id, const glm::vec3& node_pt, const std::vector<glm::vec3>& node_displ,
		const std::vector<double>& node_displ_magnitude);
	void set_max_displacement(const double& rslt_maxdispl);

	void update_modal_response(const int& mode_number, const double& ampl, const double& normalized_ampl);
	void update_pulse_response(const int& time_step);
	void clear_results();

private:
	geom_parameters* geom_param_ptr = nullptr;
	obj_mesh_data* mesh_data = nullptr;

	double rslt_maxdispl = 0.0;
};
