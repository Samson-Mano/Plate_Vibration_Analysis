#include "rslt_nodes_list_store.h"

rslt_nodes_list_store::rslt_nodes_list_store()
{
	// Empty constructor
}

rslt_nodes_list_store::~rslt_nodes_list_store()
{
	// Empty destructor
}

void rslt_nodes_list_store::init(geom_parameters* geom_param_ptr, obj_mesh_data* mesh_data)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;
	this->mesh_data = mesh_data;

	// Clear the node results
	clear_results();

}

void rslt_nodes_list_store::add_result_node(int& node_id, const glm::vec3& node_pt, 
	const std::vector<glm::vec3>& node_displ, const std::vector<double>& node_displ_magnitude)
{
	// Add result nodes
	rslt_node_store temp_node;
	temp_node.node_id = node_id;
	temp_node.node_pt = node_pt;
	temp_node.node_displ = node_displ;
	temp_node.node_displ_magnitude = node_displ_magnitude;

	//// Check whether the node_id is already there
	//if (modal_nodeMap.find(node_id) != modal_nodeMap.end())
	//{
	//	// Node ID already exist (do not add)
	//	return;
	//}

	// Insert to the nodes
	rslt_nodeMap.insert({ node_id, temp_node });
	rslt_node_count++;

	//__________________________ Add the result node points
	this->mesh_data->add_mesh_point(node_id, node_pt.x, node_pt.y, node_pt.z);

}

void rslt_nodes_list_store::set_max_displacement(const double& rslt_maxdispl)
{
	// set the node result maximum displacement
	this->rslt_maxdispl = rslt_maxdispl;

}

void rslt_nodes_list_store::update_modal_response(const int& mode_number, const double& ampl, const double& normalized_ampl)
{
	// Update the node point for modal response 
	for (const auto& nd : rslt_nodeMap)
	{
		glm::vec3 node_displacement =  nd.second.node_displ[mode_number] * static_cast<float>( ampl);
		double node_displacement_mag = nd.second.node_displ_magnitude[mode_number];

		// Displacement offset
		glm::vec3 node_pt_offset = nd.second.node_pt + node_displacement;

		this->mesh_data->update_mesh_point(nd.second.node_id, node_pt_offset.x, node_pt_offset.y, node_pt_offset.z);

	}
	
}


void rslt_nodes_list_store::update_pulse_response(const int& time_step)
{
	// Update the node point for pulse response 
	for (const auto& nd : rslt_nodeMap)
	{
		glm::vec3 node_displacement = nd.second.node_displ[time_step];
		double node_displacement_mag = nd.second.node_displ_magnitude[time_step];

		// Displacement offset
		glm::vec3 node_pt_offset = nd.second.node_pt + node_displacement;

		this->mesh_data->update_mesh_point(nd.second.node_id, node_pt_offset.x, node_pt_offset.y, node_pt_offset.z);

	}

}

void rslt_nodes_list_store::clear_results()
{
	// Clear the results
	rslt_node_count = 0;
	rslt_nodeMap.clear();
	rslt_maxdispl = 0.0;

}