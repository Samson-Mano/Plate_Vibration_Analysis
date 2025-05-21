#include "constraint_window.h"

constraint_window::constraint_window()
{
	// Empty constructor
}

constraint_window::~constraint_window()
{
	// Empty destructor
}

void constraint_window::init()
{
	// Initialize

}

void constraint_window::render_window()
{
	if (is_show_window == false)
		return;

	ImGui::Begin("Loads");

	//_________________________________________________________________________________________
	// Constraint type 0 - Fixed 1 - Pinned

	// Add your constraint input controls here
	// Option to select the types of support
	// Define an array of options
	const int options_count = 2;
	const char* options[] = { "Fixed support", "Pinned support", };

	// Define a string to hold the label for the popup select button
	std::string popupLabel = "Support: ";

	if (ImGui::Button((popupLabel + options[constraint_selectedOptionIndex]).c_str())) {
		ImGui::OpenPopup("Select an option");
	}

	if (ImGui::BeginPopup("Select an option")) {
		ImGui::Text("- Constraint Type -");
		ImGui::Separator();

		for (int i = 0; i < options_count; i++) {
			if (ImGui::Selectable(options[i], constraint_selectedOptionIndex == i)) {
				constraint_selectedOptionIndex = i;
			}
		}

		ImGui::EndPopup();
	}


	//_________________________________________________________________________________________

	// Selected Node list
	ImGui::Spacing();

	static char nodeNumbers[1024] = ""; // Increase the buffer size to accommodate more characters

	geom_parameters::copyNodenumberlistToCharArray(selected_nodes, nodeNumbers, 1024);

	ImGui::Text("Selected Nodes: ");
	ImGui::Spacing();

	// Begin a child window with ImGuiWindowFlags_HorizontalScrollbar to enable vertical scrollbar ImGuiWindowFlags_AlwaysVerticalScrollbar
	ImGui::BeginChild("Node Numbers", ImVec2(-1.0f, ImGui::GetTextLineHeight() * 10), true);

	// Assuming 'nodeNumbers' is a char array or a string
	ImGui::TextWrapped("%s", nodeNumbers);

	// End the child window
	ImGui::EndChild();

	//__________________________________________________________________________________________
	// Apply and Delete Button
	// Apply constraint button
	if (ImGui::Button("Apply"))
	{
		apply_nodal_constraint = true; // set the flag to apply to the constraint
	}

	ImGui::SameLine();

	// Delete constraint button
	if (ImGui::Button("Delete"))
	{
		delete_nodal_constraint = true; // set the flag to apply to the constraint
	}

	//__________________________________________________________________________________________
	ImGui::Spacing();

	// Close button
	if (ImGui::Button("Close"))
	{
		// Clear the selected nodes
		this->selected_nodes.clear();
		is_selected_count = false; // Number of selected nodes 0
		is_selection_changed = false; // Set the selection changed

		apply_nodal_constraint = false;
		delete_nodal_constraint = false;
		is_show_window = false; // set the flag to close the window
	}

	//__________________________________________________________________________________________

	ImGui::End();

}

void constraint_window::add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right)
{
	if (is_right == false)
	{
		// Add to the selected node list
		for (int node : selected_nodes)
		{
			// Check whether nodes are already in the list or not
			if (std::find(this->selected_nodes.begin(), this->selected_nodes.end(), node) == this->selected_nodes.end())
			{
				// Add to selected nodes
				this->selected_nodes.push_back(node);

				// Selection changed flag
				this->is_selection_changed = true;
			}
		}
	}
	else
	{
		// Remove from the selected node list
		for (int node : selected_nodes)
		{
			// Erase the node which is found in the list
			this->selected_nodes.erase(std::remove(this->selected_nodes.begin(), this->selected_nodes.end(), node),
				this->selected_nodes.end());

			// Selection changed flag
			this->is_selection_changed = true;
		}

	}

	// Number of selected nodes
	this->is_selected_count = false;
	if (static_cast<int>(this->selected_nodes.size()) > 0)
	{
		this->is_selected_count = true;
	}

}
