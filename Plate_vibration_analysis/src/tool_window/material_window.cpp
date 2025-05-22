#include "material_window.h"

material_window::material_window()
{
	// Empty constructor

}

material_window::~material_window()
{
	// Empty destructor

}

void material_window::init()
{
	is_show_window = false;

}

void material_window::render_window()
{
	if (is_show_window == false)
		return;

	if (material_list.size() == 0)
	{
		is_show_window = false;
		return;
	}



	//_____________________________________________________________________________________________________________
	ImGui::Begin("Materials");

	// Convert material_list to a vector of const char*
	std::vector<const char*> material_names;
	std::unordered_map<int, int> material_id_selected_option;
	int i = 0;

	for (const auto& mat : material_list)
	{
		material_id_selected_option[i] = mat.first;
		material_names.push_back(mat.second.material_name.c_str());
		i++;
	}

	ImGui::ListBox("Select Material", &selected_list_option, material_names.data(), static_cast<unsigned int>(material_names.size()), 4);

	ImGui::Spacing();

	selected_material_option = material_id_selected_option[selected_list_option];
	// Get selected material
	const material_data& selected_material_data = material_list[selected_material_option];

	// Get the color for this material
	glm::vec3 std_color = geom_parameters::get_standard_color(selected_material_data.material_id);
	ImVec4 text_color = ImVec4(std_color.x, std_color.y, std_color.z, 1.0f);

	ImGui::TextColored(text_color, "Material ID: %i", selected_material_data.material_id);
	ImGui::TextColored(text_color, "Selected Material: %s", selected_material_data.material_name.c_str());
	ImGui::TextColored(text_color, "Young's Modulus E: %.4e", selected_material_data.material_youngsmodulus);
	ImGui::TextColored(text_color, "Shear Modulus G: %.4e", selected_material_data.material_shearmodulus);
	ImGui::TextColored(text_color, "Material Density: %.4e", selected_material_data.material_density);
	ImGui::TextColored(text_color, "Shell thickness t: %.3f", selected_material_data.shell_thickness);
	ImGui::TextColored(text_color, "Poissons ratio: %.3f", selected_material_data.poissons_ratio);


	// Diable delete if the selected option is Default (0)
	const bool is_delete_button_disabled = selected_list_option == 0 ? true : false;
	ImGui::BeginDisabled(is_delete_button_disabled);
	if (ImGui::Button("Delete Material")) {
		// Delete material
		execute_delete_materialid = selected_material_data.material_id;
		material_list.erase(selected_material_data.material_id);
		selected_list_option = 0;
	}
	ImGui::EndDisabled();


	// List the selected Elements
		//__________________________________________________________________________________________
	// Selected Element list
	ImGui::Spacing();
	ImGui::Spacing();

	static char elementNumbers[1024] = ""; // Increase the buffer size to accommodate more characters

	geom_parameters::copyNodenumberlistToCharArray(selected_elements, elementNumbers, 1024);

	ImGui::Text("Selected Elements: ");
	ImGui::Spacing();

	// Begin a child window with ImGuiWindowFlags_HorizontalScrollbar to enable vertical scrollbar ImGuiWindowFlags_AlwaysVerticalScrollbar
	ImGui::BeginChild("Element Numbers", ImVec2(-1.0f, ImGui::GetTextLineHeight() * 10), true);

	// Assuming 'elementNumbers' is a char array or a string
	ImGui::TextWrapped("%s", elementNumbers);

	// End the child window
	ImGui::EndChild();

	// Add some spacing before the "Create Material" header
	ImGui::Spacing();
	ImGui::Spacing();

	// Assign material dropdown
		// Apply element properties button
	if (ImGui::Button("Apply"))
	{
		apply_element_properties = true; // set the flag to apply to the constraint
	}

	ImGui::Spacing();



	// Create material dropdown
	if (ImGui::CollapsingHeader("Create Material "))
	{
		static char new_material_name[256] = "New Material";
		static double new_material_youngs_modulus = 210000.0;
		static double new_material_shear_modulus = 80000.0;
		static double new_material_density = 7.865 * std::pow(10, -9);
		static double new_material_shell_thickness = 10.0; //mm
		static double new_material_poissons_ratio = 0.3; 


		ImGui::InputText("Material Name", new_material_name, IM_ARRAYSIZE(new_material_name));

		ImGui::InputDouble("Young's Modulus", &new_material_youngs_modulus, 0, 0, "%.4e");
		ImGui::InputDouble("Shear Modulus", &new_material_shear_modulus, 0, 0, "%.4e");
		ImGui::InputDouble("Material Density", &new_material_density, 0, 0, "%.4e");
		ImGui::InputDouble("Shell thickness", &new_material_shell_thickness, 0, 0, "%.3f");
		ImGui::InputDouble("Poissons ratio", &new_material_poissons_ratio, 0, 0, "%.3f");

		if (ImGui::Button("Create Material"))
		{
			// TODO: Add the new material to the material list
			  // Add the new material to the material list
			material_data new_material;
			new_material.material_id = get_unique_material_id();
			new_material.material_name = new_material_name;
			new_material.material_youngsmodulus = new_material_youngs_modulus;
			new_material.material_shearmodulus = new_material_shear_modulus;
			new_material.material_density = new_material_density;
			new_material.shell_thickness = new_material_shell_thickness;
			new_material.poissons_ratio = new_material_poissons_ratio;
		
			
			material_list[new_material.material_id] = new_material;

			// Update the combo box
			selected_list_option = material_list.size() - 1;
		}
	}


	ImGui::Spacing();

	// Add a "Close" button
	if (ImGui::Button("Close"))
	{
		// Clear the selected elements
		this->selected_elements.clear();
		is_selected_count = false; // Number of selected elements 0
		is_selection_changed = false; // Set the selection changed

		apply_element_properties = false;
		is_show_window = false;
	}

	ImGui::End();


}


int material_window::get_unique_material_id()
{
	// Add all the ids to a int list
	std::vector<int> all_ids;
	for (auto& mat : material_list)
	{
		all_ids.push_back(mat.first);
	}


	if (all_ids.size() != 0)
	{
		int i;
		std::sort(all_ids.begin(), all_ids.end());

		// Find if any of the nodes are missing in an ordered int
		for (i = 0; i < all_ids.size(); i++)
		{
			if (all_ids[i] != i)
			{
				return i;
			}
		}

		// no node id is missing in an ordered list so add to the end
		return all_ids.size();
	}

	// id for the first node is 0
	return 0;
}



void material_window::add_to_element_list(const std::vector<int>& selected_elements, const bool& is_right)
{
	if (is_right == false)
	{
		// Add to the selected elements list
		for (int elmnt : selected_elements)
		{
			// Check whether elements are already in the list or not
			if (std::find(this->selected_elements.begin(), this->selected_elements.end(), elmnt) == this->selected_elements.end())
			{
				// Add to selected elements
				this->selected_elements.push_back(elmnt);

				// Selection changed flag
				this->is_selection_changed = true;
			}
		}
	}
	else
	{
		// Remove from the selected elements list
		for (int elmnt : selected_elements)
		{
			// Erase the elements which is found in the list
			this->selected_elements.erase(std::remove(this->selected_elements.begin(), this->selected_elements.end(), elmnt),
				this->selected_elements.end());

			// Selection changed flag
			this->is_selection_changed = true;
		}
	}

	// Number of selected elements
	this->is_selected_count = false;
	if (static_cast<int>(this->selected_elements.size()) > 0)
	{
		this->is_selected_count = true;
	}
}



