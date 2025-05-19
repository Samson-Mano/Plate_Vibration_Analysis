#include "new_model_window.h"

new_model_window::new_model_window()
{
	// Empty constructor
}

new_model_window::~new_model_window()
{
	// Empty destructor
}

void new_model_window::init()
{
	is_show_window = false;
	execute_create_model = false;

}

void new_model_window::render_window()
{
	if (is_show_window == false)
		return;

	ImGui::Begin("New Model");


	ImGui::Text("Enter path to model file:");

	// C:\Users\HFXMSZ\OneDrive - LR\Documents\Programming\Other programs\002_Main_program\Plate_vibration_analysis\Plate_vibration_analysis\example_model\circle_mesh.txt

	ImGui::InputText("Model File Path", file_path_buffer, IM_ARRAYSIZE(file_path_buffer));

	ImGui::SameLine();
	if (ImGui::Button("Browse..."))
	{
		std::string selected = ShowOpenFileDialog();
		if (!selected.empty())
		{
			// Copy to file_path_buffer safely
			strncpy_s(file_path_buffer, selected.c_str(), sizeof(file_path_buffer));
			file_path_buffer[sizeof(file_path_buffer) - 1] = '\0'; // Ensure null-termination
		}
	}



	ImGui::Spacing();



	//_____________________________________________________________________________________________________________________________________________________________________
	ImGui::Spacing();

	if (ImGui::Button("Create Model"))
	{


		std::ifstream model_file(file_path_buffer, std::ifstream::in);

		if (!model_file)
		{
			std::cerr << "Failed to open file: " << file_path_buffer << std::endl;
		}
		else
		{
			// Read or parse the file here
			// Read the Raw Data
			// Read the entire file into a string
			std::string file_contents((std::istreambuf_iterator<char>(model_file)),
				std::istreambuf_iterator<char>());

			// Split the string into lines
			std::istringstream iss(file_contents);
			std::string line;

			this->data_lines.clear();

			while (std::getline(iss, line))
			{
				this->data_lines.push_back(line);
			}

			model_file.close();

			execute_create_model = true;

		}
	}

	//__________________________________________________________________________________________
	ImGui::Spacing();

	// Close button
	if (ImGui::Button("Close"))
	{
		is_show_window = false;
		execute_create_model = false;
	}

	//__________________________________________________________________________________________

	ImGui::End();

}



std::string new_model_window::ShowOpenFileDialog()
{
	OPENFILENAMEW ofn;
	wchar_t fileName[MAX_PATH] = L"";

	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.lpstrFile = fileName;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrFilter = L"Text Files (*.txt)\0*.txt\0All Files (*.*)\0*.*\0";
	ofn.nFilterIndex = 1;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	if (GetOpenFileNameW(&ofn))
	{
		int bufferSize = WideCharToMultiByte(CP_UTF8, 0, ofn.lpstrFile, -1, nullptr, 0, nullptr, nullptr);
		std::string result(bufferSize, '\0');
		WideCharToMultiByte(CP_UTF8, 0, ofn.lpstrFile, -1, &result[0], bufferSize, nullptr, nullptr);
		result.pop_back(); // Remove the null terminator added by WideCharToMultiByte
		return result;
	}

	return "";
}