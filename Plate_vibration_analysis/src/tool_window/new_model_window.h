#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#define NOMINMAX
#include <Windows.h>

#include <commdlg.h>
#include <string>
#include "../../resources/ImGui/imgui.h"
#include "../../resources/ImGui/imgui_impl_glfw.h"
#include "../../resources/ImGui/imgui_impl_opengl3.h"


class new_model_window
{
public:
	bool is_show_window = false;
	bool execute_create_model = false;
	std::vector<std::string> data_lines;

	new_model_window();
	~new_model_window();
	void init();
	void render_window();
private:

	char file_path_buffer[1024] = "circle_mesh.txt"; // default path

	std::string ShowOpenFileDialog();
};
