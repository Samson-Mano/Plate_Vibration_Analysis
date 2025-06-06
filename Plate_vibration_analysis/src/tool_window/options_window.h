#pragma once
#include <iostream>
#include "../../resources/ImGui/imgui.h"
#include "../../resources/ImGui/imgui_impl_glfw.h"
#include "../../resources/ImGui/imgui_impl_opengl3.h"

class options_window
{
public:
	// Model constraints
	bool is_show_loads = true; // Show loads
	bool is_show_inlcondition = true; // show initial condition
	bool is_show_constraint = true; // show constraints

	// Model Nodes
	bool is_show_modelnodes = false; // Show model nodes

	// Model elements
	bool is_show_modeledeges = true; // Show model edges
	bool is_show_modelelements = true; // show model elements
	bool is_show_meshnormals = false; // show mesh normals

	// Window
	bool is_show_window = false;

	options_window();
	~options_window();
	void init();
	void render_window();
private:

};