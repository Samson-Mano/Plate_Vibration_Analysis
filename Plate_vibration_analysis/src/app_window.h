#pragma once
#include <iostream>
#include <fstream>
//____ OpenGL dependencies
#include <GL/glew.h>
#include <GLFW/glfw3.h>
//____ ImGUI dependencies
#include "../resources/ImGui/imgui.h"
#include "../resources/ImGui/imgui_impl_glfw.h"
#include "../resources/ImGui/imgui_impl_opengl3.h"
#include "../resources/ImGui/stb_implement.h"
//____ Geometry store
#include "geometry_store/geom_store.h"
//____ Mouse event handler
#include "events_handler/mouse_event_handler.h"
//---- File event handler
#include "events_handler/file_events.h"
//____ Tool Window
#include "tool_window/constraint_window.h"
#include "tool_window/inlcondition_window.h"
#include "tool_window/material_window.h"
#include "tool_window/node_load_window.h"
#include "tool_window/modal_analysis_window.h"
#include "tool_window/pulse_analysis_window.h"
#include "tool_window/options_window.h"


class app_window
{
public:
	GLFWwindow* window = nullptr;
	ImFont* imgui_font = nullptr;

	// Variable to control the windows mouse events
	mouse_event_handler mouse_Handler;
	// Variable to control the file menu events
	file_events file_menu;

	bool is_glwindow_success = false;
	static int window_width;
	static int window_height;
	static bool isWindowSizeChanging;

	// main geometry variable
	geom_store geom;

	// Tool window variable
	constraint_window nd_cnst_window;
	inlcondition_window inl_window;
	options_window op_window;
	node_load_window nd_load_window;
	material_window mat_window;
	modal_analysis_window sol_modal_window;
	pulse_analysis_window sol_pulse_window;


	app_window();
	~app_window();

	// Functions
	void init();
	void fini();
	void app_render();
	void menu_events();
	static void framebufferSizeCallback(GLFWwindow* window, int window_width, int window_height);
	void GLFWwindow_set_icon(GLFWwindow* window);
private:
};