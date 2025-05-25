#version 330 core

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

uniform float vertexTransparency; // Transparency of the text

layout(location = 0) in vec3 label_location;
layout(location = 1) in vec2 char_position;
layout(location = 2) in vec2 textureCoord;
layout(location = 3) in vec3 vertexColor;

out vec4 v_textureColor;
out vec2 v_textureCoord;

void main()
{
	
	// Transfor the label location
	vec4 transformed_label_loc = projectionMatrix * viewMatrix * modelMatrix * vec4(label_location, 1.0f);

	// Set the final position of the vertex
	gl_Position = vec4(transformed_label_loc.x + char_position.x, transformed_label_loc.y + char_position.y, transformed_label_loc.z, 1.0f);

	// Calculate texture coordinates for the glyph
	v_textureCoord = textureCoord;
	
	// Pass the texture color to the fragment shader
	v_textureColor = vec4(vertexColor, vertexTransparency);
}