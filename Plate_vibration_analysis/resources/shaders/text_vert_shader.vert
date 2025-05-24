#version 330 core

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

uniform vec3 vertexColor; // color of the text
uniform float vertexTransparency; // Transparency of the text

layout(location = 0) in vec3 char_position;
layout(location = 1) in vec2 textureCoord;


out vec4 v_textureColor;
out vec2 v_textureCoord;

void main()
{
	
	// Set the final position of the vertex
	gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(char_position, 1.0f);

	// Calculate texture coordinates for the glyph
	v_textureCoord = textureCoord;
	
	// Pass the texture color to the fragment shader
	v_textureColor = vec4(vertexColor, vertexTransparency);
}