#version 330 core

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

uniform vec3 vertexColor; // color of the mesh
uniform float vertexTransparency; // Transparency of the mesh

uniform float zoomscale; // zoomscale value is passed to keep the size of the texture constant

layout(location = 0) in vec3 vertex_position;
layout(location = 1) in vec3 vertex_center;
layout(location = 2) in vec2 textureCoord;
layout(location = 3) in float textureType;

flat out uint v_textureType;
out vec2 v_textureCoord;
out vec3 v_textureColor;
out float v_Transparency;


void main()
{

	// Pass the texture type and texture coord
	v_textureType = uint(textureType);
	v_textureCoord = textureCoord;

	// Set the texture color and texture transparency
    v_textureColor = vertexColor;
    v_Transparency = vertexTransparency;


	// Final position with projection matrix
	// apply transformation to vertex position
    vec4 trans_position = projectionMatrix * viewMatrix * modelMatrix * vec4(vertex_position, 1.0);

	// apply transformation to vertex center
	vec4 trans_center = projectionMatrix * viewMatrix * modelMatrix * vec4(vertex_center, 1.0); 

	// Scale the final position
	vec2 scaled_pt = vec2(trans_position.x - trans_center.x, trans_position.y - trans_center.y) / zoomscale;

	// Final position passed to shader
	gl_Position = vec4(trans_center.x + scaled_pt.x, trans_center.y + scaled_pt.y, 0.0f, 1.0f);
		
}