#version 330 core

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;

layout(location = 0) in vec2 vertex_position;
layout(location = 1) in float vertex_intensity;
layout(location = 2) in float vertex_transparency;

out float v_Transparency;
out float v_Intensity;

void main()
{

    // Final position
    gl_Position = viewMatrix * modelMatrix * vec4(vertex_position, 0.0, 1.0);

    // Pass the intensity to fragment shader
    v_Intensity = vertex_intensity; 

    // Pass the transparency to fragment shader
    v_Transparency = vertex_transparency;

}

