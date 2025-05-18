#version 330 core

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;

uniform vec3 vertexColor; // color of the mesh
uniform float vertexTransparency; // Transparency of the mesh

layout(location = 0) in vec3 vertex_position;

out vec3 v_Color;
// out vec3 v_Normal;
out float v_Transparency;

void main()
{
    // Set the point color and transparency
    v_Color = vertexColor;
    v_Transparency = vertexTransparency;

    // Final position with projection matrix (fixes clipping issues)
    gl_Position = viewMatrix * modelMatrix * vec4(vertex_position, 1.0);

}

