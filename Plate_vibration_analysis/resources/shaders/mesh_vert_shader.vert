#version 330 core

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

uniform vec3 vertexColor; // color of the mesh
uniform float vertexTransparency; // Transparency of the mesh

layout(location = 0) in vec3 node_position;
layout(location = 1) in vec3 node_normal;
layout(location = 2) in float is_dynamic;
layout(location = 3) in float deflscale; // Deflection scale value = normalized_deflscale (varies 0 to 1) * max deformation

out vec3 vertNormal;
out vec3 v_Color;
out float v_is_dynamic;
out float v_deflscale;

void main()
{
    // Vertex Normal transformed using only model-view matrix (not projection)
    vertNormal = normalize(transpose(inverse(mat3(viewMatrix * modelMatrix))) * node_normal);

    v_is_dynamic = is_dynamic;
    v_deflscale = deflscale;

    // Set the point color and transparency
    v_Color = vertexColor;

    // Final position with projection matrix (fixes clipping issues)
    gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(node_position, 1.0);
}

