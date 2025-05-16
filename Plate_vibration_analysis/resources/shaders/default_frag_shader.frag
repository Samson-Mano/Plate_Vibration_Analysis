
#version 330 core

in vec3 v_Color;
in float v_Transparency;

out vec4 f_Color; // Final color output

void main() 
{

    f_Color = vec4(v_Color, v_Transparency );
}





