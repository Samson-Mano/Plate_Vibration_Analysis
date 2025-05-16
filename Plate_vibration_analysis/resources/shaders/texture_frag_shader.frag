#version 330 core
uniform sampler2D u_Textures[3];

flat in uint v_textureType;
in vec2 v_textureCoord;
in vec3 v_textureColor;
in float v_Transparency;

out vec4 f_Color; // fragment's final color (out to the fragment shader)

void main()
{
	// Pin Support [0], Roller Support [1], 3D circle [2]
	vec4 texColor = texture(u_Textures[v_textureType],v_textureCoord);
	
	// f_Color = vec4(v_textureColor, v_Transparency) * texColor;

	f_Color = vec4(v_textureColor, texColor.a * v_Transparency);

	// Discard fully transparent fragments
    if (f_Color.a < 0.01)
        discard;

}