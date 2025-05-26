#version 330 core
uniform sampler2D u_Texture;

in vec4 v_textureColor;
in vec2 v_textureCoord;

out vec4 f_Color; // fragment's final color (out to the fragment shader)

void main()
{

	// Sample the texture's red channel for the alpha value
    float alpha = texture(u_Texture, v_textureCoord).r;

	// Apply the sampled alpha to the vertex color
    f_Color = vec4(v_textureColor.rgb, v_textureColor.a * alpha);

    // Discard fully transparent fragments to avoid white background issues
    if (f_Color.a < 0.01)
        discard;


}