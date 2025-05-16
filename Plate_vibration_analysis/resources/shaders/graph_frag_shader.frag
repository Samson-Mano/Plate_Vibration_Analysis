
#version 330 core

in float v_Transparency;
in float v_Intensity; // Input intensity value (range [0.0, 1.0])
out vec4 f_Color; // Final color output



vec3 jet_colormap(float intensity) 
{
    // Clamp intensity to the range [0.0, 1.0]
    float t = clamp(intensity, 0.0, 1.0);

    // Calculate the red, green, and blue components based on intensity
    float r = smoothstep(0.5, 0.75, t);
    float g = smoothstep(0.25, 0.5, t) - smoothstep(0.75, 1.0, t);
    float b = 1.0 - smoothstep(0.0, 0.25, t);

    return vec3(r, g, b);
}

void main() 
{
    // Get the RGB color based on the intensity value
    vec3 v_color = jet_colormap(v_Intensity);

    // Set the final output color (RGBA with alpha = 1.0)
    f_Color = vec4(v_color, v_Transparency);
   
}





