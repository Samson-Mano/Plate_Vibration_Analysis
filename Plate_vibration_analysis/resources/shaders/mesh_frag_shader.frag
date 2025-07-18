
#version 330 core

in vec3 vertNormal;
in vec3 v_Color;
in float v_is_dynamic;
in float v_deflscale; // Deflection scale value = normalized_deflscale (varies 0 to 1) * max deformation
in float v_Transparency; 

out vec4 f_Color; // Final color output

vec3 unreal(vec3 x) 
{
    return x / (x + 0.155) * 1.019;
}

vec3 jetHeatmap(float value) 
{

    return clamp(vec3(1.5) - abs(4.0 * vec3(value) + vec3(-3, -2, -1)), vec3(0), vec3(1));
}


void main() 
{

    // View direction fixed at -z
    vec3 viewPos = vec3(0.0, 0.0, -1.0);

    // Light direction fixed at -z
    vec3 uniLightDir = vec3(0.0, 0.0, -1.0);

    // Halfway vector between view direction and light direction
    vec3 halfDir = normalize(viewPos + uniLightDir);

    vec3 c_v_Color = v_Color;

    if(v_is_dynamic == 1.0f)
    {
        // Set the contour color
        c_v_Color = jetHeatmap(v_deflscale);
    }

    // Color with slight mix to white for specular highlights
    vec3 specColor = mix(c_v_Color, vec3(1.0, 1.0, 1.0), 0.1) * 1.5;
    vec3 diffColor = c_v_Color;
    float shineness = 40.0;

    // Calculate specular and diffuse lighting
    float specular = pow(max(0.0, dot(halfDir, vertNormal)), shineness);
    float diffuse = max(0.0, dot(uniLightDir, vertNormal));
    float ambient = 0.05;
    
    // Final color with lighting
    vec3 finalColor = (diffuse + ambient) * diffColor + specular * specColor;

    // Tone mapping
    finalColor = unreal(finalColor);
    f_Color = vec4(finalColor, v_Transparency); // v_Transparency
}





