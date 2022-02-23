#version 330 core

in vec3 normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// uniforms used for lighting
// You can change these colors, or even pass them in from the cpu side.
uniform vec3 AmbientColor = vec3(0.2);
uniform vec3 LightDirection = normalize(vec3(-3, -4, -5));
uniform vec3 LightColor = vec3(1);
uniform vec3 DiffuseColor = vec3(0.7, 0.7, 0.0);

out vec4 fragColor;

void main() {
    // Compute irradiance (sum of ambient & direct lighting)
    vec3 irradiance = AmbientColor + LightColor * max(0, dot(LightDirection, normal));

    // Diffuse reflectance
    vec3 reflectance = irradiance * DiffuseColor;

    // Gamma correction
    fragColor = vec4(sqrt(reflectance), 1);
}
