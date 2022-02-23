#version 330 core

layout (location = 0) in vec4 position_in;
layout (location = 1) in vec4 normal_in;

uniform mat4 model;
uniform mat4 invT;
uniform mat4 view;
uniform mat4 projection;

out vec3 normal;

void main()
{
    gl_Position = projection * view * model * position_in;

    // for shading
    normal = normalize(vec3(invT * normal_in));
}
