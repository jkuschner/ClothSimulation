#version 330 core

layout (location = 0) in vec4 position;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 viewInv;
uniform mat4 projInv;

out vec3 near;
out vec3 far;

vec3 unprojectPoint(float x, float y, float z) {
//    mat4 viewInv = inverse(view);
//    mat4 projInv = inverse(projection);
    vec4 p = viewInv * projInv * vec4(x, y, z, 1.f);
    return p.xyz / p.w;
}

void main() {
    near = unprojectPoint(position.x, position.y, 0.f);
    far = unprojectPoint(position.x, position.y, 1.f);
    gl_Position = position;
}
