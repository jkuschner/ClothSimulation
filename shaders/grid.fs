#version 330 core

uniform mat4 view;
uniform mat4 projection;
uniform float zNear;
uniform float zFar;

in vec3 near;
in vec3 far;
out vec4 fragColor;

void main() {
    float t = -near.y / (far.y - near.y);
    vec3 fragCoord = near + t * (far - near);
    vec2 co = fragCoord.xz;
    vec2 grad = fwidth(co);
    vec2 grid = abs(fract(co - 0.5) - 0.5) / grad;
    float line = min(grid.x, grid.y);
    float minz = min(grad.y, 1);
    float minx = min(grad.x, 1);
    fragColor = vec4(0.3, 0.3, 0.3, 1.0 - min(line, 1.0));
    if (fragCoord.x > -minx && fragCoord.x < minx) {
        fragColor.z = 1.0f;
    }
    if (fragCoord.z > -minz && fragCoord.z < minz) {
        fragColor.x = 1.0f;
    }
    fragColor *= float(t > 0);
    vec4 proj = projection * view * vec4(fragCoord, 1);
    float ndc = (proj.z / proj.w) * 2.f - 1.f;
    float linear = (2 * zNear * zFar) / (zFar + zNear - ndc * (zFar - zNear));
    float fading = max(0, (0.5 - linear / zFar));
    fragColor.a *= fading;
    gl_FragDepth = (proj.z + zNear) / proj.w;
}
