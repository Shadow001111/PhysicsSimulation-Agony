#version 460 core

layout (location = 0) in vec2 uv;
layout (location = 1) in vec4 bounds;

uniform mat4 viewProjectionMatrix;

void main()
{
    float x = mix(bounds.x, bounds.z, uv.x);
    float y = mix(bounds.y, bounds.w, uv.y);
    gl_Position = viewProjectionMatrix * vec4(x, y, 0.0, 1.0);
}