#version 460 core

layout (location = 0) in vec2 aPos;

out vec2 uv;

uniform vec2 position;
uniform float radius;

uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

void main()
{
    uv = aPos;

    vec3 worldPos = vec3(position + aPos * radius, 0.0);

    vec4 viewPos = viewMatrix * vec4(worldPos, 1.0);
    vec4 clipPos = projectionMatrix * viewPos;

    gl_Position = clipPos;
}