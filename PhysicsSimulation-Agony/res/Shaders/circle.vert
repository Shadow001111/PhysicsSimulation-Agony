#version 460 core

layout (location = 0) in vec2 aPos;

out vec2 uv;

uniform vec2 position;
uniform float radius;

uniform mat4 viewProjectionMatrix;

void main()
{
    uv = aPos;

    vec3 worldPos = vec3(position + aPos * radius, 0.0);

    vec4 clipPos = viewProjectionMatrix * vec4(worldPos, 1.0);

    gl_Position = clipPos;
}