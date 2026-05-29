#version 460 core

layout (location = 0) in vec2 aPos;

uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

void main()
{
    vec3 worldPos = vec3(aPos, 0.0);

    vec4 viewPos = viewMatrix * vec4(worldPos, 1.0);
    vec4 clipPos = projectionMatrix * viewPos;

    gl_Position = clipPos;
}