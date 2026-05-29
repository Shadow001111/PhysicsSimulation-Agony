#version 460 core

layout (location = 0) in vec2 aPos;

uniform mat4 viewProjectionMatrix;

void main()
{
    vec3 worldPos = vec3(aPos, 0.0);

    vec4 clipPos = viewProjectionMatrix * vec4(worldPos, 1.0);

    gl_Position = clipPos;
}