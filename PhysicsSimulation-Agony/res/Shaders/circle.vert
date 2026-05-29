#version 460 core

layout (location = 0) in vec2 aPos;
layout (location = 1) in vec3 aInstanceData; // pos + radius

out vec2 uv;

uniform mat4 viewProjectionMatrix;

void main()
{
    uv = aPos;

    vec3 worldPos = vec3(aInstanceData.xy + aPos * aInstanceData.z, 0.0);

    vec4 clipPos = viewProjectionMatrix * vec4(worldPos, 1.0);

    gl_Position = clipPos;
}