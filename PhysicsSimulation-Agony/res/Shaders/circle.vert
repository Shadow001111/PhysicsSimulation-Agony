#version 460 core

layout (location = 0) in vec2 localPosition;
layout (location = 1) in vec2 instancePosition;
layout (location = 2) in float instanceRadius;
layout (location = 3) in int instanceCollisionDebug;

out vec2 uv;
out flat vec3 color;

uniform mat4 viewProjectionMatrix;

void main()
{
    uv = localPosition;

    vec3 worldPos = vec3(instancePosition + localPosition * instanceRadius, 0.0);
    vec4 clipPos = viewProjectionMatrix * vec4(worldPos, 1.0);
    gl_Position = clipPos;

    int broadPhaseCollision = bitfieldExtract(instanceCollisionDebug, 0, 1);
    color = vec3(1 - broadPhaseCollision) * 0.5;
}