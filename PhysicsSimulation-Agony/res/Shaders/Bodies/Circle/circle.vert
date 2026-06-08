#version 460 core

layout (location = 0) in vec2 vertLocalPosition;
layout (location = 1) in vec2 instancePosition;
layout (location = 2) in vec2 instanceCOM;
layout (location = 3) in float instanceRotation;
layout (location = 4) in float instanceRadius;
layout (location = 5) in int instanceColor; // 3 bytes used.
layout (location = 6) in uint instanceTextureId;

out vec2 uv;
out flat vec3 color;

uniform mat4 viewProjectionMatrix;

vec2 rotate2D(vec2 v, float angle)
{
    float s = sin(angle);
    float c = cos(angle);
    return vec2(
        v.x * c - v.y * s,
        v.x * s + v.y * c
        );
}

void main()
{
    uv = vertLocalPosition;

    vec2 localPosition = vertLocalPosition * instanceRadius;
    vec2 worldPos = instancePosition + rotate2D(localPosition - instanceCOM, instanceRotation) + instanceCOM;
    vec4 clipPos = viewProjectionMatrix * vec4(worldPos, 0.0, 1.0);
    gl_Position = clipPos;

    const float r = (instanceColor >> 16) / 255.0;
    const float g = ((instanceColor >> 8) & 255) / 255.0;
    const float b = (instanceColor & 255) / 255.0;
    color = vec3(r, g, b);
}