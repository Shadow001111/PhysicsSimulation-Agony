#version 460 core

layout (location = 0) in vec2 vertLocalPosition;
layout (location = 1) in vec2 instancePosition;
layout (location = 2) in float instanceRadius;
layout (location = 3) in uint instanceColor;
layout (location = 4) in uint instanceId;

out vec2 uv;
out flat vec3 color;
out flat float radius;
out flat uint fragIdOut;

uniform mat4 viewProjectionMatrix;

void main()
{
    uv = vertLocalPosition;

    vec2 worldPos = instancePosition + vertLocalPosition * instanceRadius;
    gl_Position = viewProjectionMatrix * vec4(worldPos, 0.0, 1.0);

    const float r = (instanceColor >> 16) / 255.0;
    const float g = ((instanceColor >> 8) & 255) / 255.0;
    const float b = (instanceColor & 255) / 255.0;
    color = vec3(r, g, b);

    radius = instanceRadius;
    fragIdOut = instanceId;
}