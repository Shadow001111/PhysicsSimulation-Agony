#version 460 core

layout (location = 0) in vec2 vertLocalPosition; // Quad corner in [-1, 1].
layout (location = 1) in vec2 instancePosition;
layout (location = 2) in uint instanceColor;     // Packed 0xRRGGBB.
layout (location = 3) in uint instanceIcon;

out vec2 uv;
out flat vec3 color;
out flat uint icon;

uniform mat4 viewProjectionMatrix;
uniform float iconHalfSize; // World units.

void main()
{
    uv = vertLocalPosition;

    vec2 worldPos = instancePosition + vertLocalPosition * iconHalfSize;
    gl_Position = viewProjectionMatrix * vec4(worldPos, 0.0, 1.0);

    color = vec3(
        float((instanceColor >> 16) & 0xFFu),
        float((instanceColor >> 8) & 0xFFu),
        float(instanceColor & 0xFFu)) * (1.0 / 255.0);

    icon = instanceIcon;
}
