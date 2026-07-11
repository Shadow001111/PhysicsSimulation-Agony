#version 460 core

layout (location = 0) in vec2 inPosition;
layout (location = 1) in uint inColor; // Packed 0xRRGGBB hex color

uniform mat4 viewProjectionMatrix;

out vec4 passColor;

void main()
{
    gl_Position = viewProjectionMatrix * vec4(inPosition, 0.0, 1.0);

    // Unpack R, G, B channels from the hex integer and normalize to [0.0, 1.0]
    float r = float((inColor >> 16) & 0xFFu) / 255.0;
    float g = float((inColor >> 8)  & 0xFFu) / 255.0;
    float b = float(inColor         & 0xFFu) / 255.0;

    passColor = vec4(r, g, b, 1.0);
}