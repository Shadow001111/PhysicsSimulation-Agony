#version 460 core

layout (location = 0) in vec2 vertex;
layout (location = 1) in vec4 instancePosAndSize; // (x, y, w, h)
layout (location = 2) in vec2 instanceTexSize;
layout (location = 3) in uint instanceTextureID;

out vec2 texCoords;
flat out uint textureID;

uniform mat4 projection;

void main()
{
    vec2 pos = instancePosAndSize.xy + vertex * instancePosAndSize.zw;
    texCoords.x = instanceTexSize.x * vertex.x;
    texCoords.y = instanceTexSize.y * (1.0 - vertex.y);
    textureID = instanceTextureID;
    gl_Position = projection * vec4(pos, 0.0, 1.0);
}