#version 460 core

in vec2 texCoords;
flat in uint textureID;

layout(location = 0) out vec4 fragColor;

uniform sampler2DArray glyphTextureArray;
uniform vec3 textColor;

void main()
{
    float alpha = texture(glyphTextureArray, vec3(texCoords, textureID)).r;
    fragColor = vec4(textColor, alpha);
}