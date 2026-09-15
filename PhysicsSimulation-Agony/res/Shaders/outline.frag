#version 460 core

out vec4 fragColor;

uniform usampler2D idTexture;
uniform vec3 outlineColor;

void main()
{
    ivec2 texel = ivec2(gl_FragCoord.xy);
    uint center = texelFetch(idTexture, texel, 0).r;

    uint n = texelFetch(idTexture, texel + ivec2(0, 1), 0).r;
    uint s = texelFetch(idTexture, texel + ivec2(0, -1), 0).r;
    uint e = texelFetch(idTexture, texel + ivec2(1, 0), 0).r;
    uint w = texelFetch(idTexture, texel + ivec2(-1, 0), 0).r;

    if (n == center && s == center && e == center && w == center)
        discard;

    //bool isBoundary = (n != 0u && n != center) ||
    //                   (s != 0u && s != center) ||
    //                   (e != 0u && e != center) ||
    //                   (w != 0u && w != center);

    fragColor = vec4(outlineColor, 1.0);
}