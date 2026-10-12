#version 460 core

in vec2 uv;
in flat vec3 color;
in flat vec2 halfSize; // (halfLength + radius, radius)
in flat uint fragIdOut;

layout(location = 0) out vec4 fragColor;
layout(location = 1) out uint fragId;

void main()
{
    const float radius = halfSize.y;
    const float halfLength = halfSize.x - radius;

    // Squared distance to the core segment; no sqrt needed.
    const vec2 q = vec2(max(abs(uv.x) - halfLength, 0.0), uv.y);
    if (dot(q, q) > radius * radius) discard;

    const int checker = (int(floor(uv.x)) + int(floor(uv.y))) & 1;
    fragColor = vec4(color * (0.8 + 0.2 * float(checker)), 1.0);
    fragId = fragIdOut;
}