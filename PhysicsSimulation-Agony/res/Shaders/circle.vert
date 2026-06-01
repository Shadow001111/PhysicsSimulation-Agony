#version 460 core

layout (location = 0) in vec2 localPosition;
layout (location = 1) in vec2 instancePosition;
layout (location = 2) in float instanceRotation;
layout (location = 3) in float instanceRadius;

out vec2 uv;
out flat vec3 color;

uniform mat4 viewProjectionMatrix;

uint hash(uint x)
{
    x += (x << 10u);
    x ^= (x >> 6u);
    x += (x << 3u);
    x ^= (x >> 11u);
    x += (x << 15u);
    return x;
}

float random(uint seed)
{
    return float(hash(seed)) / float(0xffffffffu);
}

vec3 randomColorFromInt(int id)
{
    uint u = uint(id);

    return vec3(random(u), random(u + 123456u), random(u + 789012u));
}

vec2 rotate2D(vec2 v, float angle)
{
    float s = sin(angle);
    float c = cos(angle);
    return vec2(v.x * c - v.y * s, v.x * s + v.y * c);
}

void main()
{
    uv = rotate2D(localPosition, instanceRotation);

    vec3 worldPos = vec3(instancePosition + uv * instanceRadius, 0.0);
    vec4 clipPos = viewProjectionMatrix * vec4(worldPos, 1.0);
    gl_Position = clipPos;

    color = randomColorFromInt(gl_InstanceID);
}