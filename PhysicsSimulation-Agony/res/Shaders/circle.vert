#version 460 core

layout (location = 0) in vec2 localPosition;
layout (location = 1) in vec2 instancePosition;
layout (location = 2) in float instanceRadius;
layout (location = 3) in int instanceCollisionDebug;

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

void main()
{
    uv = localPosition;

    vec3 worldPos = vec3(instancePosition + localPosition * instanceRadius, 0.0);
    vec4 clipPos = viewProjectionMatrix * vec4(worldPos, 1.0);
    gl_Position = clipPos;

    int broadPhaseCollision = bitfieldExtract(instanceCollisionDebug, 0, 1);
    color = randomColorFromInt(gl_InstanceID);
}