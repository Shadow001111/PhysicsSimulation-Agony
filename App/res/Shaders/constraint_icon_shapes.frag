#version 460 core

// Indices match ShapeRenderer::ConstraintIcon. p is in [-1, 1].

float springShape(vec2 p)
{
    float wave = abs(fract(p.x * 1.25) - 0.5) * 4.0 - 1.0;
    float d = abs(p.y - 0.5 * wave) * 0.371 - 0.1;
    return max(d, abs(p.x) - 0.8);
}

float revoluteShape(vec2 p)
{
    return abs(length(p) - 0.6) - 0.15;
}

float prismaticShape(vec2 p)
{
    vec2 q = abs(p) - 0.6;
    return abs(max(q.x, q.y)) - 0.12;
}

float sliderShape(vec2 p)
{
    vec2 q = abs(p) - vec2(0.7, 0.3);
    return abs(max(q.x, q.y)) - 0.1;
}


float iconDistance(uint index, vec2 p)
{
    switch (index)
    {
        case 0u: return springShape(p);
        case 1u: return revoluteShape(p);
        case 2u: return prismaticShape(p);
        case 3u: return sliderShape(p);
        default: return 1.0;
    }
}
