#version 460 core

in vec2 uv;
in flat vec3 color;
in flat float radius;

out vec4 fragColor;

const float TILING = 1.0;

void main()
{
    float dist = dot(uv, uv);
    if (dist > 1.0) discard;
    
    vec2 scaled = uv * TILING;
    int ix = int(floor(scaled.x));
    int iy = int(floor(scaled.y));
    int sum = ix + iy;
    int checker = int(mod(float(sum), 2.0));
    const float checkerboardShading = (0.8 + 0.2 * float(checker));

    vec3 finalColor = color * checkerboardShading;

    fragColor = vec4(finalColor, 1.0);
}
