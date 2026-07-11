#version 460 core

in vec2 uv;
in flat vec3 color;
in flat vec2 halfSize;

out vec4 fragColor;

const float TILING = 1.0;

void main()
{
    vec2 scaled = uv * TILING;

    int ix = int(floor(scaled.x));
    int iy = int(floor(scaled.y));
    int sum = ix + iy;
    int checker = int(mod(float(sum), 2.0));
    const float checkerboardShading = (0.8 + 0.2 * float(checker));

    float distX = halfSize.x - abs(uv.x);
    float distY = halfSize.y - abs(uv.y);
    float minEdgeDist = min(distX, distY);
    const float outlineShading = mix(1.0, 0.5, float(minEdgeDist <= 0.05));

    vec3 finalColor = color * (checkerboardShading * outlineShading);

    fragColor = vec4(finalColor, 1.0);
}