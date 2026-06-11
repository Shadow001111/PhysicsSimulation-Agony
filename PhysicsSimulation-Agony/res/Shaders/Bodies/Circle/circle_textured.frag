#version 460 core

in vec2 uv;
in flat vec3 color;
in flat uint textureId;

uniform sampler2D uHardCodedTexture;

out vec4 fragColor;

const float TILING = 1.0;
const float SHADING_INFLUENCE = 0.5;

void main()
{
    float dist = dot(uv, uv);
    if (dist > 1.0) discard;

    vec3 finalColor;
    if (textureId != 0)
    {
        // Convert vUv from [-1,1] to [0,1] for texture sampling
        vec2 texCoord = uv * 0.5 + 0.5;
        finalColor = texture(uHardCodedTexture, texCoord).rgb;
    }
    else
    {
        // Existing checker pattern
        vec2 scaled = uv * 1.0;  // TILING = 1.0
        int ix = int(floor(scaled.x));
        int iy = int(floor(scaled.y));
        int sum = ix + iy;
        int checker = int(mod(float(sum), 2.0));
        finalColor = color * (0.8 + 0.2 * float(checker));
    }

    float shading = 1.0 - dist * dist;
    shading = (1.0 - SHADING_INFLUENCE) + shading * SHADING_INFLUENCE;

    finalColor *= shading;

    fragColor = vec4(finalColor, 1.0);
}
