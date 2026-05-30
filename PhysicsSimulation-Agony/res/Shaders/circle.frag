#version 460 core

in vec2 uv;
in flat vec3 color;

out vec4 fragColor;

void main()
{
    float dist = dot(uv, uv);
    if (dist > 1.0)
        discard;
    
    fragColor = vec4(color, 1.0);
}
