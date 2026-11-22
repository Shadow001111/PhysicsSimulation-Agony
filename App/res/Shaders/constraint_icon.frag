#version 460 core

in vec2 uv;
in flat vec3 color;
in flat uint icon;

out vec4 fragColor;

// Defined in constraint_icon_shapes.frag. < 0 inside the icon.
float iconDistance(uint index, vec2 p);

void main()
{
    if (iconDistance(icon, uv) > 0.0) discard;

    fragColor = vec4(color, 1.0);
}
