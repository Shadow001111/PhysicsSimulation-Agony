#version 460 core

// Per-vertex: local polygon vertex position.
layout(location = 0) in vec2 localPos;

// Per-polygon transform, indexed by gl_DrawID.
struct PolygonInstance
{
    float positionX, positionY;
    float localCOMX, localCOMY;
    float rotation;
    uint  color;
    uint  textureId;
};
layout(std430, binding = 0) readonly buffer InstanceBuffer
{
    PolygonInstance instances[];
};

uniform mat4 viewProjectionMatrix;

out vec3 vColor;

void main()
{
    PolygonInstance inst = instances[gl_DrawID];

    vec2 localCOM = vec2(inst.localCOMX, inst.localCOMY);
    vec2 centered = localPos - localCOM;

    float c = cos(inst.rotation);
    float s = sin(inst.rotation);
    vec2 rotated = vec2(c * centered.x - s * centered.y,
                        s * centered.x + c * centered.y);

    vec2 worldPos = rotated + localCOM + vec2(inst.positionX, inst.positionY);

    // Unpack RGB from packed uint.
    vColor = vec3(
        float((inst.color >> 16u) & 0xFFu) / 255.0,
        float((inst.color >>  8u) & 0xFFu) / 255.0,
        float( inst.color         & 0xFFu) / 255.0
    );

    gl_Position = viewProjectionMatrix * vec4(worldPos, 0.0, 1.0);
}