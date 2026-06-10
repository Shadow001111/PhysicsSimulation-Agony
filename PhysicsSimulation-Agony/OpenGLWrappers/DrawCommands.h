#pragma once

struct DrawArraysIndirectCommand
{
	unsigned int count;        // Number of vertices per instance
	unsigned int instanceCount;// Number of instances to draw
	unsigned int first;        // Starting vertex index in the vertex array
	unsigned int baseInstance; // Base instance ID

	DrawArraysIndirectCommand() = default;
	DrawArraysIndirectCommand(unsigned int count, unsigned int instanceCount, unsigned int first, unsigned int baseInstance);
};

struct DrawElementsIndirectCommand
{
    unsigned int count;        // Number of indices per instance
    unsigned int instanceCount;// Number of instances to draw
    unsigned int firstIndex;   // Starting index in the index buffer (in bytes)
    int baseVertex;           // Constant offset added to vertex indices
    unsigned int baseInstance; // Base instance ID

    DrawElementsIndirectCommand() = default;
    DrawElementsIndirectCommand(unsigned int count, unsigned int instanceCount, unsigned int firstIndex, int baseVertex, unsigned int baseInstance);
};