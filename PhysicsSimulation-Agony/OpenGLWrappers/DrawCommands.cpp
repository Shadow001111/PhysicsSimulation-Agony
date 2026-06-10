#include "DrawCommands.h"

DrawArraysIndirectCommand::DrawArraysIndirectCommand(unsigned int count, unsigned int instanceCount, unsigned int first, unsigned int baseInstance) :
	count(count), instanceCount(instanceCount), first(first), baseInstance(baseInstance)
{
}

DrawElementsIndirectCommand::DrawElementsIndirectCommand(unsigned int count, unsigned int instanceCount, unsigned int firstIndex, int baseVertex, unsigned int baseInstance) :
	count(count), instanceCount(instanceCount), firstIndex(firstIndex), baseVertex(baseVertex), baseInstance(baseInstance)
{
}
