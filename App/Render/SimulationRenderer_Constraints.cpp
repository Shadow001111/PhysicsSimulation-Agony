#include "SimulationRenderer.h"
#include "SimulationRenderer_Internal.h"

#include "PS_AGONY2D/Simulation.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"

#include <cmath>

namespace PS_AGONY_RENDER
{
    namespace
    {
        struct CullRect
        {
            float minX, minY, maxX, maxY;
        };

        using LineVertices = std::vector<ShapeRenderer::LineVertex>;
        using IconInstances = std::vector<ShapeRenderer::ConstraintIconInstanceData>;

        template<ConstraintType Type>
        constexpr uint32_t iconOf() noexcept
        {
            return uint32_t(Type);
        }

        template<ConstraintType Type, class Viewer>
        void collect(
            const Viewer& viewer,
            IndexRange range,
            uint32_t color,
            const BodyTransformContext& transformContext,
            const CullRect& cull,
            LineVertices& lines,
            IconInstances* icons)
        {
            constexpr uint32_t icon = static_cast<uint32_t>(iconOf<Type>());

            for (ObjectDenseIndex i = range.begin; i < range.end; i++)
            {
                const ObjectPair pair = viewer.bodyIndexPair[i];

                const BodyTransformSample bodyA = transformContext.sample(pair.a);
                const BodyTransformSample bodyB = transformContext.sample(pair.b);

                const auto localA = viewer.localAnchorA[i];
                const auto localB = viewer.localAnchorB[i];

                const float worldAx = bodyA.x + (localA.x * bodyA.cosR - localA.y * bodyA.sinR);
                const float worldAy = bodyA.y + (localA.x * bodyA.sinR + localA.y * bodyA.cosR);
                const float worldBx = bodyB.x + (localB.x * bodyB.cosR - localB.y * bodyB.sinR);
                const float worldBy = bodyB.y + (localB.x * bodyB.sinR + localB.y * bodyB.cosR);

                float minX = std::fmin(worldAx, worldBx);
                float maxX = std::fmax(worldAx, worldBx);
                float minY = std::fmin(worldAy, worldBy);
                float maxY = std::fmax(worldAy, worldBy);

                if constexpr (Type == ConstraintType::RevoluteJoint)
                {
                    minX = std::fmin(minX, std::fmin(bodyA.x, bodyB.x));
                    maxX = std::fmax(maxX, std::fmax(bodyA.x, bodyB.x));
                    minY = std::fmin(minY, std::fmin(bodyA.y, bodyB.y));
                    maxY = std::fmax(maxY, std::fmax(bodyA.y, bodyB.y));
                }

                if (maxX < cull.minX || minX > cull.maxX || maxY < cull.minY || minY > cull.maxY)
                {
                    continue;
                }

                if constexpr (Type == ConstraintType::RevoluteJoint)
                {
                    lines.push_back({ bodyA.x, bodyA.y, color });
                    lines.push_back({ worldAx, worldAy, color });
                    lines.push_back({ bodyB.x, bodyB.y, color });
                    lines.push_back({ worldBx, worldBy, color });
                }
                else
                {
                    lines.push_back({ worldAx, worldAy, color });
                    lines.push_back({ worldBx, worldBy, color });
                }

                if (icons)
                {
                    icons->push_back({ (worldAx + worldBx) * 0.5f, (worldAy + worldBy) * 0.5f, color, icon });
                }
            }
        }
    }

    void SimulationRenderer::renderConstraints(
        const Simulation& simulation,
        const Mat4& viewProjectionMatrix,
        const BodyTransformContext& transformContext,
        const AABB& cameraAABB,
        const ConstraintGroupColors& groupColors,
        float iconSize)
    {
        TRACY_SCOPE_N("Render constraints");

        const auto& springs = simulation.constraints().getSprings();
        const auto& revoluteJoints = simulation.constraints().getRevoluteJoints();
        const auto& prismaticJoints = simulation.constraints().getPrismaticJoints();
        const auto& sliderJoints = simulation.constraints().getSliderJoints();

        const size_t totalCount = springs.getCount() + revoluteJoints.getCount()
            + prismaticJoints.getCount() + sliderJoints.getCount();
        if (totalCount == 0) return;

        const IConstraintSystem& springSystem = simulation.getConstraintSystem(ConstraintType::Spring);
        const IConstraintSystem& revoluteSystem = simulation.getConstraintSystem(ConstraintType::RevoluteJoint);
        const IConstraintSystem& prismaticSystem = simulation.getConstraintSystem(ConstraintType::PrismaticJoint);
        const IConstraintSystem& sliderSystem = simulation.getConstraintSystem(ConstraintType::SliderJoint);

        const bool drawIcons = iconSize > 0.0f;

        const CullRect cull{
            static_cast<float>(cameraAABB.minX), static_cast<float>(cameraAABB.minY),
            static_cast<float>(cameraAABB.maxX), static_cast<float>(cameraAABB.maxY)
        };

        lineVertexData.clear();
        constraintIconData.clear();
        IconInstances* icons = nullptr;
        if (drawIcons)
        {
            constraintIconData.reserve(totalCount);
            icons = &constraintIconData;
        }

        for (size_t g = 0; g < groupColors.size(); g++)
        {
            const ConstraintGroup group = static_cast<ConstraintGroup>(g);
            const uint32_t color = groupColors[g];

            collect<ConstraintType::Spring>(springs, springSystem.range(group), color, transformContext, cull, lineVertexData, icons);
            collect<ConstraintType::RevoluteJoint>(revoluteJoints, revoluteSystem.range(group), color, transformContext, cull, lineVertexData, icons);
            collect<ConstraintType::PrismaticJoint>(prismaticJoints, prismaticSystem.range(group), color, transformContext, cull, lineVertexData, icons);
            collect<ConstraintType::SliderJoint>(sliderJoints, sliderSystem.range(group), color, transformContext, cull, lineVertexData, icons);
        }

        shapeRenderer.renderLines(lineVertexData, viewProjectionMatrix);

        if (drawIcons)
        {
            shapeRenderer.renderConstraintIcons(constraintIconData, viewProjectionMatrix, iconSize * 0.5f);
        }
    }
}