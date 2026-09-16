#include "SimulationRenderer.h"
#include "SimulationRenderer_Internal.h"

#include "Physics/Simulation.h"

#include "Ecstasy/Core/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <cmath>
#include <algorithm>

namespace Render
{
    // TODO: Add culling.
    void SimulationRenderer::renderSprings(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        TRACY_SCOPE_N("Render springs");

        const auto& springs = simulation.getSprings();
        const size_t springCount = springs.getCount();
        if (springCount == 0) return;

        const size_t vertexCount = springCount * 2;
        lineVertexData.resize(vertexCount);
        ShapeRenderer::LineVertex* ECSTASY_RESTRICT verts = lineVertexData.data();

        for (size_t i = 0; i < springCount; i++)
        {
            const ObjectIndex bodyIndexA = springs.bodyIndexA[i];
            const ObjectIndex bodyIndexB = springs.bodyIndexB[i];

            const BodyTransformSample bodyA = transformContext.sample(bodyIndexA);
            const BodyTransformSample bodyB = transformContext.sample(bodyIndexB);

            // Compute world positions of spring endpoints.
            const float worldAx = bodyA.x + (springs.localAnchorA[i].x * bodyA.cosR - springs.localAnchorA[i].y * bodyA.sinR);
            const float worldAy = bodyA.y + (springs.localAnchorA[i].x * bodyA.sinR + springs.localAnchorA[i].y * bodyA.cosR);

            const float worldBx = bodyB.x + (springs.localAnchorB[i].x * bodyB.cosR - springs.localAnchorB[i].y * bodyB.sinR);
            const float worldBy = bodyB.y + (springs.localAnchorB[i].x * bodyB.sinR + springs.localAnchorB[i].y * bodyB.cosR);

            // Compute displacement.
            const float dx = worldBx - worldAx;
            const float dy = worldBy - worldAy;
            const float currentLength = std::sqrt(dx * dx + dy * dy);
            const float restLength = static_cast<float>(springs.restLength[i]);
            const float displacement = currentLength - restLength;

            // Compute color.
            uint32_t color;
            {
                // Normalize strain into [-1.0, 1.0].
                float t = std::clamp(displacement / restLength, -1.0f, 1.0f);

                // Calculate weights for each endpoint color.
                float wBlue = std::fmax(0.0f, -t);    // Active (< 0).
                float wRed = std::fmax(0.0f, t);     // Active (> 0).
                float wGrey = 1.0f - std::fabs(t);    // Active (= 0).

                // Blend RGB channels.
                uint32_t r = uint32_t(255.0f * wRed + 187.0f * wGrey);
                uint32_t g = uint32_t(187.0f * wGrey);
                uint32_t b = uint32_t(255.0f * wBlue + 187.0f * wGrey);

                color = (r << 16) | (g << 8) | b;
            }

            size_t vIdx = i * 2;
            verts[vIdx].x = worldAx;
            verts[vIdx].y = worldAy;
            verts[vIdx].color = color;

            verts[vIdx + 1].x = worldBx;
            verts[vIdx + 1].y = worldBy;
            verts[vIdx + 1].color = color;
        }

        shapeRenderer.renderLines(lineVertexData, viewProjectionMatrix);
    }

    // TODO: Add culling.
    void SimulationRenderer::renderJoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        TRACY_SCOPE_N("Render joints");

        const auto& joints = simulation.getJoints();
        const size_t jointCount = joints.getCount();
        if (jointCount == 0) return;

        // 2 lines per joint = 4 vertices.
        const size_t vertexCount = jointCount * 4;
        lineVertexData.resize(vertexCount);
        ShapeRenderer::LineVertex* ECSTASY_RESTRICT verts = lineVertexData.data();

        for (size_t i = 0; i < jointCount; i++)
        {
            const ObjectIndex bodyIndexA = joints.bodyIndexA[i];
            const ObjectIndex bodyIndexB = joints.bodyIndexB[i];

            const BodyTransformSample bodyA = transformContext.sample(bodyIndexA);
            const BodyTransformSample bodyB = transformContext.sample(bodyIndexB);

            // Compute world positions of joint anchors.
            const float worldAx = bodyA.x + (joints.localAnchorA[i].x * bodyA.cosR - joints.localAnchorA[i].y * bodyA.sinR);
            const float worldAy = bodyA.y + (joints.localAnchorA[i].x * bodyA.sinR + joints.localAnchorA[i].y * bodyA.cosR);

            const float worldBx = bodyB.x + (joints.localAnchorB[i].x * bodyB.cosR - joints.localAnchorB[i].y * bodyB.sinR);
            const float worldBy = bodyB.y + (joints.localAnchorB[i].x * bodyB.sinR + joints.localAnchorB[i].y * bodyB.cosR);

            constexpr uint32_t color = 0x00FFFF;
            const size_t vIdx = i * 4;

            // Line 1: Body A position to Anchor A
            verts[vIdx + 0].x = static_cast<float>(bodyA.x);
            verts[vIdx + 0].y = static_cast<float>(bodyA.y);
            verts[vIdx + 0].color = color;

            verts[vIdx + 1].x = worldAx;
            verts[vIdx + 1].y = worldAy;
            verts[vIdx + 1].color = color;

            // Line 2: Body B position to Anchor B
            verts[vIdx + 2].x = static_cast<float>(bodyB.x);
            verts[vIdx + 2].y = static_cast<float>(bodyB.y);
            verts[vIdx + 2].color = color;

            verts[vIdx + 3].x = worldBx;
            verts[vIdx + 3].y = worldBy;
            verts[vIdx + 3].color = color;
        }

        shapeRenderer.renderLines(lineVertexData, viewProjectionMatrix);
    }

    // TODO: Add culling.
    void SimulationRenderer::renderPrismaticJoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        TRACY_SCOPE_N("Render prismatic joints");

        const auto& prismaticJoints = simulation.getPrismaticJoints();
        const size_t prismaticJointCount = prismaticJoints.getCount();
        if (prismaticJointCount == 0) return;

        // 2 lines per joint = 4 vertices.
        const size_t vertexCount = prismaticJointCount * 4;
        lineVertexData.resize(vertexCount);
        ShapeRenderer::LineVertex* ECSTASY_RESTRICT verts = lineVertexData.data();

        constexpr uint32_t actualColor = 0xFF3333; // Reddish: anchor to anchor.
        constexpr uint32_t targetColor = 0x3399FF; // Blueish: axis-projected, clamped.

        for (size_t i = 0; i < prismaticJointCount; i++)
        {
            const ObjectIndex bodyIndexA = prismaticJoints.bodyIndexA[i];
            const ObjectIndex bodyIndexB = prismaticJoints.bodyIndexB[i];

            const BodyTransformSample bodyA = transformContext.sample(bodyIndexA);
            const BodyTransformSample bodyB = transformContext.sample(bodyIndexB);

            // Compute world positions of joint anchors.
            const float worldAx = bodyA.x + (prismaticJoints.localAnchorA[i].x * bodyA.cosR - prismaticJoints.localAnchorA[i].y * bodyA.sinR);
            const float worldAy = bodyA.y + (prismaticJoints.localAnchorA[i].x * bodyA.sinR + prismaticJoints.localAnchorA[i].y * bodyA.cosR);

            const float worldBx = bodyB.x + (prismaticJoints.localAnchorB[i].x * bodyB.cosR - prismaticJoints.localAnchorB[i].y * bodyB.sinR);
            const float worldBy = bodyB.y + (prismaticJoints.localAnchorB[i].x * bodyB.sinR + prismaticJoints.localAnchorB[i].y * bodyB.cosR);

            // Compute world-space slider axis (attached to body A, so it rotates with A).
            const float axisX = static_cast<float>(prismaticJoints.localAxisA[i].x * bodyA.cosR - prismaticJoints.localAxisA[i].y * bodyA.sinR);
            const float axisY = static_cast<float>(prismaticJoints.localAxisA[i].x * bodyA.sinR + prismaticJoints.localAxisA[i].y * bodyA.cosR);

            // Project anchor-to-anchor displacement onto the axis, clamp to the joint's limit.
            const float dx = worldBx - worldAx;
            const float dy = worldBy - worldAy;
            float projection = dx * axisX + dy * axisY;

            if (prismaticJoints.enableLimit[i])
            {
                projection = std::clamp(
                    projection,
                    static_cast<float>(prismaticJoints.lowerLimit[i]),
                    static_cast<float>(prismaticJoints.upperLimit[i])
                );
            }

            const float targetX = worldAx + axisX * projection;
            const float targetY = worldAy + axisY * projection;

            const size_t vIdx = i * 4;

            // Line 1: Anchor A to Anchor B (actual).
            verts[vIdx + 0].x = worldAx;
            verts[vIdx + 0].y = worldAy;
            verts[vIdx + 0].color = actualColor;

            verts[vIdx + 1].x = worldBx;
            verts[vIdx + 1].y = worldBy;
            verts[vIdx + 1].color = actualColor;

            // Line 2: Anchor A to axis-projected, clamped point (target).
            verts[vIdx + 2].x = worldAx;
            verts[vIdx + 2].y = worldAy;
            verts[vIdx + 2].color = targetColor;

            verts[vIdx + 3].x = targetX;
            verts[vIdx + 3].y = targetY;
            verts[vIdx + 3].color = targetColor;
        }

        shapeRenderer.renderLines(lineVertexData, viewProjectionMatrix);
    }

    // TODO: Add culling.
    void SimulationRenderer::renderRods(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext)
    {
        TRACY_SCOPE_N("Render rods");

        const auto& rods = simulation.getRods();
        const size_t rodCount = rods.getCount();
        if (rodCount == 0) return;

        const size_t vertexCount = rodCount * 2;
        lineVertexData.resize(vertexCount);
        ShapeRenderer::LineVertex* ECSTASY_RESTRICT verts = lineVertexData.data();

        for (size_t i = 0; i < rodCount; i++)
        {
            const ObjectIndex bodyIndexA = rods.bodyIndexA[i];
            const ObjectIndex bodyIndexB = rods.bodyIndexB[i];

            const BodyTransformSample bodyA = transformContext.sample(bodyIndexA);
            const BodyTransformSample bodyB = transformContext.sample(bodyIndexB);

            // Compute world positions of rod endpoints.
            const float worldAx = bodyA.x + (rods.localAnchorA[i].x * bodyA.cosR - rods.localAnchorA[i].y * bodyA.sinR);
            const float worldAy = bodyA.y + (rods.localAnchorA[i].x * bodyA.sinR + rods.localAnchorA[i].y * bodyA.cosR);

            const float worldBx = bodyB.x + (rods.localAnchorB[i].x * bodyB.cosR - rods.localAnchorB[i].y * bodyB.sinR);
            const float worldBy = bodyB.y + (rods.localAnchorB[i].x * bodyB.sinR + rods.localAnchorB[i].y * bodyB.cosR);

            // Compute displacement.
            const float dx = worldBx - worldAx;
            const float dy = worldBy - worldAy;
            const float currentLength = std::sqrt(dx * dx + dy * dy);
            const float restLength = static_cast<float>(rods.length[i]);
            const float displacement = currentLength - restLength;

            // Compute color.
            uint32_t color;
            {
                // Normalize strain into [-1.0, 1.0].
                float t = std::clamp(displacement / restLength, -1.0f, 1.0f);

                // Calculate weights for each endpoint color.
                float wBlue = std::fmax(0.0f, -t);    // Active (< 0).
                float wRed = std::fmax(0.0f, t);     // Active (> 0).
                float wGrey = 1.0f - std::fabs(t);    // Active (= 0).

                // Blend RGB channels.
                uint32_t r = uint32_t(255.0f * wRed + 187.0f * wGrey);
                uint32_t g = uint32_t(187.0f * wGrey);
                uint32_t b = uint32_t(255.0f * wBlue + 187.0f * wGrey);

                color = (r << 16) | (g << 8) | b;
            }

            size_t vIdx = i * 2;
            verts[vIdx].x = worldAx;
            verts[vIdx].y = worldAy;
            verts[vIdx].color = color;

            verts[vIdx + 1].x = worldBx;
            verts[vIdx + 1].y = worldBy;
            verts[vIdx + 1].color = color;
        }

        shapeRenderer.renderLines(lineVertexData, viewProjectionMatrix);
    }
}