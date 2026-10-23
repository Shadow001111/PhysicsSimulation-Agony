#include "SimulationRenderer.h"
#include "SimulationRenderer_Internal.h"

#include "Physics/Simulation.h"

#include "Physics/Profiling/TracyProfiler.h"
#include "Ecstasy/Core/Portablity.h"

#include <cmath>
#include <algorithm>

namespace PS_AGONY_RENDER
{
    void SimulationRenderer::renderSprings(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB)
    {
        TRACY_SCOPE_N("Render springs");

        const auto& springs = simulation.getSprings();
        const size_t springCount = springs.getCount();
        if (springCount == 0) return;

        const float cullMinX = static_cast<float>(cameraAABB.minX);
        const float cullMinY = static_cast<float>(cameraAABB.minY);
        const float cullMaxX = static_cast<float>(cameraAABB.maxX);
        const float cullMaxY = static_cast<float>(cameraAABB.maxY);

        lineVertexData.clear();
        lineVertexData.reserve(springCount * 2);

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

            // Cull the spring if its segment bounding box misses the camera.
            const float segMinX = std::fmin(worldAx, worldBx);
            const float segMaxX = std::fmax(worldAx, worldBx);
            const float segMinY = std::fmin(worldAy, worldBy);
            const float segMaxY = std::fmax(worldAy, worldBy);

            if (segMaxX < cullMinX || segMinX > cullMaxX || segMaxY < cullMinY || segMinY > cullMaxY)
            {
                continue;
            }

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

            ShapeRenderer::LineVertex& vA = lineVertexData.emplace_back();
            vA.x = worldAx;
            vA.y = worldAy;
            vA.color = color;

            ShapeRenderer::LineVertex& vB = lineVertexData.emplace_back();
            vB.x = worldBx;
            vB.y = worldBy;
            vB.color = color;
        }

        shapeRenderer.renderLines(lineVertexData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderJoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB)
    {
        TRACY_SCOPE_N("Render joints");

        const auto& joints = simulation.getJoints();
        const size_t jointCount = joints.getCount();
        if (jointCount == 0) return;

        const float cullMinX = static_cast<float>(cameraAABB.minX);
        const float cullMinY = static_cast<float>(cameraAABB.minY);
        const float cullMaxX = static_cast<float>(cameraAABB.maxX);
        const float cullMaxY = static_cast<float>(cameraAABB.maxY);

        // Up to 2 lines per joint = 4 vertices.
        lineVertexData.clear();
        lineVertexData.reserve(jointCount * 4);

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

            const float bodyAx = static_cast<float>(bodyA.x);
            const float bodyAy = static_cast<float>(bodyA.y);
            const float bodyBx = static_cast<float>(bodyB.x);
            const float bodyBy = static_cast<float>(bodyB.y);

            // Cull the whole joint (both lines) if its bounding box misses the camera.
            const float boxMinX = std::fmin(std::fmin(bodyAx, worldAx), std::fmin(bodyBx, worldBx));
            const float boxMaxX = std::fmax(std::fmax(bodyAx, worldAx), std::fmax(bodyBx, worldBx));
            const float boxMinY = std::fmin(std::fmin(bodyAy, worldAy), std::fmin(bodyBy, worldBy));
            const float boxMaxY = std::fmax(std::fmax(bodyAy, worldAy), std::fmax(bodyBy, worldBy));

            if (boxMaxX < cullMinX || boxMinX > cullMaxX || boxMaxY < cullMinY || boxMinY > cullMaxY)
            {
                continue;
            }

            constexpr uint32_t color = 0x00FFFF;

            // Line 1: Body A position to Anchor A
            ShapeRenderer::LineVertex& vA0 = lineVertexData.emplace_back();
            vA0.x = bodyAx;
            vA0.y = bodyAy;
            vA0.color = color;

            ShapeRenderer::LineVertex& vA1 = lineVertexData.emplace_back();
            vA1.x = worldAx;
            vA1.y = worldAy;
            vA1.color = color;

            // Line 2: Body B position to Anchor B
            ShapeRenderer::LineVertex& vB0 = lineVertexData.emplace_back();
            vB0.x = bodyBx;
            vB0.y = bodyBy;
            vB0.color = color;

            ShapeRenderer::LineVertex& vB1 = lineVertexData.emplace_back();
            vB1.x = worldBx;
            vB1.y = worldBy;
            vB1.color = color;
        }

        shapeRenderer.renderLines(lineVertexData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderPrismaticJoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB)
    {
        TRACY_SCOPE_N("Render prismatic joints");

        const auto& prismaticJoints = simulation.getPrismaticJoints();
        const size_t prismaticJointCount = prismaticJoints.getCount();
        if (prismaticJointCount == 0) return;

        const float cullMinX = static_cast<float>(cameraAABB.minX);
        const float cullMinY = static_cast<float>(cameraAABB.minY);
        const float cullMaxX = static_cast<float>(cameraAABB.maxX);
        const float cullMaxY = static_cast<float>(cameraAABB.maxY);

        // Up to 2 lines per joint = 4 vertices.
        lineVertexData.clear();
        lineVertexData.reserve(prismaticJointCount * 4);

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

            // Cull the whole joint (both lines) if its bounding box misses the camera.
            const float boxMinX = std::fmin(std::fmin(worldAx, worldBx), targetX);
            const float boxMaxX = std::fmax(std::fmax(worldAx, worldBx), targetX);
            const float boxMinY = std::fmin(std::fmin(worldAy, worldBy), targetY);
            const float boxMaxY = std::fmax(std::fmax(worldAy, worldBy), targetY);

            if (boxMaxX < cullMinX || boxMinX > cullMaxX || boxMaxY < cullMinY || boxMinY > cullMaxY)
            {
                continue;
            }

            // Line 1: Anchor A to Anchor B (actual).
            ShapeRenderer::LineVertex& vA0 = lineVertexData.emplace_back();
            vA0.x = worldAx;
            vA0.y = worldAy;
            vA0.color = actualColor;

            ShapeRenderer::LineVertex& vA1 = lineVertexData.emplace_back();
            vA1.x = worldBx;
            vA1.y = worldBy;
            vA1.color = actualColor;

            // Line 2: Anchor A to axis-projected, clamped point (target).
            ShapeRenderer::LineVertex& vB0 = lineVertexData.emplace_back();
            vB0.x = worldAx;
            vB0.y = worldAy;
            vB0.color = targetColor;

            ShapeRenderer::LineVertex& vB1 = lineVertexData.emplace_back();
            vB1.x = targetX;
            vB1.y = targetY;
            vB1.color = targetColor;
        }

        shapeRenderer.renderLines(lineVertexData, viewProjectionMatrix);
    }

    void SimulationRenderer::renderRods(const Simulation& simulation, const Mat4& viewProjectionMatrix, const BodyTransformContext& transformContext, const AABB& cameraAABB)
    {
        TRACY_SCOPE_N("Render rods");

        const auto& rods = simulation.getRods();
        const size_t rodCount = rods.getCount();
        if (rodCount == 0) return;

        const float cullMinX = static_cast<float>(cameraAABB.minX);
        const float cullMinY = static_cast<float>(cameraAABB.minY);
        const float cullMaxX = static_cast<float>(cameraAABB.maxX);
        const float cullMaxY = static_cast<float>(cameraAABB.maxY);

        lineVertexData.clear();
        lineVertexData.reserve(rodCount * 2);

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

            // Cull the rod if its segment bounding box misses the camera.
            const float segMinX = std::fmin(worldAx, worldBx);
            const float segMaxX = std::fmax(worldAx, worldBx);
            const float segMinY = std::fmin(worldAy, worldBy);
            const float segMaxY = std::fmax(worldAy, worldBy);

            if (segMaxX < cullMinX || segMinX > cullMaxX || segMaxY < cullMinY || segMinY > cullMaxY)
            {
                continue;
            }

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

            ShapeRenderer::LineVertex& vA = lineVertexData.emplace_back();
            vA.x = worldAx;
            vA.y = worldAy;
            vA.color = color;

            ShapeRenderer::LineVertex& vB = lineVertexData.emplace_back();
            vB.x = worldBx;
            vB.y = worldBy;
            vB.color = color;
        }

        shapeRenderer.renderLines(lineVertexData, viewProjectionMatrix);
    }
}