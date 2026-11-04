#include "SimulationRenderer.h"
#include "SimulationRenderer_Internal.h"

#include "PS_AGONY2D/Simulation.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"

#include <algorithm>
#include <cmath>

namespace PS_AGONY_RENDER
{
    namespace
    {
        uint32_t packColor(const glm::vec3& c) noexcept
        {
            const auto to8 = [](float v) -> uint32_t
                {
                    return static_cast<uint32_t>(std::fmax(0.0f, std::fmin(1.0f, v)) * 255.0f + 0.5f);
                };
            return (to8(c.x) << 16) | (to8(c.y) << 8) | to8(c.z);
        }
    }

    void SimulationRenderer::init()
    {
        TRACY_SCOPE_N("SimulationRenderer init");

        shapeRenderer.init();
    }

    void SimulationRenderer::renderSimulation(
        const Simulation& simulation,
        Real simRenderAlpha,
        const AABB& cameraAABB,
        const RenderOptions& renderOptions,
        GLuint idTextureID
    )
    {
        TRACY_SCOPE_N("Render simulation");

        // Set references.
        bodies = simulation.objects().getBodies();
        colliders = simulation.objects().getColliders();
        circles = simulation.objects().getCircles();
        boxes = simulation.objects().getBoxes();
        capsules = simulation.objects().getCapsules();
        polygons = simulation.objects().getPolygons();

        // Camera.
        const Mat4 viewMatrix = camera.getViewMatrix();
        const Mat4 projectionMatrix = camera.getProjectionMatrix();
        const Mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

        // Built once per frame and threaded through every render function below,
        // instead of each one re-fetching the same body pointers and redoing the
        // same lerp/wrap math. When renderOptions.interpolation is false, sampling
        // a body transform skips that math entirely (see BodyTransformContext::sample).
        const BodyTransformContext transformContext = BodyTransformContext::make(bodies, simRenderAlpha, renderOptions.interpolation);

        // Fetch colliders which overlap camera AABB.
        queryCollidersForRender(simulation, cameraAABB);

        // Render.
        GroupColors groupColors;
        for (size_t g = 0; g < FoundColliders::groupCount; g++)
        {
            groupColors[g] = packColor(renderOptions.bodyGroupColors[g]);
        }
        renderColliders(viewProjectionMatrix, transformContext, groupColors);

        // Color-only from here: avoids a feedback loop when sampling the id texture.
        glDrawBuffers(1, &RenderAttachments::colorDrawBuffer);

        if (renderOptions.outlineColliders)
        {
            renderOutlines(idTextureID, renderOptions.outlineColor);
        }

        renderSprings(simulation, viewProjectionMatrix, transformContext, cameraAABB);
        renderJoints(simulation, viewProjectionMatrix, transformContext, cameraAABB);
        renderPrismaticJoints(simulation, viewProjectionMatrix, transformContext, cameraAABB);

        if (renderOptions.colliderAABBs)
        {
            renderColliderAABBs(foundColliders.raw, viewProjectionMatrix);
        }

        if (
            renderOptions.broadPhaseDynamicAABBs ||
            renderOptions.broadPhaseStaticAABBs ||
            renderOptions.broadPhaseSleepingAABBs
            )
        {
            TRACY_SCOPE_N("Broad phase leaf AABBs");

            const auto& bfcd = simulation.getBroadPhaseCollisionDetector();
            const BroadPhaseInternal::AABBQuery query(cameraAABB);

            if (renderOptions.broadPhaseDynamicAABBs)
            {
                queriedAABBs.clear();
                bfcd.queryLeafNodeAABBsInShape(query, BroadPhaseInternal::BvhTreeTypeFlag::Dynamic, queriedAABBs);
                renderLeafAABBs(queriedAABBs, renderOptions.broadPhaseDynamicAABBColor, viewProjectionMatrix);
            }
            if (renderOptions.broadPhaseStaticAABBs)
            {
                queriedAABBs.clear();
                bfcd.queryLeafNodeAABBsInShape(query, BroadPhaseInternal::BvhTreeTypeFlag::Static, queriedAABBs);
                renderLeafAABBs(queriedAABBs, renderOptions.broadPhaseStaticAABBColor, viewProjectionMatrix);
            }
            if (renderOptions.broadPhaseSleepingAABBs)
            {
                queriedAABBs.clear();
                bfcd.queryLeafNodeAABBsInShape(query, BroadPhaseInternal::BvhTreeTypeFlag::Sleeping, queriedAABBs);
                renderLeafAABBs(queriedAABBs, renderOptions.broadPhaseSleepingAABBColor, viewProjectionMatrix);
            }
        }
        if (renderOptions.contactPoints)
        {
            renderContactPoints(simulation, viewProjectionMatrix, cameraAABB);
        }

        //renderBodyCentersOfMass(viewProjectionMatrix, cameraAABB); // Red.
        //renderBodyTruePositions(viewProjectionMatrix, cameraAABB); // Green.
        //renderBodyPositions(viewProjectionMatrix, cameraAABB); // Blue.
    }

    const char* SimulationRenderer::renderObjectPreview(const void* params, ColliderShapeType type)
    {
        if (params == nullptr) return nullptr;

        // Get matrices.
        const Mat4 viewMatrix = camera.getViewMatrix();
        const Mat4 projectionMatrix = camera.getProjectionMatrix();
        const Mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

        // Draw body depending on the type.
        if (type == ColliderShapeType::Circle)
        {
            const auto* circleParams = static_cast<const CircleCreateParams*>(params);

            circleInstanceData.resize(1);
            ShapeRenderer::CircleInstanceData& data = circleInstanceData[0];

            data.positionX = static_cast<float>(circleParams->base.position.x);
            data.positionY = static_cast<float>(circleParams->base.position.y);
            data.localCOMX = 0.0f;
            data.localCOMY = 0.0f;
            data.rotation = static_cast<float>(circleParams->base.rotation);
            data.radius = static_cast<float>(circleParams->radius);

            data.color = 0x00FF00;

            shapeRenderer.renderCircleShapes(circleInstanceData, viewProjectionMatrix);
        }
        else if (type == ColliderShapeType::Box)
        {
            const auto* boxParams = static_cast<const BoxCreateParams*>(params);

            boxInstanceData.resize(1);
            ShapeRenderer::BoxInstanceData& data = boxInstanceData[0];

            data.positionX = static_cast<float>(boxParams->base.position.x);
            data.positionY = static_cast<float>(boxParams->base.position.y);
            data.localCOMX = 0.0f;
            data.localCOMY = 0.0f;
            data.rotation = static_cast<float>(boxParams->base.rotation);
            data.halfWidth = static_cast<float>(boxParams->size.x * 0.5);
            data.halfHeight = static_cast<float>(boxParams->size.y * 0.5);

            data.color = 0x00FF00;

            shapeRenderer.renderBoxShapes(boxInstanceData, viewProjectionMatrix);
        }
        else if (type == ColliderShapeType::Polygon)
        {
            const auto* polyParams = static_cast<const PolygonCreateParams*>(params);
            if (polyParams->localVertices.data() == nullptr) return nullptr;

            const std::span<const Vec2> src = polyParams->localVertices;
            PolygonPreviewCache& cache = polygonPreviewCache;

            const bool same = cache.valid
                && cache.concaveMethod == polyParams->concaveDecompositionMethod
                && cache.convexMethod == polyParams->convexDecompositionMethod
                && cache.centerOfMass == polyParams->base.centerOfMass
                && cache.sourceVertices.size() == src.size()
                && std::equal(src.begin(), src.end(), cache.sourceVertices.begin());

            if (!same)
            {
                // Same pipeline as createPolygon, so preview == result (decomposition, checks, errors).
                const PhysicsGeometry::PolygonBuild build = PhysicsGeometry::buildPolygon(
                    src, polyParams->concaveDecompositionMethod, Real(1), polyParams->base.centerOfMass,
                    polyParams->convexDecompositionMethod
                );

                cache.valid = true;
                cache.sourceVertices.assign(src.begin(), src.end());
                cache.centerOfMass = polyParams->base.centerOfMass;
                cache.concaveMethod = polyParams->concaveDecompositionMethod;
                cache.convexMethod = polyParams->convexDecompositionMethod;
                cache.error = build.error;
                cache.vertices.clear();
                cache.commands.clear();

                if (build.ok())
                {
                    // Pieces are COM-centered; shift back so the preview is drawn in the creator's
                    // frame and rotates about its origin, exactly like the created body.
                    for (const std::vector<Vec2>& piece : build.pieces)
                    {
                        DrawArraysIndirectCommand cmd{};
                        cmd.count = static_cast<uint32_t>(piece.size());
                        cmd.instanceCount = 1;
                        cmd.first = static_cast<uint32_t>(cache.vertices.size());
                        cmd.baseInstance = static_cast<uint32_t>(cache.commands.size());
                        cache.commands.push_back(cmd);

                        for (const Vec2& v : piece)
                        {
                            const Vec2 o = v + build.trueCenterOfMass;
                            cache.vertices.emplace_back(static_cast<float>(o.x), static_cast<float>(o.y));
                        }
                    }
                }
            }

            if (cache.error != PhysicsGeometry::PolygonBuildError::None)
            {
                return PhysicsGeometry::polygonBuildErrorMessage(cache.error);
            }

            const size_t pieceCount = cache.commands.size();
            shapeRenderer.ensurePolygonBufferCapacity(cache.vertices.size(), pieceCount);

            polygonInstanceData.resize(pieceCount);
            for (ShapeRenderer::PolygonInstanceData& inst : polygonInstanceData)
            {
                inst.positionX = static_cast<float>(polyParams->base.position.x);
                inst.positionY = static_cast<float>(polyParams->base.position.y);
                inst.localCOMX = 0.0f;
                inst.localCOMY = 0.0f;
                inst.rotation = static_cast<float>(polyParams->base.rotation);
                inst.color = 0x00FF00;
            }

            shapeRenderer.renderPolygonShapes(cache.vertices, polygonInstanceData, cache.commands, viewProjectionMatrix);
        }

        return nullptr;
    }

    void SimulationRenderer::FoundColliders::build(const ColliderSoAViewer& colliders)
    {
        const size_t n = raw.size();
        shapes.resize(n);
        sorted.resize(n);
        offsets.fill(0);

        // Pass 1: count per bin, cache shape.
        for (size_t g = 0; g < groupCount; g++)
        {
            for (size_t i = groupBegin[g]; i < groupBegin[g + 1]; i++)
            {
                const size_t shape = size_t(colliders.shapeType[raw[i]]);
                shapes[i] = static_cast<uint8_t>(shape);
                offsets[shape * groupCount + g + 1]++;
            }
        }

        for (size_t b = 0; b < binCount; b++)
        {
            offsets[b + 1] += offsets[b];
        }

        // Pass 2: scatter.
        std::array<uint32_t, binCount> cursor;
        std::copy_n(offsets.begin(), binCount, cursor.begin());

        for (size_t g = 0; g < groupCount; g++)
        {
            for (size_t i = groupBegin[g]; i < groupBegin[g + 1]; i++)
            {
                sorted[cursor[size_t(shapes[i]) * groupCount + g]++] = raw[i];
            }
        }
    }

    void SimulationRenderer::queryCollidersForRender(const Simulation& simulation, const AABB& cameraAABB)
    {
        {
            TRACY_SCOPE_N("Find colliders for render");

            const auto& collisionDetector = simulation.getBroadPhaseCollisionDetector();
            const BroadPhaseInternal::AABBQuery query(cameraAABB);

            using TreeFlag = BroadPhaseInternal::BvhTreeTypeFlag;
            constexpr TreeFlag groupTree[FoundColliders::groupCount] = {
                TreeFlag::Dynamic, TreeFlag::Static, TreeFlag::Sleeping // Indexed by BodyGroup.
            };

            foundColliders.raw.clear();
            for (size_t g = 0; g < FoundColliders::groupCount; g++)
            {
                foundColliders.groupBegin[g] = static_cast<uint32_t>(foundColliders.raw.size());
                collisionDetector.queryCollidersInShape(query, groupTree[g], foundColliders.raw);
            }
            foundColliders.groupBegin[FoundColliders::groupCount] = static_cast<uint32_t>(foundColliders.raw.size());
        }
        {
            TRACY_SCOPE_N("Partition found colliders");

            foundColliders.build(colliders);
        }
    }

    size_t SimulationRenderer::getMemoryUsage() const noexcept
    {
        size_t total = 0;

        total += getVectorMemoryUsage(foundColliders.raw);
        total += getVectorMemoryUsage(foundColliders.sorted);
        total += getVectorMemoryUsage(foundColliders.shapes);

        total += getVectorMemoryUsage(circleInstanceData);
        total += getVectorMemoryUsage(simplePointInstanceData);
        total += getVectorMemoryUsage(boxInstanceData);

        total += getVectorMemoryUsage(polygonVertexData);
        total += getVectorMemoryUsage(polygonInstanceData);
        total += getVectorMemoryUsage(polygonDrawCommands);

        total += getVectorMemoryUsage(lineVertexData);

        total += getVectorMemoryUsage(aabbInstanceData);

        return total;
    }

    size_t SimulationRenderer::getVideoMemoryUsage() const noexcept
    {
        size_t total = 0;

        total += shapeRenderer.circleResources.vbo.getCapacity();
        total += shapeRenderer.circleResources.instanceVbo.getCapacity();

        total += shapeRenderer.simplePointCircleResources.vbo.getCapacity();
        total += shapeRenderer.simplePointCircleResources.instanceVbo.getCapacity();

        total += shapeRenderer.boxResources.vbo.getCapacity();
        total += shapeRenderer.boxResources.instanceVbo.getCapacity();

        total += shapeRenderer.capsuleResources.vbo.getCapacity();
        total += shapeRenderer.capsuleResources.instanceVbo.getCapacity();

        total += shapeRenderer.polygonResources.vertexVbo.getCapacity();
        total += shapeRenderer.polygonResources.instanceVbo.getCapacity();
        total += shapeRenderer.polygonResources.indirectBuf.getCapacity();

        total += shapeRenderer.lineResources.vbo.getCapacity();

        total += shapeRenderer.aabbResources.vbo.getCapacity();
        total += shapeRenderer.aabbResources.instanceVbo.getCapacity();

        return total;
    }

    void SimulationRenderer::renderOutlines(GLuint idTextureID, const glm::vec3& outlineColor)
    {
        shapeRenderer.renderOutlines(idTextureID, outlineColor);
    }
}