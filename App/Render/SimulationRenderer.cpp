#include "SimulationRenderer.h"
#include "SimulationRenderer_Internal.h"

#include "PS_AGONY2D/Simulation.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"

#include <algorithm>
#include <utility>

namespace PS_AGONY_RENDER
{
    namespace
    {
        constexpr uint32_t previewColor = 0x00FF00;
    }

    void SimulationRenderer::init()
    {
        shapeRenderer.init();
    }

    void SimulationRenderer::updateViewProjectionMatrix()
    {
        viewProjectionMatrix = camera.getProjectionMatrix() * camera.getViewMatrix();
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

        bodies = simulation.objects().getBodies();
        colliders = simulation.objects().getColliders();
        circles = simulation.objects().getCircles();
        boxes = simulation.objects().getBoxes();
        capsules = simulation.objects().getCapsules();
        polygons = simulation.objects().getPolygons();

        updateViewProjectionMatrix();

        const BodyTransformContext transformContext = BodyTransformContext::make(bodies, simRenderAlpha, renderOptions.bodies.interpolation);

        queryCollidersForRender(simulation, cameraAABB);

        GroupColors groupColors;
        for (size_t g = 0; g < FoundColliders::groupCount; g++)
        {
            groupColors[g] = packColor(renderOptions.bodies.groupColors[g]);
        }
        renderColliders(transformContext, groupColors);

        // Color-only from here: avoids a feedback loop when sampling the id texture.
        glDrawBuffers(1, &RenderAttachments::colorDrawBuffer);

        if (renderOptions.colliders.outline)
        {
            shapeRenderer.renderOutlines(idTextureID, renderOptions.colliders.outlineColor);
        }

        ConstraintGroupColors constraintColors;
        for (size_t g = 0; g < constraintColors.size(); g++)
        {
            constraintColors[g] = packColor(renderOptions.constraints.groupColors[g]);
        }
        renderConstraints(
            simulation,
            transformContext,
            cameraAABB,
            constraintColors,
            renderOptions.constraints.icons ? renderOptions.constraints.iconSize : 0.0f
        );

        if (renderOptions.colliders.aabbs)
        {
            renderColliderAABBs(foundColliders.raw);
        }
        if (renderOptions.colliders.minExtent)
        {
            renderColliderExtents(foundColliders.raw, renderOptions.colliders.minExtentColor, colliders.minExtent, transformContext);
        }
        if (renderOptions.colliders.maxExtent)
        {
            renderColliderExtents(foundColliders.raw, renderOptions.colliders.maxExtentColor, colliders.maxExtent, transformContext);
        }

        if (renderOptions.bodies.boundRadius)
        {
            // Get body indices.
            const ObjectDenseIndex* ECSTASY_RESTRICT ownerPtr = colliders.bodyIndex;

            const size_t colliderCount = foundColliders.raw.size();

            foundBodies.clear();
            foundBodies.resize(colliderCount);
            for (size_t i = 0; i < colliderCount; i++)
            {
                ObjectDenseIndex colliderIndex = foundColliders.raw[i];
                foundBodies[i] = ownerPtr[colliderIndex];
            }

            // Dedupe.
            std::sort(foundBodies.begin(), foundBodies.end());
            foundBodies.erase(std::unique(foundBodies.begin(), foundBodies.end()), foundBodies.end());
            
            renderBodyBoundRadiuses(foundBodies, renderOptions.bodies.boundRadiusColor);
        }

        renderBroadPhaseAABBs(simulation, cameraAABB, renderOptions.broadphaseAABBs);

        if (renderOptions.contactPoints)
        {
            renderContactPoints(simulation, cameraAABB);
        }
    }

    void SimulationRenderer::renderBroadPhaseAABBs(const Simulation& simulation, const AABB& cameraAABB, const RenderOptions::BroadPhaseAABBs& options)
    {
        TRACY_SCOPE_N("Broad phase leaf AABBs");

        using TreeType = BroadPhaseInternal::BvhTreeType;
        using TreeFlag = BroadPhaseInternal::BvhTreeTypeFlag;

        constexpr std::pair<TreeType, TreeFlag> trees[] = {
            { TreeType::Dynamic, TreeFlag::Dynamic },
            { TreeType::Static, TreeFlag::Static },
            { TreeType::Sleeping, TreeFlag::Sleeping }
        };

        const auto& detector = simulation.getBroadPhaseCollisionDetector();
        const BroadPhaseInternal::AABBQuery query(cameraAABB);

        for (const auto& [type, flag] : trees)
        {
            if (!options.enabled[size_t(type)]) continue;

            queriedAABBs.clear();
            detector.queryLeafNodeAABBsInShape(query, flag, queriedAABBs);
            renderLeafAABBs(queriedAABBs, options.colors[size_t(type)]);
        }
    }

    const char* SimulationRenderer::renderObjectPreview(const void* params, ColliderShapeType type)
    {
        if (params == nullptr) return nullptr;

        updateViewProjectionMatrix();

        if (type == ColliderShapeType::Circle)
        {
            const auto* circleParams = static_cast<const CircleCreateParams*>(params);

            circleInstanceData.assign(1, ShapeRenderer::CircleInstanceData{
                static_cast<float>(circleParams->base.position.x),
                static_cast<float>(circleParams->base.position.y),
                static_cast<float>(circleParams->base.rotation),
                static_cast<float>(circleParams->radius),
                previewColor, 0
                });

            shapeRenderer.renderCircleShapes(circleInstanceData, viewProjectionMatrix);
        }
        else if (type == ColliderShapeType::Box)
        {
            const auto* boxParams = static_cast<const BoxCreateParams*>(params);

            boxInstanceData.assign(1, ShapeRenderer::BoxInstanceData{
                static_cast<float>(boxParams->base.position.x),
                static_cast<float>(boxParams->base.position.y),
                static_cast<float>(boxParams->base.rotation),
                static_cast<float>(boxParams->size.x * 0.5),
                static_cast<float>(boxParams->size.y * 0.5),
                previewColor, 0
                });

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
                        cache.commands.push_back({
                            static_cast<uint32_t>(piece.size()),
                            1,
                            static_cast<uint32_t>(cache.vertices.size()),
                            static_cast<uint32_t>(cache.commands.size())
                            });

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

            polygonInstanceData.assign(cache.commands.size(), ShapeRenderer::PolygonInstanceData{
                static_cast<float>(polyParams->base.position.x),
                static_cast<float>(polyParams->base.position.y),
                static_cast<float>(polyParams->base.rotation),
                previewColor, 0
                });

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

        // Count per bin and cache shape.
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

        // Scatter.
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

        foundColliders.build(colliders);
    }

    size_t SimulationRenderer::getMemoryUsage() const noexcept
    {
        size_t total = 0;

        total += getVectorMemoryUsage(foundColliders.raw);
        total += getVectorMemoryUsage(foundColliders.sorted);
        total += getVectorMemoryUsage(foundColliders.shapes);

        total += getVectorMemoryUsage(queriedAABBs);

        total += getVectorMemoryUsage(circleInstanceData);
        total += getVectorMemoryUsage(simplePointInstanceData);
        total += getVectorMemoryUsage(boxInstanceData);

        total += getVectorMemoryUsage(polygonVertexData);
        total += getVectorMemoryUsage(polygonInstanceData);
        total += getVectorMemoryUsage(polygonDrawCommands);

        total += getVectorMemoryUsage(aabbInstanceData);
        total += getVectorMemoryUsage(lineVertexData);
        total += getVectorMemoryUsage(constraintIconData);
        total += getVectorMemoryUsage(hollowCircleData);

        total += getVectorMemoryUsage(polygonPreviewCache.sourceVertices);
        total += getVectorMemoryUsage(polygonPreviewCache.vertices);
        total += getVectorMemoryUsage(polygonPreviewCache.commands);

        return total;
    }
}