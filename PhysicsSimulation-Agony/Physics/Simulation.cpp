#include "Simulation.h"
#include "Threading.h"
#include "FastCosSin.h"

#include "EcstasyCore/TracyProfiler.h"
#include "EcstasyCore/Portablity.h"

#include <iostream>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <numeric>

namespace PS_AGONY
{
    using RealSimd = Ecstasy::Simd<Real>;


    static Real calculateCircleInertia(Real mass, Real radius, Vec2 centerOfMass)
    {
        const Real radiusSquared = radius * radius;
        const Real deltaSquared = glm::dot(centerOfMass, centerOfMass);
        return (Real(0.5) * radiusSquared + deltaSquared) * mass;
    }

    static Real calculateBoxInertia(Real mass, Real width, Real height, Vec2 centerOfMass)
    {
        constexpr Real div = 1.0 / 12.0;
        const Real deltaSquared = glm::dot(centerOfMass, centerOfMass);
        return mass * (div * (width * width + height * height) + deltaSquared);
    }

    static std::pair<Real, Vec2> calculatePolygonInertia(
        Real mass,
        VerticesContainer& verticesContainer,
        std::optional<Vec2> centerOfMass = std::nullopt
    )
    {
        const size_t verticesCount = verticesContainer.size();
        if (verticesCount < 3)
        {
            return { Real(0), centerOfMass.value_or(Vec2(Real(0))) };
        }

        Real signedArea = Real(0);
        Real cx = Real(0);
        Real cy = Real(0);
        Real xx = Real(0);
        Real yy = Real(0);

        const bool computeCOM = !centerOfMass.has_value();
        const Vec2* verticesPtr = verticesContainer.data();

        for (size_t i = 0; i < verticesCount; i++)
        {
            const Vec2& p0 = verticesPtr[i];
            const Vec2& p1 = verticesPtr[(i + 1) % verticesCount];

            Real cross = p0.x * p1.y - p1.x * p0.y;
            signedArea += cross;

            if (computeCOM)
            {
                cx += (p0.x + p1.x) * cross;
                cy += (p0.y + p1.y) * cross;
            }

            // Area moments about origin.
            xx += (p0.y * p0.y + p0.y * p1.y + p1.y * p1.y) * cross;
            yy += (p0.x * p0.x + p0.x * p1.x + p1.x * p1.x) * cross;
        }

        // If winding order is clockwise, reverse the container to make it counter-clockwise.
        // Since all accumulated values are linear with respect to 'cross', we can just negate them.
        if (signedArea < Real(0))
        {
            std::reverse(verticesContainer.begin(), verticesContainer.end());
            signedArea = -signedArea;
            xx = -xx;
            yy = -yy;
            if (computeCOM)
            {
                cx = -cx;
                cy = -cy;
            }
        }

        signedArea *= Real(0.5);
        const Real absoluteArea = signedArea;
        if (absoluteArea < std::numeric_limits<Real>::epsilon())
        {
            return { Real(0), centerOfMass.value_or(Vec2(Real(0))) };
        }

        // Determine final Center of Mass.
        Vec2 finalCOM;
        if (computeCOM)
        {
            finalCOM = Vec2(
                cx / (Real(6) * signedArea),
                cy / (Real(6) * signedArea)
            );
        }
        else
        {
            finalCOM = centerOfMass.value();
        }

        xx /= Real(12);
        yy /= Real(12);

        // Local/World polar moment of area scaled to mass moment
        Real inertia = (mass / absoluteArea) * (xx + yy);

        // If COM was explicitly provided, treat vertices as local space 
        // and shift to world origin via the parallel axis theorem.
        if (!computeCOM)
        {
            Real deltaSquared = glm::dot(finalCOM, finalCOM);
            inertia += mass * deltaSquared;
        }

        return { inertia, finalCOM };
    }


    static double percentileFromSorted(const std::vector<double>& sorted, double p)
    {
        if (sorted.empty()) return 0.0;

        // Nearest-rank style percentile, clamped.
        const double rank = p * (static_cast<double>(sorted.size()) - 1.0);
        const size_t idx = static_cast<size_t>(std::round(rank));
        return sorted[std::min(idx, sorted.size() - 1)];
    }

    static void computeStats(const std::vector<double>& samples,
        double& meanOut,
        double& medianOut,
        double& p90Out,
        double& p99Out,
        double& minOut,
        double& maxOut,
        double& stdDevOut)
    {
        if (samples.empty())
        {
            meanOut = medianOut = p90Out = p99Out = minOut = maxOut = stdDevOut = 0.0;
            return;
        }

        std::vector<double> sorted = samples;
        std::sort(sorted.begin(), sorted.end());

        const double sum = std::accumulate(sorted.begin(), sorted.end(), 0.0);
        meanOut = sum / static_cast<double>(sorted.size());

        if (sorted.size() % 2 == 0)
        {
            const size_t mid = sorted.size() / 2;
            medianOut = (sorted[mid - 1] + sorted[mid]) * 0.5;
        }
        else
        {
            medianOut = sorted[sorted.size() / 2];
        }

        p90Out = percentileFromSorted(sorted, 0.90);
        p99Out = percentileFromSorted(sorted, 0.99);
        minOut = sorted.front();
        maxOut = sorted.back();

        double variance = 0.0;
        for (double v : sorted)
        {
            const double d = v - meanOut;
            variance += d * d;
        }
        variance /= static_cast<double>(sorted.size());
        stdDevOut = std::sqrt(variance);
    }
    

    Simulation::Simulation()
    {
        materials.reserve(16);
        materials.emplace_back(); // Default material.

        // Spawn a thread pool.
        auto& threadPool = Threading::getGlobalThreadPool();
        (void)threadPool;
    }

    void Simulation::update(Real deltaTime)
    {
        TRACY_SCOPE_NC("Simulation update", Ecstasy::Color::Wheat);

        // Delta time check.
        if (deltaTime <= 0) return;

        // Cap how much real time this call is allowed to consume.
        const Real cappedDeltaTime = std::fmin(deltaTime, simulationSettings.maxDeltaTimePerUpdateCall);

        // Advance counter.
        updateTimeAccumulator += cappedDeltaTime;

        // Physics steps.
        uint32_t stepCount = std::floor(updateTimeAccumulator / simulationSettings.updateInterval);
        updateTimeAccumulator -= stepCount * simulationSettings.updateInterval;

        const Real fixedDeltaTime = simulationSettings.updateInterval * simulationSettings.timeScale;
        if (stepCount > 0)
        {
            for (uint32_t i = 0; i < stepCount; i++)
            {
                physicsStep(fixedDeltaTime);
            }
            postUpdate();
        }

        // Debug data.
        {
            constexpr Real DEBUG_DATA_SWITCH_INTERVAL = 0.25;

            debugDataResetTimeAccumulator += deltaTime;
            if (debugDataResetTimeAccumulator > DEBUG_DATA_SWITCH_INTERVAL)
            {
                debugDataSnaphot = runtimeDebugData;

                debugDataResetTimeAccumulator = 0.0;

                runtimeDebugData.updatesHappened = 0;

                collectMemoryUsage(debugDataSnaphot);
            }

            runtimeDebugData.updatesHappened += stepCount / DEBUG_DATA_SWITCH_INTERVAL;
            runtimeDebugData.updatesSupposedToHappen = std::floor(Real(1.0) / simulationSettings.updateInterval);
        }
    }

    void Simulation::createCircle(const CircleCreateParams& params)
    {
        const BodyIndex newBodyIndex = bodies.getCount();
        const BodyIndex newShapeIndex = circles.getCount();

        const Real mass   = std::fmax(Real(0), params.base.mass);
        const Real radius = std::fmax(Real(0), params.radius);
        const Vec2 centerOfMass = params.base.centerOfMass.value_or(Vec2(0));
        const MaterialIndex materialIndex = params.base.materialIndex < materials.size() ? params.base.materialIndex : 0;

        const Real inertia = calculateCircleInertia(mass, radius, centerOfMass);

        const Real invMass = mass == 0.0 ? 0.0 : 1.0 / mass;
        const Real invInertia = inertia == 0.0 ? 0.0 : 1.0 / inertia;

        bodies.append(
            params.base.position,
            params.base.velocity,
            params.base.rotation,
            params.base.angularVelocity,
            mass,    invMass,
            inertia, invInertia,
			centerOfMass,
            materialIndex,
            BodyType::Circle,
            newShapeIndex
		);

        circles.append(
            newBodyIndex,
            radius
		);
    }

    void Simulation::createBox(const BoxCreateParams& params)
    {
        const BodyIndex newBodyIndex = bodies.getCount();
        const BodyIndex newShapeIndex = boxes.getCount();

        const Real mass = std::fmax(Real(0), params.base.mass);
        const Real width = std::fmax(Real(0), params.size.x);
        const Real height = std::fmax(Real(0), params.size.y);
        const Vec2 centerOfMass = params.base.centerOfMass.value_or(Vec2(0));
        const MaterialIndex materialIndex = params.base.materialIndex < materials.size() ? params.base.materialIndex : 0;

        const Real inertia = calculateBoxInertia(mass, width, height, centerOfMass);

        const Real invMass = mass == 0.0 ? 0.0 : 1.0 / mass;
        const Real invInertia = inertia == 0.0 ? 0.0 : 1.0 / inertia;

        bodies.append(
            params.base.position,
            params.base.velocity,
            params.base.rotation,
            params.base.angularVelocity,
            mass,    invMass,
            inertia, invInertia,
			centerOfMass,
            materialIndex,
            BodyType::Box,
            newShapeIndex
        );

        boxes.append(
            newBodyIndex,
            width  * Real(0.5),
            height * Real(0.5)
        );
    }

    void Simulation::createPolygon(const PolygonCreateParams& params)
    {
        if (params.localVertices == nullptr)
        {
            std::cerr << "[AGONY][Simulation::createPolygon]: Failed to create a polygon: Vertices container is nullptr.\n";
            return;
        }
        if (params.verticesCount < 3)
        {
            std::cerr << "[AGONY][Simulation::createPolygon]: Failed to create a polygon: Vertices count is less than three.\n";
            return;
        }
        //if (params.base.centerOfMass.has_value())
        //{
        //    // I just don't know how to make it work with my 'true positions' and other stuff.
        //    std::cerr << "[AGONY][Simulation::createPolygon]: Failed to create a polygon: Custom center of mass is not supported.\n";
        //    return;
        //}

        const BodyIndex newBodyIndex = bodies.getCount();
        const BodyIndex newShapeIndex = polygons.getCount();

        const Real mass = std::fmax(Real(0), params.base.mass);
        const MaterialIndex materialIndex = params.base.materialIndex < materials.size() ? params.base.materialIndex : 0;

        VerticesContainer vertices{ params.localVertices, params.verticesCount };

        auto iCOM = calculatePolygonInertia(mass, vertices, std::nullopt);
        const Real inertia = iCOM.first;
        Vec2 trueCenterOfMass = iCOM.second;
        Vec2 neededCenterOfMass = params.base.centerOfMass.value_or(trueCenterOfMass);

        // This makes: position == true position == world COM.
        // TODO: This probably invalidates inertia. Need second pass.
        for (Vec2& v : vertices)
        {
            v -= trueCenterOfMass;
        }
        neededCenterOfMass -= trueCenterOfMass;

        const Real invMass = mass == 0.0 ? 0.0 : 1.0 / mass;
        const Real invInertia = inertia == 0.0 ? 0.0 : 1.0 / inertia;

        bodies.append(
            params.base.position + trueCenterOfMass,
            params.base.velocity,
            params.base.rotation,
            params.base.angularVelocity,
            mass, invMass,
            inertia, invInertia,
            neededCenterOfMass,
            materialIndex,
            BodyType::Polygon,
            newShapeIndex
        );

        polygons.append(
            newBodyIndex,
            std::move(vertices)
        );
    }

    void Simulation::destroyBody(BodyIndex bodyIndex)
    {
        const size_t bodyCount = bodies.getCount();
        if (bodyIndex >= bodyCount) return;

        const BodyType type = bodies.bodyType[bodyIndex];
        const BodyIndex shapeIdx = bodies.shapeIndex[bodyIndex];

        // Remove shape entry from the appropriate SoA.
        if (type == BodyType::Circle)
        {
            const size_t circleCount = circles.getCount();
            if (shapeIdx < circleCount)
            {
                // Swap with last element if not already last.
                if (shapeIdx != circleCount - 1)
                {
                    // Swap body indices in circles.
                    std::swap(circles.bodyIndices[shapeIdx], circles.bodyIndices.back());
                    std::swap(circles.radius[shapeIdx], circles.radius.back());

                    // Update the body that now occupies shapeIdx to point to the new shape index.
                    const BodyIndex swappedBody = circles.bodyIndices[shapeIdx];
                    bodies.shapeIndex[swappedBody] = shapeIdx;
                }
                circles.bodyIndices.pop_back();
                circles.radius.pop_back();
            }
        }
        else if (type == BodyType::Box)
        {
            const size_t boxCount = boxes.getCount();
            if (shapeIdx < boxCount)
            {
                if (shapeIdx != boxCount - 1)
                {
                    std::swap(boxes.bodyIndices[shapeIdx], boxes.bodyIndices.back());
                    std::swap(boxes.halfWidth[shapeIdx], boxes.halfWidth.back());
                    std::swap(boxes.halfHeight[shapeIdx], boxes.halfHeight.back());

                    const BodyIndex swappedBody = boxes.bodyIndices[shapeIdx];
                    bodies.shapeIndex[swappedBody] = shapeIdx;
                }
                boxes.bodyIndices.pop_back();
                boxes.halfWidth.pop_back();
                boxes.halfHeight.pop_back();
            }
        }

        // Remove body entry from BodySoA.
        if (bodyIndex != bodyCount - 1)
        {
            // Swap all vectors in BodySoA.
            std::swap(bodies.offsetX[bodyIndex], bodies.offsetX.back());
            std::swap(bodies.offsetY[bodyIndex], bodies.offsetY.back());
            std::swap(bodies.localCenterOfMassX[bodyIndex], bodies.localCenterOfMassX.back());
            std::swap(bodies.localCenterOfMassY[bodyIndex], bodies.localCenterOfMassY.back());
            std::swap(bodies.worldCenterX[bodyIndex], bodies.worldCenterX.back());
            std::swap(bodies.worldCenterY[bodyIndex], bodies.worldCenterY.back());
            std::swap(bodies.velocityX[bodyIndex], bodies.velocityX.back());
            std::swap(bodies.velocityY[bodyIndex], bodies.velocityY.back());
            std::swap(bodies.rotation[bodyIndex], bodies.rotation.back());
            std::swap(bodies.angularVelocity[bodyIndex], bodies.angularVelocity.back());
            std::swap(bodies.mass[bodyIndex], bodies.mass.back());
            std::swap(bodies.invMass[bodyIndex], bodies.invMass.back());
            std::swap(bodies.inertia[bodyIndex], bodies.inertia.back());
            std::swap(bodies.invInertia[bodyIndex], bodies.invInertia.back());
            std::swap(bodies.rotationCos[bodyIndex], bodies.rotationCos.back());
            std::swap(bodies.rotationSin[bodyIndex], bodies.rotationSin.back());
            std::swap(bodies.isStatic[bodyIndex], bodies.isStatic.back());
            std::swap(bodies.materialIndex[bodyIndex], bodies.materialIndex.back());
            std::swap(bodies.aabb.minX[bodyIndex], bodies.aabb.minX.back());
            std::swap(bodies.aabb.minY[bodyIndex], bodies.aabb.minY.back());
            std::swap(bodies.aabb.maxX[bodyIndex], bodies.aabb.maxX.back());
            std::swap(bodies.aabb.maxY[bodyIndex], bodies.aabb.maxY.back());
            std::swap(bodies.bodyType[bodyIndex], bodies.bodyType.back());
            std::swap(bodies.shapeIndex[bodyIndex], bodies.shapeIndex.back());

            // Update the shape entry that refers to the swapped body (if any).
            //const BodyIndex swappedBodyIndex = bodyCount - 1;
            const BodyType swappedType = bodies.bodyType[bodyIndex];
            const BodyIndex swappedShapeIdx = bodies.shapeIndex[bodyIndex];
            if (swappedType == BodyType::Circle)
            {
                if (swappedShapeIdx < circles.getCount())
                    circles.bodyIndices[swappedShapeIdx] = bodyIndex;
            }
            else if (swappedType == BodyType::Box)
            {
                if (swappedShapeIdx < boxes.getCount())
                    boxes.bodyIndices[swappedShapeIdx] = bodyIndex;
            }

            // Record deletion.
            deletedBodies.emplace_back( bodyIndex, static_cast<BodyIndex>(bodyCount - 1) );
        }

        // Pop back all BodySoA vectors.
        bodies.offsetX.pop_back();
        bodies.offsetY.pop_back();
        bodies.localCenterOfMassX.pop_back();
        bodies.localCenterOfMassY.pop_back();
        bodies.worldCenterX.pop_back();
        bodies.worldCenterY.pop_back();
        bodies.velocityX.pop_back();
        bodies.velocityY.pop_back();
        bodies.rotation.pop_back();
        bodies.angularVelocity.pop_back();
        bodies.mass.pop_back();
        bodies.invMass.pop_back();
        bodies.inertia.pop_back();
        bodies.invInertia.pop_back();
        bodies.rotationCos.pop_back();
        bodies.rotationSin.pop_back();
        bodies.isStatic.pop_back();
        bodies.materialIndex.pop_back();
        bodies.aabb.minX.pop_back();
        bodies.aabb.minY.pop_back();
        bodies.aabb.maxX.pop_back();
        bodies.aabb.maxY.pop_back();
        bodies.bodyType.pop_back();
        bodies.shapeIndex.pop_back();
    }

    MaterialIndex Simulation::createMaterial(const Material& material)
    {
        const MaterialIndex materialIndex = materials.size();
        materials.push_back(material);
        return materialIndex;
    }

    void Simulation::mainBodyHolderGrabAt(Vec2 grabPosition)
    {
        if (mainBodyHolder.heldBody.has_value()) return;

        constexpr Real MAX_GRAB_DISTANCE = 4.0;
        constexpr Real MAX_GRAB_DISTANCE_SQ = MAX_GRAB_DISTANCE * MAX_GRAB_DISTANCE;

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY.data();
        const Real* ECSTASY_RESTRICT worldCenterXPtr = bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT worldCenterYPtr = bodies.worldCenterY.data();
        const Real* ECSTASY_RESTRICT massPtr = bodies.mass.data();

        Real minSqDistance = FLT_MAX;
        BodyIndex closestBody;
        Vec2 closestBodyDelta;

        const uint32_t bodyCount = bodies.getCount();
        for (uint32_t i = 0; i < bodyCount; i++)
        {
            if (massPtr[i] == 0) continue;

            const Vec2 bodyTruePosition = { worldCenterXPtr[i], worldCenterYPtr[i] };

            const Vec2 delta = bodyTruePosition - grabPosition;

            const Real sqDistance = glm::dot(delta, delta);

            if (sqDistance > MAX_GRAB_DISTANCE_SQ) continue;
            else if (sqDistance < minSqDistance)
            {
                const Vec2 bodyPosition = { positionXPtr[i], positionYPtr[i] };

                minSqDistance = sqDistance;
                closestBody = i;
                closestBodyDelta = bodyPosition - grabPosition;
            }
        }

        // Grab.
        if (minSqDistance < FLT_MAX)
        {
            mainBodyHolder.heldBody = closestBody;
            mainBodyHolder.bodyOffset = closestBodyDelta;
        }
    }

    void Simulation::mainBodyHolderRelease()
    {
        if (!mainBodyHolder.heldBody.has_value()) return;

        const BodyIndex bodyIndex = mainBodyHolder.heldBody.value();
        if (bodyIndex >= bodies.getCount()) return;

        const Vec2 newBodyVelocity = mainBodyHolder.getVelocity();

        bodies.velocityX[bodyIndex] = newBodyVelocity.x;
        bodies.velocityY[bodyIndex] = newBodyVelocity.y;

        mainBodyHolder.heldBody = std::nullopt;
    }

    void Simulation::mainBodyHolderIncreaseAngularVelocity(Real radiansSpeedUp)
    {
        if (!mainBodyHolder.heldBody.has_value()) return;

        const BodyIndex bodyIndex = mainBodyHolder.heldBody.value();
        if (bodyIndex >= bodies.getCount()) return;

        bodies.angularVelocity[bodyIndex] += radiansSpeedUp;
    }

    void Simulation::runBroadPhaseBenchmark(uint32_t minBodies, uint32_t maxBodies, uint32_t step, uint32_t sampleCount)
    {
        std::filesystem::path dirPath = "output/Benchmarks";
        std::filesystem::create_directories(dirPath);

        std::filesystem::path filePath = dirPath / "broad_phase_benchmark.csv";
        std::ofstream outFile(filePath);

        if (!outFile.is_open())
        {
            std::cerr << "[AGONY][Simulation::runBroadPhaseBenchmark]: Failed to create or open file: " << filePath << "\n";
            return;
        }

        // CSV header for plotting mean + percentiles cleanly.
        outFile << "BodyCount,Density,Threading,Mean_us,Median_us,P90_us,P99_us,Min_us,Max_us,StdDev_us\n";

        auto runTestConfig = [&](uint32_t count, BenchmarkDensity density, bool useThreading)
            {
                // Reset state.
                while (bodies.getCount() > 0)
                {
                    destroyBody(0);
                }

                // Create bodies.
                constexpr Real ballRadius = 1.0;

                Real choosenBodyOffset;
                std::string densityString;
                switch (density)
                {
                case BenchmarkDensity::NoTouching:
                default:
                    choosenBodyOffset = ballRadius * 2.0;
                    densityString = "NoTouching";
                    break;
                case BenchmarkDensity::Touching:
                    choosenBodyOffset = ballRadius * 2.0 * (0.95);
                    densityString = "Touching";
                    break;
                case BenchmarkDensity::AllTouching:
                    choosenBodyOffset = 0.0;
                    densityString = "AllTouching";
                    break;
                }

                uint32_t gridSide = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<float>(count))));
                for (uint32_t i = 0; i < count; i++)
                {
                    Vec2 position{ 0.0f, 0.0f };
                    Real x = i % gridSide;
                    Real y = i / gridSide;
                    position = Vec2(x, y) * choosenBodyOffset;

                    createCircle({
                        .base.position = position,
                        .base.mass = 1,
                        .base.materialIndex = 0,
                        .radius = ballRadius
                        });
                }

                // Prepare.
                broadPhaseCollisionDetector.setDataViewers(AABBSoAViewer(bodies.aabb));
                computeWorldCenters();
                buildBodyAABBs();

                // Warm-up run.
                const auto executionPolicy = useThreading ?
                    BroadPhaseCollisionDetector::ExecutionPolicy::ForceMultiThreaded :
                    BroadPhaseCollisionDetector::ExecutionPolicy::ForceSingleThreaded;
                broadPhaseCollisionDetector.findCollisions(true, executionPolicy);

                // Measure individual samples.
                std::vector<double> sampleTimesUs;
                sampleTimesUs.reserve(sampleCount);

                for (uint32_t s = 0; s < sampleCount; s++)
                {
                    const auto startTime = std::chrono::steady_clock::now();
                    volatile const auto& pairs = broadPhaseCollisionDetector.findCollisions(false, executionPolicy);
                    (void)pairs;
                    const auto endTime = std::chrono::steady_clock::now();

                    const double durationUs =
                        static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count());

                    sampleTimesUs.push_back(durationUs);
                }

                double meanUs = 0.0;
                double medianUs = 0.0;
                double p90Us = 0.0;
                double p99Us = 0.0;
                double minUs = 0.0;
                double maxUs = 0.0;
                double stdDevUs = 0.0;

                computeStats(sampleTimesUs, meanUs, medianUs, p90Us, p99Us, minUs, maxUs, stdDevUs);

                outFile << count << ","
                    << densityString << ","
                    << (useThreading ? "Threaded" : "Single") << ","
                    << meanUs << ","
                    << medianUs << ","
                    << p90Us << ","
                    << p99Us << ","
                    << minUs << ","
                    << maxUs << ","
                    << stdDevUs << "\n";
            };

        size_t totalTests = 0;
        for (uint32_t count = minBodies; count <= maxBodies; count += step)
        {
            totalTests += size_t(BenchmarkDensity::COUNT) * 2;
        }

        size_t completedTests = 0;
        for (bool useThreading : { false, true })
        {
            for (size_t densityIndex = 0; densityIndex < size_t(BenchmarkDensity::COUNT); densityIndex++)
            {
                const auto density = BenchmarkDensity(densityIndex);
                for (uint32_t count = minBodies; count <= maxBodies; count += step)
                {
                    runTestConfig(count, density, useThreading);
                    completedTests++;

                    if ((completedTests & 15) == 0)
                    {
                        const float percent = static_cast<float>(completedTests) / static_cast<float>(totalTests) * 100.0f;

                        std::cout << "Benchmark completed: " << percent << "%\n";
                    }
                }
            }
        }

        outFile.close();
    }

    void Simulation::runNarrowPhaseBenchmark(uint32_t minBodies, uint32_t maxBodies, uint32_t step, uint32_t sampleCount)
    {
        std::filesystem::path dirPath = "output/Benchmarks";
        std::filesystem::create_directories(dirPath);

        std::filesystem::path filePath = dirPath / "narrow_phase_benchmark.csv";
        std::ofstream outFile(filePath);

        if (!outFile.is_open())
        {
            std::cerr << "[AGONY][Simulation::runNarrowPhaseBenchmark]: Failed to create or open file: " << filePath << "\n";
            return;
        }

        // CSV header for plotting mean + percentiles cleanly.
        outFile << "BodyCount,Density,Threading,Mean_us,Median_us,P90_us,P99_us,Min_us,Max_us,StdDev_us\n";

        auto runTestConfig = [&](uint32_t count, BenchmarkDensity density, bool useThreading)
            {
                // Reset state.
                while (bodies.getCount() > 0)
                {
                    destroyBody(0);
                }

                // Create bodies.
                constexpr Real ballRadius = 1.0;

                Real choosenBodyOffset;
                std::string densityString;
                switch (density)
                {
                case BenchmarkDensity::NoTouching:
                default:
                    choosenBodyOffset = ballRadius * 2.0;
                    densityString = "NoTouching";
                    break;
                case BenchmarkDensity::Touching:
                    choosenBodyOffset = ballRadius * 2.0 * (0.95);
                    densityString = "Touching";
                    break;
                case BenchmarkDensity::AllTouching:
                    choosenBodyOffset = 0.0;
                    densityString = "AllTouching";
                    break;
                }

                uint32_t gridSide = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<float>(count))));
                for (uint32_t i = 0; i < count; i++)
                {
                    Vec2 position{ 0.0f, 0.0f };
                    Real x = i % gridSide;
                    Real y = i / gridSide;
                    position = Vec2(x, y) * choosenBodyOffset;

                    createCircle({
                        .base.position = position,
                        .base.mass = 1,
                        .base.materialIndex = 0,
                        .radius = ballRadius
                        });
                }

                // Prepare.
                const auto executionPolicy = useThreading ?
                    NarrowPhaseCollisionDetector::ExecutionPolicy::ForceMultiThreaded :
                    NarrowPhaseCollisionDetector::ExecutionPolicy::ForceSingleThreaded;

                computeWorldCenters();
                buildBodyAABBs();
                broadPhaseCollisionDetector.setDataViewers(AABBSoAViewer(bodies.aabb));
                const auto& broadCollisions = broadPhaseCollisionDetector.findCollisions(true);

                narrowPhaseCollisionDetector.setDataViewers(
                    BodySoAViewer(bodies),
                    CircleSoAViewer(circles),
                    BoxSoAViewer(boxes),
                    PolygonSoAViewer(polygons)
                );

                // Warm-up run.
                narrowPhaseCollisionDetector.findCollisions(broadCollisions, executionPolicy);

                // Measure individual samples.
                std::vector<double> sampleTimesUs;
                sampleTimesUs.reserve(sampleCount);

                for (uint32_t s = 0; s < sampleCount; s++)
                {
                    const auto startTime = std::chrono::steady_clock::now();
                    volatile const auto& collisionData = narrowPhaseCollisionDetector.findCollisions(broadCollisions, executionPolicy);
                    (void)collisionData;
                    const auto endTime = std::chrono::steady_clock::now();

                    const double durationUs =
                        static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count());

                    sampleTimesUs.push_back(durationUs);
                }

                double meanUs = 0.0;
                double medianUs = 0.0;
                double p90Us = 0.0;
                double p99Us = 0.0;
                double minUs = 0.0;
                double maxUs = 0.0;
                double stdDevUs = 0.0;

                computeStats(sampleTimesUs, meanUs, medianUs, p90Us, p99Us, minUs, maxUs, stdDevUs);

                outFile << count << ","
                    << densityString << ","
                    << (useThreading ? "Threaded" : "Single") << ","
                    << meanUs << ","
                    << medianUs << ","
                    << p90Us << ","
                    << p99Us << ","
                    << minUs << ","
                    << maxUs << ","
                    << stdDevUs << "\n";
            };

        size_t totalTests = 0;
        for (uint32_t count = minBodies; count <= maxBodies; count += step)
        {
            totalTests += size_t(BenchmarkDensity::COUNT) * 2;
        }

        size_t completedTests = 0;
        for (bool useThreading : { false, true })
        {
            for (size_t densityIndex = 0; densityIndex < size_t(BenchmarkDensity::COUNT); densityIndex++)
            {
                const auto density = BenchmarkDensity(densityIndex);
                for (uint32_t count = minBodies; count <= maxBodies; count += step)
                {
                    runTestConfig(count, density, useThreading);
                    completedTests++;

                    if ((completedTests & 15) == 0)
                    {
                        const float percent = static_cast<float>(completedTests) / static_cast<float>(totalTests) * 100.0f;

                        std::cout << "Benchmark completed: " << percent << "%\n";
                    }
                }
            }
        }

        outFile.close();
    }

    void Simulation::getBroadPhaseAABBs(std::vector<AABB>& outAABBs) const
    {
		broadPhaseCollisionDetector.fetchAABBs(outAABBs);
    }

    void Simulation::physicsStep(Real deltaTime)
    {
        TRACY_SCOPE_NC("Physics step", Ecstasy::Color::Orange);

        // Update timer.
        simulationRunTimer += deltaTime;

        // Check if any body exist.
        const size_t bodyCount = bodies.getCount();
        if (bodyCount == 0) return;

        // Check if any material exist.
        if (materials.empty()) [[unlikely]]
        {
            std::cerr << "[AGONY][Simulation]: Material count is zero, which should be impossible.\n";
            materials.emplace_back(); // Default material.
        }

        // Set data viewers.
        broadPhaseCollisionDetector.setDataViewers(
            AABBSoAViewer(bodies.aabb)
        );

        narrowPhaseCollisionDetector.setDataViewers(
            BodySoAViewer(bodies),
            CircleSoAViewer(circles),
            BoxSoAViewer(boxes),
            PolygonSoAViewer(polygons)
        );

        solver.setDataViewers(
            bodies,
            materials
        );

        // Remap persistent contact data if body was deleted.
        narrowPhaseCollisionDetector.remapPersistentContactData(deletedBodies);
        deletedBodies.clear();

        // Main stuff.
        integrateVelocities(bodyCount, deltaTime);
        applyBodyHolderConstraint();
        integratePositions(bodyCount, deltaTime);
        wrapRotation();
        computeRotationCosSin();

        // Compute true position for all bodies.
        computeWorldCenters();

        // Rebuild AABBs.
        buildBodyAABBs();

        if (bodyCount >= 2)
        {
            // Broad phase.
            // TODO: Refit instead of rebuilding each time.
            const std::vector<BodyPair>& broadCollisionData = broadPhaseCollisionDetector.findCollisions(true);
            if (broadCollisionData.empty()) return;

            // Narrow phase.
            const std::vector<BodyCollisionData>& narrowCollisionData = narrowPhaseCollisionDetector.findCollisions(broadCollisionData);
            if (narrowCollisionData.empty()) return;

            // Collision resolution.
            solver.solveThreaded(
                narrowCollisionData,
                simulationSettings.velocitySolvingIterations,
                simulationSettings.positionSolvingIterations
            );

            // Updating persistent contact data.
            narrowPhaseCollisionDetector.updatePersistentContactData();
        }
        else
        {
            // Call manual reset for data that can be displayed.
        }
    }

    void Simulation::postUpdate()
    {
        // That's for renderer to have actual information.
        computeWorldCenters();
        buildBodyAABBs();
    }

    void Simulation::integrateVelocities(size_t bodyCount, Real deltaTime)
    {
        TRACY_SCOPE_NC("Integrate velocities", Ecstasy::Color::Red);

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.worldCenterY.data();
        Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();
        const Real* ECSTASY_RESTRICT invMassPtr = bodies.invMass.data();

        const Vec2 gravityDelta = simulationSettings.gravity * deltaTime;
        const RealSimd gravityDeltaXV{ gravityDelta.x };
        const RealSimd gravityDeltaYV{ gravityDelta.y };

        const RealSimd zeros = RealSimd(Real(0));

        size_t i = 0;
        if constexpr (true)
        {
            for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            {
                RealSimd velX = RealSimd::load(velocityXPtr + i);
                RealSimd velY = RealSimd::load(velocityYPtr + i);

                const RealSimd invMassV = RealSimd::load(invMassPtr + i);
                const auto movableMask = invMassV != zeros;

                RealSimd newVelX = velX + gravityDeltaXV;
                RealSimd newVelY = velY + gravityDeltaYV;

                velX = RealSimd::blendv(velX, newVelX, movableMask);
                velY = RealSimd::blendv(velY, newVelY, movableMask);

                velX.store(velocityXPtr + i);
                velY.store(velocityYPtr + i);
            }
            for (; i < bodyCount; i++)
            {
                const Real invMass = invMassPtr[i];
                const Real movableMask = invMass != Real(0.0);

                velocityXPtr[i] += gravityDelta.x * movableMask;
                velocityYPtr[i] += gravityDelta.y * movableMask;
            }
        }
        else
        {
            constexpr Real PLANET_RADIUS = 10;
            constexpr Real PLANET_RADIUS_SQUARED = PLANET_RADIUS * PLANET_RADIUS;
            const Real G = 1000;

            const RealSimd gV(G);
            const RealSimd planetRadiusV(PLANET_RADIUS);
            const RealSimd planetRadiusSquaredV(PLANET_RADIUS_SQUARED);

            size_t i = 0;
            //for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            //{
            //    RealSimd velX = RealSimd::load(velocityXPtr + i);
            //    RealSimd velY = RealSimd::load(velocityYPtr + i);
            //    const RealSimd invMassV = RealSimd::load(invMassPtr + i);
            //    const auto movableMask = invMassV != zeros;
            //
            //    RealSimd posX = RealSimd::load(positionXPtr + i);
            //    RealSimd posY = RealSimd::load(positionYPtr + i);
            //
            //    RealSimd r2 = posX * posX + posY * posY + softSqV;
            //
            //    RealSimd accX = -Gv * posX / r2;
            //    RealSimd accY = -Gv * posY / r2;
            //
            //    RealSimd deltaVX = accX * RealSimd(deltaTime);
            //    RealSimd deltaVY = accY * RealSimd(deltaTime);
            //
            //    RealSimd newVelX = velX + deltaVX;
            //    RealSimd newVelY = velY + deltaVY;
            //    velX = RealSimd::blendv(velX, newVelX, movableMask);
            //    velY = RealSimd::blendv(velY, newVelY, movableMask);
            //
            //    velX.store(velocityXPtr + i);
            //    velY.store(velocityYPtr + i);
            //}
            for (; i < bodyCount; i++)
            {
                const Real invMass = invMassPtr[i];
                if (invMass == Real(0.0)) continue;

                const Real posX = positionXPtr[i];
                const Real posY = positionYPtr[i];
                const Real distanceSquared = posX * posX + posY * posY;

                Real accX, accY;
                if (distanceSquared < PLANET_RADIUS_SQUARED)
                {
                    accX = posX / PLANET_RADIUS;
                    accY = posY / PLANET_RADIUS;
                }
                else
                {
                    const Real distance = std::sqrt(distanceSquared);

                    const Real normalX = posX / distance;
                    const Real normalY = posY / distance;

                    const Real radiusRatioSquared = PLANET_RADIUS_SQUARED / distanceSquared;

                    accX = normalX * radiusRatioSquared;
                    accY = normalY * radiusRatioSquared;
                }
                accX *= -G;
                accY *= -G;

                velocityXPtr[i] += accX * deltaTime;
                velocityYPtr[i] += accY * deltaTime;
            }
        }
    }

    void Simulation::integratePositions(size_t bodyCount, Real deltaTime)
    {
        TRACY_SCOPE_NC("Intergrate positions", Ecstasy::Color::Blue);

        const RealSimd deltaTimeV{ deltaTime };

        // Position and rotatiob.
        {
            Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX.data();
            Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY.data();
            Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation.data();

            const Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
            const Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();
            const Real* ECSTASY_RESTRICT angularVelocityPtr = bodies.angularVelocity.data();

            size_t i = 0;
            for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
            {
                const RealSimd velX   = RealSimd::load(velocityXPtr + i);
                const RealSimd velY   = RealSimd::load(velocityYPtr + i);
                const RealSimd angVel = RealSimd::load(angularVelocityPtr + i);

                RealSimd posX = RealSimd::load(positionXPtr + i);
                RealSimd posY = RealSimd::load(positionYPtr + i);
                RealSimd rot  = RealSimd::load(rotationPtr + i);

                posX = RealSimd::mulAdd(velX,   deltaTimeV, posX);
                posY = RealSimd::mulAdd(velY,   deltaTimeV, posY);
                rot  = RealSimd::mulAdd(angVel, deltaTimeV, rot);

                posX.store(positionXPtr + i);
                posY.store(positionYPtr + i);
                rot.store(rotationPtr + i);
            }
            for (; i < bodyCount; i++)
            {
                positionXPtr[i] += velocityXPtr[i]       * deltaTime;
                positionYPtr[i] += velocityYPtr[i]       * deltaTime;
                rotationPtr[i]  += angularVelocityPtr[i] * deltaTime;
            }
        }
    }

    void Simulation::buildBodyAABBs()
    {
        TRACY_SCOPE_NC("Build body AABBs", Ecstasy::Color::Green);
        buildCircleAABBs();
        buildBoxAABBs();
        buildPolygonAABBs();
    }

    void Simulation::buildCircleAABBs()
    {
        const size_t count = circles.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build circle AABBs", Ecstasy::Color::DarkGreen);

        using IndexSimd = std::conditional_t<
            std::is_same_v<Real, float>,
            Ecstasy::Simd<int32_t, 256>,
            Ecstasy::Simd<int32_t, 128>
        >;

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.worldCenterY.data();

        const BodyIndex* ECSTASY_RESTRICT bodyIndexPtr = circles.bodyIndices.data();
        const Real* ECSTASY_RESTRICT radiusPtr = circles.radius.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = bodies.aabb.minX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = bodies.aabb.minY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = bodies.aabb.maxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = bodies.aabb.maxY.data();

        alignas(IndexSimd::bytes) BodyIndex bodyIndexBatch[IndexSimd::lanes];
        alignas(RealSimd::bytes) Real xBatch[RealSimd::lanes];
        alignas(RealSimd::bytes) Real yBatch[RealSimd::lanes];

        alignas(RealSimd::bytes) Real minXBatch[RealSimd::lanes];
        alignas(RealSimd::bytes) Real minYBatch[RealSimd::lanes];
        alignas(RealSimd::bytes) Real maxXBatch[RealSimd::lanes];
        alignas(RealSimd::bytes) Real maxYBatch[RealSimd::lanes];

        size_t i = 0;
        for (; i + RealSimd::lanes <= count; i += RealSimd::lanes)
        {
            std::memcpy(bodyIndexBatch, bodyIndexPtr + i, IndexSimd::bytes);

            RealSimd x;
            RealSimd y;

            if constexpr (RealSimd::isGatherAvailable())
            {
                const auto indices = IndexSimd::load((const int32_t*)bodyIndexBatch);
                x = RealSimd::gather(positionXPtr, indices);
                y = RealSimd::gather(positionYPtr, indices);
            }
            else
            {
                for (size_t j = 0; j < RealSimd::lanes; j++)
                {
                    const BodyIndex bodyIndex = bodyIndexBatch[j];
                    xBatch[j] = positionXPtr[bodyIndex];
                    yBatch[j] = positionYPtr[bodyIndex];
                }
                x = RealSimd::load(xBatch);
                y = RealSimd::load(yBatch);
            }

            const RealSimd radius = RealSimd::load(radiusPtr + i);

            (x - radius).store(minXBatch);
            (y - radius).store(minYBatch);
            (x + radius).store(maxXBatch);
            (y + radius).store(maxYBatch);

            for (size_t j = 0; j < RealSimd::lanes; j++)
            {
                const BodyIndex bodyIndex = bodyIndexBatch[j];
            
                aabbMinXPtr[bodyIndex] = minXBatch[j];
                aabbMinYPtr[bodyIndex] = minYBatch[j];
                aabbMaxXPtr[bodyIndex] = maxXBatch[j];
                aabbMaxYPtr[bodyIndex] = maxYBatch[j];
            }
        }
        for (; i < count; i++)
        {
            const BodyIndex bodyIndex = bodyIndexPtr[i];
            const Real radius = radiusPtr[i];

            const Real x = positionXPtr[bodyIndex];
            const Real y = positionYPtr[bodyIndex];

            aabbMinXPtr[bodyIndex] = x - radius;
            aabbMinYPtr[bodyIndex] = y - radius;
            aabbMaxXPtr[bodyIndex] = x + radius;
            aabbMaxYPtr[bodyIndex] = y + radius;
        }
    }

    void Simulation::buildBoxAABBs()
    {
        const size_t count = boxes.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build box AABBs", Ecstasy::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.worldCenterY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin.data();

        const BodyIndex* ECSTASY_RESTRICT bodyIndexPtr = boxes.bodyIndices.data();
        const Real* ECSTASY_RESTRICT halfWidthPtr = boxes.halfWidth.data();
        const Real* ECSTASY_RESTRICT halfHeightPtr = boxes.halfHeight.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = bodies.aabb.minX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = bodies.aabb.minY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = bodies.aabb.maxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = bodies.aabb.maxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const BodyIndex bodyIndex = bodyIndexPtr[i];
            const Real halfWidth = halfWidthPtr[i];
            const Real halfHeight = halfHeightPtr[i];

            const Real x = positionXPtr[bodyIndex];
            const Real y = positionYPtr[bodyIndex];
            const Real cos = rotationCosPtr[bodyIndex];
            const Real sin = rotationSinPtr[bodyIndex];

            const Real absCos = std::fabs(cos);
            const Real absSin = std::fabs(sin);

            const Real ex = absCos * halfWidth + absSin * halfHeight;
            const Real ey = absSin * halfWidth + absCos * halfHeight;

            aabbMinXPtr[bodyIndex] = x - ex;
            aabbMinYPtr[bodyIndex] = y - ey;
            aabbMaxXPtr[bodyIndex] = x + ex;
            aabbMaxYPtr[bodyIndex] = y + ey;
        }
    }

    void Simulation::buildPolygonAABBs()
    {
        const size_t count = polygons.getCount();
        if (count == 0) return;

        TRACY_SCOPE_NC("Build polygon AABBs", Ecstasy::Color::DarkGreen);

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.worldCenterX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.worldCenterY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin.data();
        const BodyIndex* ECSTASY_RESTRICT bodyIndexPtr = polygons.bodyIndices.data();
        const VerticesContainer* ECSTASY_RESTRICT localVertsPtr = polygons.localVertices.data();

        Real* ECSTASY_RESTRICT aabbMinXPtr = bodies.aabb.minX.data();
        Real* ECSTASY_RESTRICT aabbMinYPtr = bodies.aabb.minY.data();
        Real* ECSTASY_RESTRICT aabbMaxXPtr = bodies.aabb.maxX.data();
        Real* ECSTASY_RESTRICT aabbMaxYPtr = bodies.aabb.maxY.data();

        for (size_t i = 0; i < count; i++)
        {
            const BodyIndex bodyIndex = bodyIndexPtr[i];
            const Real x = positionXPtr[bodyIndex];
            const Real y = positionYPtr[bodyIndex];
            const Real cos = rotationCosPtr[bodyIndex];
            const Real sin = rotationSinPtr[bodyIndex];

            const Vec2* verticesPtr = localVertsPtr[i].data();
            const size_t vertexCount = localVertsPtr[i].size();

            Real minX =  std::numeric_limits<Real>::max();
            Real minY =  std::numeric_limits<Real>::max();
            Real maxX = -std::numeric_limits<Real>::max();
            Real maxY = -std::numeric_limits<Real>::max();

            for (size_t v = 0; v < vertexCount; v++)
            {
                const Vec2 vertex = verticesPtr[v];
                const Real wx = cos * vertex.x - sin * vertex.y;
                const Real wy = sin * vertex.x + cos * vertex.y;
                minX = std::fmin(minX, wx);
                maxX = std::fmax(maxX, wx);
                minY = std::fmin(minY, wy);
                maxY = std::fmax(maxY, wy);
            }

            aabbMinXPtr[bodyIndex] = x + minX;
            aabbMinYPtr[bodyIndex] = y + minY;
            aabbMaxXPtr[bodyIndex] = x + maxX;
            aabbMaxYPtr[bodyIndex] = y + maxY;
        }
    }

    void Simulation::wrapRotation()
    {
        

        constexpr size_t LANES = RealSimd::lanes;

        TRACY_SCOPE_NC("Wrap rotation", Ecstasy::Color::Cyan);

        Real* ECSTASY_RESTRICT rotationPtr = bodies.rotation.data();

        const size_t bodyCount = bodies.getCount();

        const RealSimd twoPIV(Constants::TWO_PI);
        const RealSimd invTwoPIV(Real(1) / Constants::TWO_PI);

        size_t i = 0;
        for (; i + LANES <= bodyCount; i += LANES)
        {
            RealSimd rot = RealSimd::load(rotationPtr + i);

            RealSimd q = RealSimd::roundTowardsZero(rot * invTwoPIV);
            rot = rot - q * twoPIV;

            RealSimd isRotNegativeMask = rot < RealSimd(0);
            rot += isRotNegativeMask & twoPIV;

            rot.store(rotationPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            float rot = rotationPtr[i];
            rot = std::fmod(rot, Constants::TWO_PI);
            rot += (rot < 0) * Constants::TWO_PI;
            rotationPtr[i] = rot;
        }
    }

    void Simulation::computeRotationCosSin()
    {
        TRACY_SCOPE_NC("Compute rotation cos/sin", Ecstasy::Color::Teal);

        FastCosSin::order4CosSin(
            bodies.rotation.data(),
            bodies.rotationCos.data(),
            bodies.rotationSin.data(),
            bodies.getCount()
        );

        //for (size_t i = 0; i < bodies.getCount(); i++)
        //{
        //    const Real angle = bodies.rotation[i];
        //    bodies.rotationCos[i] = std::cos(angle);
        //    bodies.rotationSin[i] = std::sin(angle);
        //}
    }

    void Simulation::computeWorldCenters()
    {
        

        TRACY_SCOPE_NC("Compute true positions", Ecstasy::Color::Magenta);

        const Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX.data();
        const Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassXPtr = bodies.localCenterOfMassX.data();
        const Real* ECSTASY_RESTRICT localCenterOfMassYPtr = bodies.localCenterOfMassY.data();
        const Real* ECSTASY_RESTRICT rotationCosPtr = bodies.rotationCos.data();
        const Real* ECSTASY_RESTRICT rotationSinPtr = bodies.rotationSin.data();

        Real* ECSTASY_RESTRICT worldCenterXPtr = bodies.worldCenterX.data();
        Real* ECSTASY_RESTRICT worldCenterYPtr = bodies.worldCenterY.data();

        const size_t bodyCount = bodies.getCount();

        size_t i = 0;
        for (; i + RealSimd::lanes <= bodyCount; i += RealSimd::lanes)
        {
            const RealSimd positionX = RealSimd::load(positionXPtr + i);
            const RealSimd positionY = RealSimd::load(positionYPtr + i);

            const RealSimd localCenterOfMassX = RealSimd::load(localCenterOfMassXPtr + i);
            const RealSimd localCenterOfMassY = RealSimd::load(localCenterOfMassYPtr + i);

            const RealSimd cosRot = RealSimd::load(rotationCosPtr + i);
            const RealSimd sinRot = RealSimd::load(rotationSinPtr + i);

            const RealSimd truePositionX = RealSimd::negMulAdd(localCenterOfMassX, cosRot,     RealSimd::mulAdd(localCenterOfMassY, sinRot, positionX + localCenterOfMassX));
            const RealSimd truePositionY = RealSimd::negMulAdd(localCenterOfMassX, sinRot, RealSimd::negMulAdd(localCenterOfMassY, cosRot, positionY + localCenterOfMassY));
        
            truePositionX.store(worldCenterXPtr + i);
            truePositionY.store(worldCenterYPtr + i);
        }
        for (; i < bodyCount; i++)
        {
            const Real positionX = positionXPtr[i];
            const Real positionY = positionYPtr[i];
            const Real localCenterOfMassX = localCenterOfMassXPtr[i];
            const Real localCenterOfMassY = localCenterOfMassYPtr[i];
            const Real cosRot = rotationCosPtr[i];
            const Real sinRot = rotationSinPtr[i];
            worldCenterXPtr[i] = (positionX + localCenterOfMassX) - (localCenterOfMassX * cosRot - localCenterOfMassY * sinRot);
            worldCenterYPtr[i] = (positionY + localCenterOfMassY) - (localCenterOfMassX * sinRot + localCenterOfMassY * cosRot);
        }
    }

    void Simulation::applyBodyHolderConstraint()
    {
        if (!mainBodyHolder.heldBody.has_value()) return;

        const BodyIndex bodyIndex = mainBodyHolder.heldBody.value();
        if (bodyIndex >= bodies.getCount()) return;

        const Vec2 newBodyPosition = mainBodyHolder.getPosition() + mainBodyHolder.bodyOffset;
        const Vec2 newBodyVelocity = mainBodyHolder.getVelocity();

        Real* ECSTASY_RESTRICT positionXPtr = bodies.offsetX.data();
        Real* ECSTASY_RESTRICT positionYPtr = bodies.offsetY.data();
        Real* ECSTASY_RESTRICT velocityXPtr = bodies.velocityX.data();
        Real* ECSTASY_RESTRICT velocityYPtr = bodies.velocityY.data();

        positionXPtr[bodyIndex] = newBodyPosition.x;
        positionYPtr[bodyIndex] = newBodyPosition.y;

        velocityXPtr[bodyIndex] = newBodyVelocity.x;
        velocityYPtr[bodyIndex] = newBodyVelocity.y;
    }

    void Simulation::collectMemoryUsage(DebugData& data) const
    {
        // Memory
        data.bodyDataMemoryUsage = sizeof(BodySoA) + bodies.getMemoryUsage();
        data.circleDataMemoryUsage = sizeof(CircleSoA) + circles.getMemoryUsage();
        data.boxDataMemoryUsage = sizeof(BoxSoA) + boxes.getMemoryUsage();
        data.polygonDataMemoryUsage = sizeof(PolygonSoA) + polygons.getMemoryUsage();

        data.materialDataMemoryUsage = materials.capacity() * sizeof(materials[0]);
        data.broadPhaseDetectorMemoryUsage = broadPhaseCollisionDetector.getMemoryUsage();
        data.narrowPhaseDetectorMemoryUsage = narrowPhaseCollisionDetector.getMemoryUsage();
        data.solverMemoryUsage = solver.getMemoryUsage();
    }
}