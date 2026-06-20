#pragma once
#include "BodySoAViewer.h"

#include "EcstasyCore/TracyProfiler.h"

#include <vector>
#include <mutex>
#include <condition_variable>

namespace PS_AGONY
{
	class BroadPhaseCollisionDetector
	{
		using MortonCode = uint32_t;

		// Note: Splitting on cold and hot didn't help.
		struct BvhNode
		{
			// Max KD_LEAF_SIZE is 32. Larger size will fuck up bitwise mask.
			// (We can change mask to me uint64_t to allow max KD_LEAF_SIZE to be 64, but increasing KD_LEAF_SIZE leads to perfomance decrease in narrow phase.)
			static constexpr uint32_t KD_LEAF_SIZE = 16;

			static constexpr uint32_t INVALID_INDEX = -1;

			Real minX, maxX, minY, maxY; // Merged AABB of all bodies in this subtree.
			uint32_t leftChildIndex = INVALID_INDEX; // INVALID_INDEX for leaves.
			// rightChildIndex = leftChildIndex + 1.
			uint32_t start, end; // Range in kdIndices: [start, end).
			uint32_t leafIndex; // If node is a leaf, it's its index.
		
			BvhNode() :
				start(0), end(0)
			{}

			BvhNode(uint32_t start, uint32_t end) :
				start(start), end(end)
			{}
		};

		struct BvhNodePair
		{
			uint32_t a, b;

			BvhNodePair swap() const noexcept { return { b, a }; }
		};

		struct alignas(64) PairVector
		{
			std::vector<BodyPair> pairs;
		};

		struct BvhFunctionResources
		{
			std::vector<BvhNode> nodes;

			SimdAlignedVector<Real> transformedCentroidX;
			SimdAlignedVector<Real> transformedCentroidY;

			SimdAlignedVector<MortonCode> mortonCodes;

			std::vector<BodyIndex> mainBodyIndices;
			std::vector<BodyIndex> tempBodyIndicesToSort;

			std::vector<BvhNodePair> nodePairsToTraverse;
			std::vector<BvhNodePair> leafPairsToTestCollisions;
			std::vector<uint32_t> leavesToTestCollisions;
		};

		struct QueryPairsThreadedResources
		{
			struct alignas(64) WorkerData
			{
				//mutable TracyLockableN(std::mutex, mutex, "Worker mutex");
				mutable std::mutex mutex;
				std::condition_variable_any cv;

				bool stopRequested = false;
				bool finished = true;
				bool running = false;

				std::vector<uint32_t> incomingSelfTasks;
				std::vector<BvhNodePair> incomingCrossTasks;

				std::vector<uint32_t> localSelfTasks;
				std::vector<BvhNodePair> localCrossTasks;

				std::vector<BodyPair> outCollisionData;


				WorkerData() = default;
				~WorkerData() = default;
				WorkerData(const WorkerData&) = delete;
				WorkerData& operator=(const WorkerData&) = delete;

				WorkerData(WorkerData&& other) noexcept
				{
					stopRequested = std::exchange(stopRequested, true);
					finished = std::exchange(finished, true);
					running = std::exchange(running, false);

					incomingSelfTasks  = std::move(other.incomingSelfTasks);
					incomingCrossTasks = std::move(other.incomingCrossTasks);
					localSelfTasks	   = std::move(other.localSelfTasks);
					localCrossTasks    = std::move(other.localCrossTasks);
					outCollisionData   = std::move(other.outCollisionData);
				}

				WorkerData& operator=(WorkerData&& other) noexcept
				{
					if (this != &other)
					{
						stopRequested = std::exchange(stopRequested, true);
						finished = std::exchange(finished, true);
						running = std::exchange(running, false);

						incomingSelfTasks = std::move(other.incomingSelfTasks);
						incomingCrossTasks = std::move(other.incomingCrossTasks);
						localSelfTasks = std::move(other.localSelfTasks);
						localCrossTasks = std::move(other.localCrossTasks);
						outCollisionData = std::move(other.outCollisionData);
					}
					return *this;
				}


				void pushSelfTasks(const uint32_t* taskSource, size_t taskCount)
				{
					bool needNotify = false;
					{
						std::lock_guard lock(mutex);
						incomingSelfTasks.insert(
							incomingSelfTasks.end(),
							taskSource, taskSource + taskCount
						);
						needNotify = !running;
					}
					if (needNotify)
					{
						cv.notify_one();
					}
				}

				void pushCrossTasks(const BvhNodePair* taskSource, size_t taskCount)
				{
					bool needNotify = false;
					{
						std::lock_guard lock(mutex);
						incomingCrossTasks.insert(
							incomingCrossTasks.end(),
							taskSource, taskSource + taskCount
						);
						needNotify = !running;
					}
					if (needNotify)
					{
						cv.notify_one();
					}
				}
			};

			std::vector<WorkerData> workerData;
			size_t workerCount = 0;
		};

		struct LeafBodyAABBSoA
		{
			struct LeafData
			{
				Real data[BvhNode::KD_LEAF_SIZE];
			};

			RealSimdAlignedVector<LeafData> minX;
			RealSimdAlignedVector<LeafData> maxX;
			RealSimdAlignedVector<LeafData> minY;
			RealSimdAlignedVector<LeafData> maxY;
		};

		AABBSoAViewer bodiesAABB;
		BvhFunctionResources bvhFunctionResources;
		QueryPairsThreadedResources queryPairsThreadedResources;

		LeafBodyAABBSoA leafBodyAABBs;

		std::vector<BodyPair> collisionData;
	public:
		BroadPhaseCollisionDetector() = default;
		~BroadPhaseCollisionDetector() = default;
		BroadPhaseCollisionDetector(const BroadPhaseCollisionDetector&) = delete;
		BroadPhaseCollisionDetector& operator=(const BroadPhaseCollisionDetector&) = delete;
		BroadPhaseCollisionDetector(BroadPhaseCollisionDetector&&) = delete;
		BroadPhaseCollisionDetector& operator=(BroadPhaseCollisionDetector&&) = delete;

		void setDataViewers(
			const AABBSoAViewer& aabbs
		);

		const std::vector<BodyPair>& findCollisions(bool rebuild);

		void fetchAABBs(std::vector<AABB>& outAABBs) const;

		size_t getMemoryUsage() const;
	private:
		void computeCentroidsWithTransformations(uint32_t bodyCount, Vec2 globalMin, Vec2 scale, Real clampMax);

		template<std::floating_point TReal>
		void computeMortonCodes(uint32_t bodyCount);

		void sortBodyIndicesByMortonCodes(uint32_t bodyCount);

		void buildBvhTree(const uint32_t bodyCount);

		void refitBvhNodeAABBS();

		void queryBvhPairs();
		void queryBvhPairsThreaded();

		void traverseNodesToGetOverlappingLeafPairs();
		void traverseNodesToGetOverlappingLeafPairsThreaded();

		void testCollisionsInLeaves();
	};
}

