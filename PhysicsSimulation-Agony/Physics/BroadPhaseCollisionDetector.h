#pragma once
#include "BodySoAViewer.h"
#include "Threading.h"

#include "EcstasyCore/TracyProfiler.h"

#include <vector>
#include <mutex>
#include <condition_variable>
#include <array>

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
				mutable TracyLockableN(std::mutex, mutex, "Worker mutex");
				std::condition_variable_any cv;

				bool stopRequested = false;
				bool finished = true;
				bool running = false;

				std::vector<uint32_t> incomingSelfTasks;
				std::vector<BvhNodePair> incomingCrossTasks;

				std::vector<uint32_t> localSelfTasks;
				std::vector<BvhNodePair> localCrossTasks;

				std::vector<BodyPair> outCollisionData;

				void pushSelfTasks(const uint32_t* taskSource, size_t taskCount)
				{
					bool needNotify = false;
					{
						TRACY_SCOPE_N("Push");
						std::lock_guard lock(mutex);
						incomingSelfTasks.insert(
							incomingSelfTasks.end(),
							taskSource, taskSource + taskCount
						);
						needNotify = !running;
					}
					{
						TRACY_SCOPE_N("Notify");
						if (needNotify)
						{
							cv.notify_one();
						}
					}
				}

				void pushCrossTasks(const BvhNodePair* taskSource, size_t taskCount)
				{
					bool needNotify = false;
					{
						TRACY_SCOPE_N("Push");
						std::lock_guard lock(mutex);
						incomingCrossTasks.insert(
							incomingCrossTasks.end(),
							taskSource, taskSource + taskCount
						);
						needNotify = !running;
					}
					{
						TRACY_SCOPE_N("Notify");
						if (needNotify)
						{
							cv.notify_one();
						}
					}
				}
			};

			static constexpr size_t WORKER_COUNT = std::min(4ull, size_t(Threading::MAX_THREADS_ALLOWED));

			std::array<WorkerData, WORKER_COUNT> workerData;
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


		static constexpr bool USE_THREADING = Threading::MAX_THREADS_ALLOWED > 0;

		AABBSoAViewer bodiesAABB;
		BvhFunctionResources bvhFunctionResources;
		QueryPairsThreadedResources queryPairsThreadedResources;

		LeafBodyAABBSoA leafBodyAABBs;

		std::vector<BodyPair> collisionData;
	public:
		BroadPhaseCollisionDetector() = default;
		~BroadPhaseCollisionDetector() = default;
		BroadPhaseCollisionDetector(const BroadPhaseCollisionDetector&) = default;
		BroadPhaseCollisionDetector& operator=(const BroadPhaseCollisionDetector&) = default;
		BroadPhaseCollisionDetector(BroadPhaseCollisionDetector&&) = default;
		BroadPhaseCollisionDetector& operator=(BroadPhaseCollisionDetector&&) = default;

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

