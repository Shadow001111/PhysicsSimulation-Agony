#pragma once
#include <utility>
#include <span>

namespace PS_AGONY2D::Threading
{
	using TaskFn = void(*)(void* context, size_t workerIndex);

	// User-implemented threading API.
	class IThreadingBackend
	{
	public:
		virtual ~IThreadingBackend() = default;

		// Number of threads (besides the calling one) that can run tasks truly concurrently.
		// Wave scheduling spin/waits on all workers at once, so this must be honest.
		virtual size_t getThreadCount() const noexcept = 0;

		// Runs fn(context, i) for i in [0, taskCount) on pool threads, all concurrently
		// (submit as one bulk operation). Must return without waiting for the tasks.
		virtual void launch(size_t taskCount, TaskFn fn, void* context, std::span<const uint32_t> targets = {}) = 0;
	};

	// Splits taskRange into chunks: about threadCount * loadBalancingFactor of them.
	// Returns { chunkCount, chunkSize }.
	inline std::pair<size_t, size_t> getChunkCountAndSize(size_t threadCount, size_t taskRange, size_t loadBalancingFactor) noexcept
	{
		const size_t maxChunkCount = threadCount * loadBalancingFactor;
		if (taskRange == 0 || maxChunkCount == 0) [[unlikely]] return { 0, 0 };

		const size_t chunkSize = (taskRange + maxChunkCount - 1) / maxChunkCount;
		const size_t chunkCount = (taskRange + chunkSize - 1) / chunkSize;
		return { chunkCount, chunkSize };
	}

	// Call once at startup, before any solver runs.
	void setThreadingBackend(IThreadingBackend* backend) noexcept;

	// Aborts (also in release) if no backend was set.
	IThreadingBackend& getThreadingBackend();
}