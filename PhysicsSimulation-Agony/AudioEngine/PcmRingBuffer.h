#pragma once
#include "miniaudio/miniaudio.h"
#include <cstddef>
#include <cstring>

namespace AudioEngine
{
    class PcmRingBuffer final
    {
        ma_pcm_rb rb{};
        bool initialized = false;
        bool deinterleavedValue = false;
        ma_uint32 frameCapacityValue = 0;
        ma_uint32 subbufferSizeValue = 0;
        ma_uint32 subbufferCountValue = 0;
        ma_uint32 subbufferStrideValue = 0;
    public:
        struct Block
        {
            void* data = nullptr;
            ma_uint32 frames = 0;

            // Returns true when the block contains a valid contiguous region.
            explicit operator bool() const noexcept
            {
                return data != nullptr && frames != 0;
            }

            // Clears the block to an empty state.
            void clear() noexcept
            {
                data = nullptr;
                frames = 0;
            }
        };

        // Creates an empty wrapper with no initialized ring buffer.
        PcmRingBuffer() noexcept = default;

        // Initializes an interleaved PCM ring buffer.
        PcmRingBuffer(ma_format format,
            ma_uint32 channels,
            ma_uint32 bufferFrames,
            void* optionalPreallocatedBuffer = nullptr,
            const ma_allocation_callbacks* allocationCallbacks = nullptr);

        // Moves ring buffer ownership into this object.
        PcmRingBuffer(PcmRingBuffer&& other) noexcept;

        // Transfers ring buffer ownership into this object.
        PcmRingBuffer& operator=(PcmRingBuffer&& other) noexcept;

        // Copying is disabled because the underlying ring buffer owns state.
        PcmRingBuffer(const PcmRingBuffer&) = delete;

        // Copy assignment is disabled because the underlying ring buffer owns state.
        PcmRingBuffer& operator=(const PcmRingBuffer&) = delete;

        // Releases the ring buffer when the wrapper goes out of scope.
        ~PcmRingBuffer() noexcept;

        // Initializes an interleaved PCM ring buffer.
        ma_result initialize(ma_format format,
            ma_uint32 channels,
            ma_uint32 bufferFrames,
            void* optionalPreallocatedBuffer = nullptr,
            const ma_allocation_callbacks* allocationCallbacks = nullptr) noexcept;

        // Initializes a deinterleaved PCM ring buffer.
        ma_result initializeDeinterleaved(ma_format format,
            ma_uint32 channels,
            ma_uint32 subbufferFrames,
            ma_uint32 subbufferCount,
            ma_uint32 subbufferStrideFrames = 0,
            void* optionalPreallocatedBuffer = nullptr,
            const ma_allocation_callbacks* allocationCallbacks = nullptr) noexcept;

        // Releases the ring buffer and resets the wrapper to an empty state.
        void shutdown() noexcept;

        // Resets the read and write cursors to their initial positions.
        void reset() noexcept
        {
            if (initialized)
            {
                ma_pcm_rb_reset(&rb);
            }
        }

        // Returns true when the wrapper owns an initialized ring buffer.
        bool isInitialized() const noexcept
        {
            return initialized;
        }

        // Returns the PCM sample format used by the ring buffer.
        ma_format format() const noexcept
        {
            return rb.format;
        }

        // Returns the number of channels used by the ring buffer.
        ma_uint32 channelCount() const noexcept
        {
            return rb.channels;
        }

        // Returns the nominal frame capacity requested at initialization.
        ma_uint32 capacityFrames() const noexcept
        {
            return frameCapacityValue;
        }

        // Returns true when the ring buffer was initialized in deinterleaved mode.
        bool deinterleaved() const noexcept
        {
            return deinterleavedValue;
        }

        // Returns the subbuffer size in frames.
        ma_uint32 subbufferSizeFrames() const noexcept
        {
            return subbufferSizeValue;
        }

        // Returns the number of subbuffers in the deinterleaved layout.
        ma_uint32 subbufferCount() const noexcept
        {
            return subbufferCountValue;
        }

        // Returns the spacing, in frames, between consecutive subbuffers.
        ma_uint32 subbufferStrideFrames() const noexcept
        {
            return subbufferStrideValue;
        }

        // Returns the number of frames currently available for reading.
        ma_uint32 availableRead() const noexcept;

        // Returns the number of frames currently available for writing.
        ma_uint32 availableWrite() const noexcept;

        // Returns the signed frame distance between the read and write cursors.
        ma_int32 pointerDistance() const noexcept;

        // Reads PCM frames into the caller's buffer without exposing acquire/commit details.
        ma_uint32 read(void* destination, ma_uint32 requestedFrames) noexcept;

        // Writes PCM frames from the caller's buffer without exposing acquire/commit details.
        ma_uint32 write(const void* source, ma_uint32 requestedFrames) noexcept;

        // Advances the read cursor by the specified number of frames.
        ma_result seekRead(ma_uint32 offsetFrames) noexcept;

        // Advances the write cursor by the specified number of frames.
        ma_result seekWrite(ma_uint32 offsetFrames) noexcept;

        // Returns the size of one subbuffer in frames.
        ma_uint32 subbufferSize(ma_uint32 subbufferIndex) const noexcept;

        // Returns the offset of a subbuffer in frames.
        ma_uint32 subbufferOffset(ma_uint32 subbufferIndex) const noexcept;

        // Returns the address of a subbuffer inside a caller-supplied base buffer.
        void* subbufferPtr(ma_uint32 subbufferIndex, void* baseBuffer) const noexcept;
    private:
        // Copies state from another wrapper and leaves the source empty.
        void moveFrom(PcmRingBuffer& other) noexcept;
    };
}