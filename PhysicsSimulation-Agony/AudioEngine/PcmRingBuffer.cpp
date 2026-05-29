#include "PcmRingBuffer.h"

#include <cstring>

namespace AudioEngine
{
    PcmRingBuffer::PcmRingBuffer(ma_format format,
        ma_uint32 channels,
        ma_uint32 bufferFrames,
        void* optionalPreallocatedBuffer,
        const ma_allocation_callbacks* allocationCallbacks)
    {
        initialize(format, channels, bufferFrames, optionalPreallocatedBuffer, allocationCallbacks);
    }

    PcmRingBuffer::PcmRingBuffer(PcmRingBuffer&& other) noexcept
    {
        moveFrom(other);
    }

    PcmRingBuffer& PcmRingBuffer::operator=(PcmRingBuffer&& other) noexcept
    {
        if (this != &other)
        {
            shutdown();
            moveFrom(other);
        }

        return *this;
    }

    PcmRingBuffer::~PcmRingBuffer() noexcept
    {
        shutdown();
    }

    ma_result PcmRingBuffer::initialize(ma_format format,
        ma_uint32 channels,
        ma_uint32 bufferFrames,
        void* optionalPreallocatedBuffer,
        const ma_allocation_callbacks* allocationCallbacks) noexcept
    {
        shutdown();

        const ma_result result = ma_pcm_rb_init(format,
            channels,
            bufferFrames,
            optionalPreallocatedBuffer,
            allocationCallbacks,
            &rb);

        if (result == MA_SUCCESS)
        {
            initialized = true;
            frameCapacityValue = bufferFrames;
            subbufferSizeValue = ma_pcm_rb_get_subbuffer_size(&rb);
            subbufferCountValue = 1;
            subbufferStrideValue = ma_pcm_rb_get_subbuffer_stride(&rb);
            deinterleavedValue = false;
        }

        return result;
    }

    ma_result PcmRingBuffer::initializeDeinterleaved(ma_format format,
        ma_uint32 channels,
        ma_uint32 subbufferFrames,
        ma_uint32 subbufferCount,
        ma_uint32 subbufferStrideFrames,
        void* optionalPreallocatedBuffer,
        const ma_allocation_callbacks* allocationCallbacks) noexcept
    {
        shutdown();

        const ma_result result = ma_pcm_rb_init_ex(format,
            channels,
            subbufferFrames,
            subbufferCount,
            subbufferStrideFrames,
            optionalPreallocatedBuffer,
            allocationCallbacks,
            &rb);

        if (result == MA_SUCCESS)
        {
            initialized = true;
            frameCapacityValue = subbufferFrames * subbufferCount;
            subbufferSizeValue = ma_pcm_rb_get_subbuffer_size(&rb);
            subbufferCountValue = subbufferCount;
            subbufferStrideValue = ma_pcm_rb_get_subbuffer_stride(&rb);
            deinterleavedValue = true;
        }

        return result;
    }

    void PcmRingBuffer::shutdown() noexcept
    {
        if (initialized)
        {
            ma_pcm_rb_uninit(&rb);
            rb = {};
            initialized = false;
            frameCapacityValue = 0;
            subbufferSizeValue = 0;
            subbufferCountValue = 0;
            subbufferStrideValue = 0;
            deinterleavedValue = false;
        }
    }

    ma_uint32 PcmRingBuffer::availableRead() const noexcept
    {
        return initialized ? ma_pcm_rb_available_read(const_cast<ma_pcm_rb*>(&rb)) : 0;
    }

    ma_uint32 PcmRingBuffer::availableWrite() const noexcept
    {
        return initialized ? ma_pcm_rb_available_write(const_cast<ma_pcm_rb*>(&rb)) : 0;
    }

    ma_int32 PcmRingBuffer::pointerDistance() const noexcept
    {
        return initialized ? ma_pcm_rb_pointer_distance(const_cast<ma_pcm_rb*>(&rb)) : 0;
    }
    
    ma_uint32 PcmRingBuffer::read(void* destination, ma_uint32 requestedFrames) noexcept
    {
        if (!initialized || destination == nullptr || requestedFrames == 0)
        {
            return 0;
        }

        const ma_uint32 bytesPerFrame = ma_get_bytes_per_frame(rb.format, rb.channels);
        if (bytesPerFrame == 0)
        {
            return 0;
        }

        ma_uint8* outputBytes = static_cast<ma_uint8*>(destination);
        ma_uint32 totalFramesRead = 0;

        while (totalFramesRead < requestedFrames)
        {
            ma_uint32 framesToRead = requestedFrames - totalFramesRead;
            void* pBlock = nullptr;

            if (ma_pcm_rb_acquire_read(&rb, &framesToRead, &pBlock) != MA_SUCCESS || framesToRead == 0)
            {
                break;
            }

            std::memcpy(outputBytes + (totalFramesRead * bytesPerFrame),
                pBlock,
                framesToRead * bytesPerFrame);

            if (ma_pcm_rb_commit_read(&rb, framesToRead) != MA_SUCCESS)
            {
                break;
            }

            totalFramesRead += framesToRead;
        }

        return totalFramesRead;
    }

    ma_uint32 PcmRingBuffer::write(const void* source, ma_uint32 requestedFrames) noexcept
    {
        if (!initialized || source == nullptr || requestedFrames == 0)
        {
            return 0;
        }

        const ma_uint32 bytesPerFrame = ma_get_bytes_per_frame(rb.format, rb.channels);
        if (bytesPerFrame == 0)
        {
            return 0;
        }

        const ma_uint8* inputBytes = static_cast<const ma_uint8*>(source);
        ma_uint32 totalFramesWritten = 0;

        while (totalFramesWritten < requestedFrames)
        {
            ma_uint32 framesToWrite = requestedFrames - totalFramesWritten;
            void* pBlock = nullptr;

            if (ma_pcm_rb_acquire_write(&rb, &framesToWrite, &pBlock) != MA_SUCCESS || framesToWrite == 0)
            {
                break;
            }

            std::memcpy(pBlock,
                inputBytes + (totalFramesWritten * bytesPerFrame),
                framesToWrite * bytesPerFrame);

            if (ma_pcm_rb_commit_write(&rb, framesToWrite) != MA_SUCCESS)
            {
                break;
            }

            totalFramesWritten += framesToWrite;
        }

        return totalFramesWritten;
    }

    ma_result PcmRingBuffer::seekRead(ma_uint32 offsetFrames) noexcept
    {
        return initialized ? ma_pcm_rb_seek_read(&rb, offsetFrames) : MA_INVALID_OPERATION;
    }

    ma_result PcmRingBuffer::seekWrite(ma_uint32 offsetFrames) noexcept
    {
        return initialized ? ma_pcm_rb_seek_write(&rb, offsetFrames) : MA_INVALID_OPERATION;
    }

    ma_uint32 PcmRingBuffer::subbufferSize(ma_uint32 subbufferIndex) const noexcept
    {
        return initialized ? ma_pcm_rb_get_subbuffer_size(const_cast<ma_pcm_rb*>(&rb)) : 0;
    }

    ma_uint32 PcmRingBuffer::subbufferOffset(ma_uint32 subbufferIndex) const noexcept
    {
        return initialized ? ma_pcm_rb_get_subbuffer_offset(const_cast<ma_pcm_rb*>(&rb), subbufferIndex) : 0;
    }

    void* PcmRingBuffer::subbufferPtr(ma_uint32 subbufferIndex, void* baseBuffer) const noexcept
    {
        return initialized ? ma_pcm_rb_get_subbuffer_ptr(const_cast<ma_pcm_rb*>(&rb), subbufferIndex, baseBuffer) : nullptr;
    }

    void PcmRingBuffer::moveFrom(PcmRingBuffer& other) noexcept
    {
        rb = other.rb;
        initialized = other.initialized;
        frameCapacityValue = other.frameCapacityValue;
        subbufferSizeValue = other.subbufferSizeValue;
        subbufferCountValue = other.subbufferCountValue;
        subbufferStrideValue = other.subbufferStrideValue;
        deinterleavedValue = other.deinterleavedValue;

        other.rb = {};
        other.initialized = false;
        other.frameCapacityValue = 0;
        other.subbufferSizeValue = 0;
        other.subbufferCountValue = 0;
        other.subbufferStrideValue = 0;
        other.deinterleavedValue = false;
    }
}