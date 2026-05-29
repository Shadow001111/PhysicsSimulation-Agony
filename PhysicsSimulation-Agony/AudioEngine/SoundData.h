#pragma once
#include "Core/Resource.h"

#include "SoundSample.h"

#include "Decoders/IAudioDecoder.h"

#include <cstdint>
#include <vector>
#include <memory>
#include <filesystem>

namespace AudioEngine
{
    class AudioLoader;

    class SoundSampleArray
    {
        size_t dataSize = 0;
        SoundSample* dataPtr = nullptr;
    public:
        SoundSampleArray() = default;

        // From vector
        SoundSampleArray(const std::vector<SoundSample>& inSamples) :
            dataSize(inSamples.size()),
            dataPtr(inSamples.size() > 0 ? new SoundSample[inSamples.size()] : nullptr)
        {
            std::copy(inSamples.begin(), inSamples.end(), dataPtr);
        }

        SoundSampleArray& operator=(const std::vector<SoundSample>& inSamples)
        {
            if (dataPtr)
            {
                delete[] dataPtr;
                dataPtr = nullptr;
            }

            dataSize = inSamples.size();
            if (dataSize > 0)
            {
                dataPtr = new SoundSample[dataSize];
                std::copy(inSamples.begin(), inSamples.end(), dataPtr);
            }

            return *this;
        }

        // From SoundSampleArray
        SoundSampleArray(const SoundSampleArray& other) :
            dataSize(other.dataSize),
            dataPtr(other.dataSize > 0 ? new SoundSample[other.dataSize] : nullptr)
        {
            std::copy(other.dataPtr, other.dataPtr + dataSize, dataPtr);
        }

        SoundSampleArray& operator=(const SoundSampleArray& other)
        {
            if (this != &other)
            {
                SoundSample* newData = nullptr;
                if (other.dataSize > 0)
                {
                    newData = new SoundSample[other.dataSize];
                    std::copy(other.dataPtr, other.dataPtr + other.dataSize, newData);
                }

                delete[] dataPtr;
                dataPtr = newData;
                dataSize = other.dataSize;
            }
            return *this;
        }

        SoundSampleArray(SoundSampleArray&& other) noexcept :
            dataSize(other.dataSize),
            dataPtr(other.dataPtr)
        {
            other.dataPtr = nullptr;
            other.dataSize = 0;
        }

        SoundSampleArray& operator=(SoundSampleArray&& other) noexcept
        {
            if (this != &other)
            {
                if (dataPtr)
                {
                    delete[] dataPtr;
                }
                dataPtr = other.dataPtr;
                dataSize = other.dataSize;
                other.dataPtr = nullptr;
                other.dataSize = 0;
            }
            return *this;
        }

        ~SoundSampleArray()
        {
            if (dataPtr)
            {
                delete[] dataPtr;
                dataPtr = nullptr;
            }
        }

        [[nodiscard]] size_t size() const noexcept { return dataSize; }
        [[nodiscard]] bool empty() const noexcept { return dataSize == 0; }
        [[nodiscard]] SoundSample* data() noexcept { return dataPtr; }
        [[nodiscard]] const SoundSample* data() const noexcept { return dataPtr; }
    };

    enum class SoundType
    {
        Static,
		Streaming
    };

    enum class AudioFileExtension
    {
        UNKNOWN,
        WAV,
        OGG,
        MP3
    };

    AudioFileExtension getFileExtensionFromString(const std::string& ext);

    std::unique_ptr< Decoding::IAudioStreamingDecoder> createStreamingDecoderFromFileExtension(AudioFileExtension ext);

    class ISound : public ResourceReferenceCounter<ISound>
    {
    public:
        virtual ~ISound() = default;

        virtual void setSamples(const std::vector<SoundSample>& inSamples) = 0;
        virtual void readSamples(SoundSample* out, size_t begin, size_t count) const noexcept = 0;

        virtual size_t getSampleCount() const noexcept = 0;
        virtual uint32_t getSampleRate() const noexcept = 0;
        virtual uint16_t getChannelCount() const noexcept = 0;
        virtual uint32_t getFrameCount() const noexcept = 0;
        virtual bool isValid() const noexcept = 0;
    };

    class StaticSound final : public ISound
    {
        friend class AudioLoader;

        SoundSampleArray samples; // Interleaved, normalized [-1, 1]
        uint32_t sampleRate = 0;
        uint16_t channelCount = 0;
    public:
        void setSamples(const std::vector<SoundSample>& inSamples) override
        {
            samples = inSamples;
        }

        void readSamples(SoundSample* out, size_t begin, size_t count) const noexcept override
        {
            if (!out || count == 0 || begin >= samples.size()) [[unlikely]]
            {
                return;
            }

            const size_t maxCount = samples.size() - begin;
            const size_t toCopy = std::min(count, maxCount);

            std::memcpy(out, samples.data() + begin, toCopy * sizeof(SoundSample));
        }

        size_t getSampleCount() const noexcept override { return samples.size(); }

        uint32_t getSampleRate() const noexcept override { return sampleRate; }

        uint16_t getChannelCount() const noexcept override { return channelCount; }

        uint32_t getFrameCount() const noexcept override
        {
            return channelCount ? static_cast<uint32_t>(samples.size() / channelCount) : 0;
        }

        bool isValid() const noexcept override
        {
            return
                channelCount != 0 &&
                samples.size() > 0 &&
                (samples.size() % channelCount) == 0;
        }

        StaticSound() = default;
        ~StaticSound() override = default;
    private:
        // No copying or moving
        StaticSound(const StaticSound&) = delete;
        StaticSound& operator=(const StaticSound&) = delete;
        StaticSound(StaticSound&&) = delete;
        StaticSound& operator=(StaticSound&&) = delete;

        SoundSample* writeSamples() noexcept { return samples.data(); }
    };

    class StreamingSound final : public ISound
    {
        std::unique_ptr<Decoding::IAudioStreamingDecoder> decoder;
        size_t totalFrames = 0;
        uint32_t sampleRate = 0;
        uint16_t channelCount = 0;
        bool valid = false;
    public:
        StreamingSound(const std::filesystem::path& filePath);

        void readSamples(SoundSample* out, size_t begin, size_t count) const noexcept override;

        size_t getSampleCount() const noexcept override { return totalFrames * channelCount; }

        uint32_t getSampleRate() const noexcept override { return sampleRate; }

        uint16_t getChannelCount() const noexcept override { return channelCount; }

        uint32_t getFrameCount() const noexcept override
        {
            return totalFrames;
        }

        bool isValid() const noexcept override
        {
            return valid;
        }

        StreamingSound() = default;
        ~StreamingSound() override = default;
    private:
        // No copying or moving
        StreamingSound(const StreamingSound&) = delete;
        StreamingSound& operator=(const StreamingSound&) = delete;
        StreamingSound(StreamingSound&&) = delete;
        StreamingSound& operator=(StreamingSound&&) = delete;

        void setSamples(const std::vector<SoundSample>& /*inSamples*/) override
        {
            // no-op: streaming sounds get data from file/map, not from in-memory vector
        }
    };

    using SoundResource = ResourcePtr<ISound>;
}