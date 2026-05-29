#pragma once
#include "IAudioDecoder.h"

namespace AudioEngine::Decoding
{
	class WavDecoder final : public IAudioDecoder
	{
		
	public:
		std::vector<SoundSample> loadFile(const std::filesystem::path& filePath, AudioFileMetadata& metadata) override;
	};


    class WavStreamingDecoder final : public IAudioStreamingDecoder
    {
        MappedFile m_mappedFile;

        const uint8_t* m_pcmDataStart = nullptr;
        const uint8_t* m_pcmDataEnd = nullptr;
        const uint8_t* m_readPtr = nullptr;

        uint32_t m_sampleRate = 0;
        uint16_t m_channels = 0;
        uint16_t m_bitsPerSample = 0;
        uint16_t m_audioFormat = 0; // 1 = PCM, 3 = IEEE float

        bool m_eof = true;

        size_t m_totalFrames = 0;
        size_t m_currentFrame = 0;
    public:
        WavStreamingDecoder() = default;
        ~WavStreamingDecoder() override { close(); }

        bool open(const std::filesystem::path& filePath, AudioStreamMetadata& metadata) override;
        size_t readSamples(float* buffer, size_t numSamples) override;
        bool seekToFrame(size_t frameIndex) override;
        void close() override;
        bool isEOF() const override { return m_eof; }
    };
}

