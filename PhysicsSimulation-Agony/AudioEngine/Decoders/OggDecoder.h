#pragma once
#include "IAudioDecoder.h"

#include "decoder_libs/stb_vorbis.h"

namespace AudioEngine::Decoding
{
	class OggDecoder final : public IAudioDecoder
	{
	public:
		std::vector<SoundSample> loadFile(const std::filesystem::path& filePath, AudioFileMetadata& metadata) override;
	};

    class OggStreamingDecoder final : public IAudioStreamingDecoder
    {
    public:
        OggStreamingDecoder() = default;
        ~OggStreamingDecoder() override { close(); }

        bool open(const std::filesystem::path& filePath, AudioStreamMetadata& metadata) override;
        size_t readSamples(SoundSample* buffer, size_t numSamples) override;
        bool seekToFrame(size_t frameIndex) override;
        void close() override;
        bool isEOF() const override { return m_eof; }
    private:
        MappedFile m_mappedFile;
        stb_vorbis* m_vorbis = nullptr;

        uint32_t m_sampleRate = 0;
        uint16_t m_channels = 0;

        bool m_eof = true;

        size_t m_totalFrames = 0;
        size_t m_currentFrame = 0;
    };
}
