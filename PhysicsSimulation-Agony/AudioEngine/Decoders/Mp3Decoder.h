#pragma once
#include "IAudioDecoder.h"

#include "decoder_libs/minimp3.h"

namespace AudioEngine::Decoding
{
	class Mp3Decoder final : public IAudioDecoder
	{
	public:
		std::vector<SoundSample> loadFile(const std::filesystem::path& filePath, AudioFileMetadata& metadata) override;
	};

    class Mp3StreamingDecoder final : public IAudioStreamingDecoder
    {
    public:
        Mp3StreamingDecoder() = default;
        ~Mp3StreamingDecoder() override { close(); }

        bool open(const std::filesystem::path& filePath, AudioStreamMetadata& metadata) override;
        size_t readSamples(SoundSample* buffer, size_t numSamples) override;
        bool seekToFrame(size_t frameIndex) override;
        void close() override;
        bool isEOF() const override { return m_eof; }

    private:
        MappedFile m_mappedFile;
        //uint32_t m_sampleRate = 0; // = m_decoder.info.hz
        //uint16_t m_channels = 0; // = m_decoder.info.channels

        bool m_eof = true;
        bool m_valid = true;

        //size_t m_totalFrames = 0; // = m_decoder.samples / m_decoder.info.channels
        size_t m_currentFrame = 0;

        mp3dec_ex_t m_decoder{};
    };
}

