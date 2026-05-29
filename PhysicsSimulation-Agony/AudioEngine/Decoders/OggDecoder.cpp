#include "OggDecoder.h"

namespace AudioEngine::Decoding
{
    std::vector<SoundSample> OggDecoder::loadFile(const std::filesystem::path& filePath, AudioFileMetadata& metadata)
    {
        metadata = AudioFileMetadata{}; // reset metadata

        int channels = 0;
        int sampleRate = 0;
        short* pcm = nullptr;
        int samplesPerChannel = stb_vorbis_decode_filename(filePath.string().c_str(), &channels, &sampleRate, &pcm);
        if (samplesPerChannel <= 0)
        {
            free(pcm);
            return {};
        }

        metadata.sampleRate = uint32_t(sampleRate);
        metadata.channelCount = uint16_t(channels);

        size_t totalSamples = size_t(samplesPerChannel) * size_t(channels);
        std::vector<SoundSample> samples;
        samples.reserve(totalSamples);

        for (size_t i = 0; i < totalSamples; i++)
        {
            float sample = float(pcm[i]) / 32768.0f;
            samples.push_back(std::clamp(sample, -1.0f, 1.0f));
        }
        free(pcm);

		return samples;
    }

    bool OggStreamingDecoder::open(const std::filesystem::path& filePath, AudioStreamMetadata& metadata)
    {
        close();

        std::string error;
        if (!m_mappedFile.open(filePath, MappedFile::Access::ReadOnly,
            MappedFile::Creation::OpenExisting, 0, &error))
            return false;

        const uint8_t* data = reinterpret_cast<const uint8_t*>(m_mappedFile.getData());
        size_t size = m_mappedFile.getSize();

        int errorCode = 0;
        m_vorbis = stb_vorbis_open_memory(data, static_cast<int>(size), &errorCode, nullptr);
        if (!m_vorbis)
        {
            close();
            return false;
        }

        const stb_vorbis_info info = stb_vorbis_get_info(m_vorbis);
        m_sampleRate = info.sample_rate;
        m_channels = info.channels;
        m_totalFrames = stb_vorbis_stream_length_in_samples(m_vorbis);// / info.channels; // TODO: Check
        m_currentFrame = 0;
        m_eof = false;

        metadata.sampleRate = m_sampleRate;
        metadata.channelCount = m_channels;
        metadata.totalSampleFrames = m_totalFrames;
        return true;
    }

    size_t OggStreamingDecoder::readSamples(SoundSample* buffer, size_t numSamples)
    {
        if (m_eof || !m_vorbis || buffer == nullptr || numSamples == 0)
            return 0;

        // Ensure the request is a multiple of the channel count.
        const size_t channels = m_channels;
        numSamples -= numSamples % channels;
        if (numSamples == 0)
            return 0;

        const int framesRead = stb_vorbis_get_samples_float_interleaved(m_vorbis, channels, buffer, numSamples);

        if (framesRead <= 0)
        {
            //if (stb_vorbis_get_error(m_vorbis) != VORBIS__no_error)
            //{
            //
            //}
            m_eof = true;
            return 0;
        }

        const size_t samplesRead = static_cast<size_t>(framesRead) * channels;
        m_currentFrame += static_cast<size_t>(framesRead);

        return samplesRead;
    }

    bool OggStreamingDecoder::seekToFrame(size_t frameIndex)
    {
        if (!m_vorbis || frameIndex >= m_totalFrames)
            return false;

        if (frameIndex > UINT_MAX) [[unlikely]] return false;
        if (stb_vorbis_seek(m_vorbis, static_cast<unsigned int>(frameIndex)) != 0)
        {
            m_currentFrame = frameIndex;
            m_eof = false;
            return true;
        }
        return false;
    }

    void OggStreamingDecoder::close()
    {
        if (m_vorbis)
        {
            stb_vorbis_close(m_vorbis);
            m_vorbis = nullptr;
        }
        m_mappedFile.close();
        m_sampleRate = 0;
        m_channels = 0;
        m_totalFrames = 0;
        m_currentFrame = 0;
        m_eof = true;
    }
}