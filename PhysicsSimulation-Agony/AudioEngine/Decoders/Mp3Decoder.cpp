#include "Mp3Decoder.h"
#include <fstream>

namespace AudioEngine::Decoding
{
	std::vector<SoundSample> Mp3Decoder::loadFile(const std::filesystem::path& filePath, AudioFileMetadata& metadata)
	{
        metadata = AudioFileMetadata{}; // reset metadata

        std::ifstream file(filePath, std::ios::binary);
        if (!file)
        {
            return {};
        }

        std::vector<uint8_t> fileData(std::istreambuf_iterator<char>(file), {});

        mp3dec_t dec;
        mp3dec_init(&dec);

        mp3d_sample_t frame[MINIMP3_MAX_SAMPLES_PER_FRAME]; // float[1152*2]
        mp3dec_frame_info_t info{};

        const uint8_t* ptr = fileData.data();
        int remaining = static_cast<int>(fileData.size());
        bool firstFrame = true;

        std::vector<SoundSample> samples;

        while (remaining > 0)
        {
            int sampleCount = mp3dec_decode_frame(&dec, ptr, remaining, frame, &info);

            if (info.frame_bytes == 0)
                break; // no more sync

            ptr += info.frame_bytes;
            remaining -= info.frame_bytes;

            if (sampleCount == 0)
                continue; // ID3 / reservoir frame, keep going

            if (firstFrame)
            {
                metadata.sampleRate = info.hz;
                metadata.channelCount = info.channels;
                firstFrame = false;
            }

            samples.insert(samples.end(), frame, frame + sampleCount * info.channels);
        }

        return samples;
	}

    bool Mp3StreamingDecoder::open(const std::filesystem::path& filePath, AudioStreamMetadata& metadata)
    {
        close();

        std::string error;
        if (!m_mappedFile.open(filePath, MappedFile::Access::ReadOnly,
            MappedFile::Creation::OpenExisting, 0, &error))
        {
            return false;
        }

        const uint8_t* fileData = reinterpret_cast<const uint8_t*>(m_mappedFile.getData());
        size_t fileSize = m_mappedFile.getSize();

        // MP3D_SEEK_TO_SAMPLE enables sample-accurate seeking (requires an index scan)
        const int flags = MP3D_SEEK_TO_SAMPLE;
        int res = mp3dec_ex_open_buf(&m_decoder, fileData, fileSize, flags);
        if (res != 0)
        {
            close();
            return false;
        }

        m_currentFrame = 0;
        m_eof = false;
        m_valid = true;

        metadata.sampleRate = m_decoder.info.hz;
        metadata.channelCount = m_decoder.info.channels;
        metadata.totalSampleFrames = m_decoder.samples / m_decoder.info.channels;

        return true;
    }

    size_t Mp3StreamingDecoder::readSamples(SoundSample* buffer, size_t numSamples)
    {
        if (!m_valid || m_eof || buffer == nullptr || numSamples == 0)
            return 0;

        size_t samplesRead = mp3dec_ex_read(&m_decoder, buffer, numSamples);
        m_currentFrame += samplesRead / m_decoder.info.channels;

        if (samplesRead == 0)
            m_eof = true;

        return samplesRead;
    }

    bool Mp3StreamingDecoder::seekToFrame(size_t frameIndex)
    {
        if (!m_valid)
            return false;

        uint64_t samplePos = frameIndex * m_decoder.info.channels;
        if (samplePos >= m_decoder.samples)
            return false;

        if (mp3dec_ex_seek(&m_decoder, samplePos) == 0)
        {
            m_currentFrame = frameIndex;
            m_eof = false;
            return true;
        }

        return false;
    }

    void Mp3StreamingDecoder::close()
    {
        if (m_valid)
        {
            mp3dec_ex_close(&m_decoder);
            m_valid = false;
        }
        m_mappedFile.close();
        m_currentFrame = 0;
        m_eof = true;
    }
}