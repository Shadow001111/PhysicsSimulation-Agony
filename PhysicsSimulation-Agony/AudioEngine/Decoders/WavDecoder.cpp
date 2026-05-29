#include "WavDecoder.h"
#include <fstream>
#include <cstring>
#include <algorithm>

namespace AudioEngine::Decoding
{
    namespace WavUtils
    {
        struct HeaderInfo
        {
            uint16_t audioFormat = 0;
            uint16_t channels = 0;
            uint32_t sampleRate = 0;
            uint16_t bitsPerSample = 0;
            size_t dataOffset = 0;
            size_t dataSize = 0;
            size_t totalFrames = 0;
        };

        static __forceinline uint16_t readU16(const uint8_t* p) noexcept
        {
            return uint16_t(p[0] | (p[1] << 8));
        }

        static __forceinline uint32_t readU32(const uint8_t* p) noexcept
        {
            return uint32_t(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
        }

        // Parse WAV header from in-memory data. Returns true if valid, fills info.
        static bool parseHeader(const uint8_t* data, size_t dataSize, HeaderInfo& outInfo)
        {
            if (dataSize < 44) return false;
            if (std::memcmp(data, "RIFF", 4) != 0) return false;
            if (std::memcmp(data + 8, "WAVE", 4) != 0) return false;

            size_t offset = 12;
            bool foundFmt = false;
            bool foundData = false;

            while (offset + 8 <= dataSize)
            {
                const uint8_t* chunkId = data + offset;
                uint32_t chunkSize = readU32(data + offset + 4);
                offset += 8;

                if (offset + chunkSize > dataSize) break;

                if (std::memcmp(chunkId, "fmt ", 4) == 0)
                {
                    if (chunkSize < 16) return false;
                    outInfo.audioFormat = readU16(data + offset + 0);
                    outInfo.channels = readU16(data + offset + 2);
                    outInfo.sampleRate = readU32(data + offset + 4);
                    outInfo.bitsPerSample = readU16(data + offset + 14);
                    foundFmt = true;
                }
                else if (std::memcmp(chunkId, "data", 4) == 0)
                {
                    outInfo.dataOffset = offset;
                    outInfo.dataSize = chunkSize;
                    foundData = true;
                    // We can break early once data chunk is found (fmt must have been seen)
                    break;
                }

                offset += chunkSize;
                if (chunkSize & 1) ++offset; // pad byte
            }

            if (!foundFmt || !foundData) return false;

            // Validate format
            if (!(outInfo.channels == 1 || outInfo.channels == 2)) return false;
            if (!(outInfo.audioFormat == 1 || outInfo.audioFormat == 3)) return false;
            if (!(outInfo.bitsPerSample == 8 || outInfo.bitsPerSample == 16 ||
                outInfo.bitsPerSample == 24 || outInfo.bitsPerSample == 32)) return false;

            const size_t bytesPerFrame = (outInfo.bitsPerSample / 8) * outInfo.channels;
            if (bytesPerFrame == 0) return false;

            outInfo.totalFrames = outInfo.dataSize / bytesPerFrame;
            return true;
        }

        // Convert raw PCM bytes to interleaved float samples, clamping to [-1,1].
        // Returns number of float samples written (<= dstSamples).
        static size_t decodePCM(const uint8_t* src, size_t srcBytes,
            float* dst, size_t dstSamples,
            uint16_t audioFormat, uint16_t bitsPerSample)
        {
            const size_t bytesPerSample = bitsPerSample / 8;
            const size_t maxSamples = srcBytes / bytesPerSample;
            const size_t samplesToWrite = std::min(dstSamples, maxSamples);
            const uint8_t* srcPointerHead = src;

            for (size_t i = 0; i < samplesToWrite; ++i)
            {
                float sample = 0.0f;

                if (audioFormat == 1) // PCM
                {
                    if (bitsPerSample == 8)
                    {
                        uint8_t u = *srcPointerHead++;
                        sample = (static_cast<float>(u) - 128.0f) / 128.0f;
                    }
                    else if (bitsPerSample == 16)
                    {
                        int16_t s = static_cast<int16_t>(readU16(srcPointerHead));
                        srcPointerHead += 2;
                        sample = static_cast<float>(s) / 32768.0f;
                    }
                    else if (bitsPerSample == 24)
                    {
                        int32_t v = static_cast<int32_t>(srcPointerHead[0] | (srcPointerHead[1] << 8) | (srcPointerHead[2] << 16));
                        if (v & 0x800000) v |= ~0xFFFFFF;
                        srcPointerHead += 3;
                        sample = static_cast<float>(v) / 8388608.0f;
                    }
                    else // 32-bit PCM
                    {
                        int32_t s = static_cast<int32_t>(readU32(srcPointerHead));
                        srcPointerHead += 4;
                        sample = static_cast<float>(s) / 2147483648.0f;
                    }
                }
                else if (audioFormat == 3) // IEEE float
                {
                    float f;
                    std::memcpy(&f, srcPointerHead, sizeof(float));
                    srcPointerHead += 4;
                    sample = f;
                }

                dst[i] = std::clamp(sample, -1.0f, 1.0f);
            }

            return samplesToWrite;
        }
    }

    std::vector<SoundSample> WavDecoder::loadFile(const std::filesystem::path& filePath, AudioFileMetadata& metadata)
    {
        metadata = AudioFileMetadata{};

        std::ifstream file(filePath, std::ios::binary);
        if (!file) return {};

        file.seekg(0, std::ios::end);
        const std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);

        if (size < 44) return {};

        std::vector<uint8_t> data(static_cast<size_t>(size));
        if (!file.read(reinterpret_cast<char*>(data.data()), size))
            return {};

        WavUtils::HeaderInfo info;
        if (!WavUtils::parseHeader(data.data(), data.size(), info))
            return {};

        const uint8_t* pcmData = data.data() + info.dataOffset;
        const size_t frameCount = info.totalFrames; // computed in parseHeader

        std::vector<SoundSample> samples;
        samples.reserve(frameCount * info.channels);

        // Decode entire PCM data in one go using the shared utility
        const size_t totalSamples = frameCount * info.channels;
        samples.resize(totalSamples);
        WavUtils::decodePCM(pcmData, info.dataSize,
            samples.data(), totalSamples,
            info.audioFormat, info.bitsPerSample);

        metadata.sampleRate = info.sampleRate;
        metadata.channelCount = info.channels;

        return samples;
    }

    bool WavStreamingDecoder::open(const std::filesystem::path& filePath, AudioStreamMetadata& metadata)
    {
        close();

        std::string error;
        if (!m_mappedFile.open(filePath, MappedFile::Access::ReadOnly,
            MappedFile::Creation::OpenExisting, 0, &error))
            return false;

        const uint8_t* fileData = reinterpret_cast<const uint8_t*>(m_mappedFile.getData());
        size_t fileSize = m_mappedFile.getSize();

        WavUtils::HeaderInfo info;
        if (!WavUtils::parseHeader(fileData, fileSize, info))
        {
            close();
            return false;
        }

        // Store header info for later use
        m_audioFormat = info.audioFormat;
        m_channels = info.channels;
        m_sampleRate = info.sampleRate;
        m_bitsPerSample = info.bitsPerSample;
        m_totalFrames = info.totalFrames;

        m_pcmDataStart = fileData + info.dataOffset;
        m_pcmDataEnd = m_pcmDataStart + info.dataSize;
        m_readPtr = m_pcmDataStart;

        metadata.sampleRate = m_sampleRate;
        metadata.channelCount = m_channels;
        metadata.totalSampleFrames = m_totalFrames;

        m_currentFrame = 0;
        m_eof = (info.dataSize == 0);
        return true;
    }

    size_t WavStreamingDecoder::readSamples(float* buffer, size_t numSamples)
    {
        if (m_eof || buffer == nullptr || numSamples == 0)
            return 0;

        const size_t bytesPerFrame = (m_bitsPerSample / 8) * m_channels;
        const size_t framesRequested = numSamples / m_channels;
        const size_t bytesRequested = framesRequested * bytesPerFrame;

        const uint8_t* current = m_readPtr;
        size_t bytesAvailable = m_pcmDataEnd - current;
        if (bytesAvailable == 0)
        {
            m_eof = true;
            return 0;
        }

        size_t bytesToRead = std::min(bytesRequested, bytesAvailable);
        size_t samplesDecoded = WavUtils::decodePCM(current, bytesToRead,
            buffer, numSamples,
            m_audioFormat, m_bitsPerSample);
        size_t framesDecoded = samplesDecoded / m_channels;

        m_readPtr += framesDecoded * bytesPerFrame;
        m_currentFrame += framesDecoded;

        if (m_readPtr >= m_pcmDataEnd)
            m_eof = true;

        return samplesDecoded;
    }

    bool WavStreamingDecoder::seekToFrame(size_t frameIndex)
    {
        if (!m_mappedFile.getIsOpen() || frameIndex >= m_totalFrames)
            return false;

        const size_t bytesPerFrame = (m_bitsPerSample / 8) * m_channels;
        const uint8_t* newPtr = m_pcmDataStart + frameIndex * bytesPerFrame;

        if (newPtr > m_pcmDataEnd)
            return false;

        m_readPtr = newPtr;
        m_currentFrame = frameIndex;
        m_eof = false;
        return true;
    }

    void WavStreamingDecoder::close()
    {
        m_mappedFile.close();
        m_pcmDataStart = nullptr;
        m_pcmDataEnd = nullptr;
        m_readPtr = nullptr;
        m_sampleRate = 0;
        m_channels = 0;
        m_bitsPerSample = 0;
        m_audioFormat = 0;
        m_totalFrames = 0;
        m_currentFrame = 0;
        m_eof = true;
    }
}