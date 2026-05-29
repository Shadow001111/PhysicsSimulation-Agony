#include "SoundData.h"

#include "Decoders/WavDecoder.h"
#include "Decoders/OggDecoder.h"
#include "Decoders/Mp3Decoder.h"

#include <iostream>

namespace AudioEngine
{
    AudioFileExtension getFileExtensionFromString(const std::string& ext)
    {
        if (ext == "wav" || ext == "WAV") return AudioFileExtension::WAV;
        if (ext == "ogg" || ext == "OGG") return AudioFileExtension::OGG;
        if (ext == "mp3" || ext == "MP3") return AudioFileExtension::MP3;
        return AudioFileExtension::UNKNOWN;
    }

    std::unique_ptr<Decoding::IAudioStreamingDecoder> createStreamingDecoderFromFileExtension(AudioFileExtension ext)
    {
        std::unique_ptr<Decoding::IAudioStreamingDecoder> ptr;

        if (ext == AudioFileExtension::WAV)
        {
            ptr = std::make_unique<Decoding::WavStreamingDecoder>();
        }
        else if (ext == AudioFileExtension::OGG)
        {
            ptr = std::make_unique<Decoding::OggStreamingDecoder>();
        }
        else if (ext == AudioFileExtension::MP3)
        {
            ptr = std::make_unique<Decoding::Mp3StreamingDecoder>();
        }

        return std::move(ptr);
    }

    StreamingSound::StreamingSound(const std::filesystem::path& filePath)
    {
        std::string stringExt = filePath.extension().string();
        stringExt.erase(0, 1); // Remove the dot from the extension.

        AudioFileExtension ext = getFileExtensionFromString(stringExt);
        decoder = createStreamingDecoderFromFileExtension(ext);
        if (!decoder)
        {
            return;
        }

        Decoding::AudioStreamMetadata meta;
        if (!decoder->open(filePath, meta))
        {
            decoder.reset();
            return;
        }

        sampleRate = meta.sampleRate;
        channelCount = meta.channelCount;
        totalFrames = meta.totalSampleFrames;

        valid = sampleRate != 0 && channelCount != 0 && totalFrames != 0;
    }

    void StreamingSound::readSamples(SoundSample* out, size_t begin, size_t count) const noexcept
    {
        if (!valid || !out || count == 0 || channelCount == 0)
            return;

        // Convert sample index to frame index
        uint64_t startFrame = begin / channelCount;
        if (!decoder->seekToFrame(startFrame))
            return;;

        // Read up to 'count' samples (the decoder reads interleaved samples)
        size_t samplesRead = decoder->readSamples(out, static_cast<uint32_t>(count));
        // If we couldn't read enough, fill the rest with silence
        if (samplesRead < count)
        {
            std::fill(out + samplesRead, out + count, 0.0f);
        }
    }
}
