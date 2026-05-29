#include "AudioLoader.h"

#include "Decoders/WavDecoder.h"
#include "Decoders/OggDecoder.h"
#include "Decoders/Mp3Decoder.h"

namespace AudioEngine
{
    bool AudioLoader::loadAudioFileInternal(Decoding::IAudioDecoder& decoder, const std::filesystem::path& path, StaticSound& out)
    {
        Decoding::AudioFileMetadata metadata{};
        std::vector<SoundSample> samples = decoder.loadFile(path, metadata);

        if (samples.empty())
        {
            return false;
        }

        out.setSamples(samples);
        out.sampleRate = metadata.sampleRate;
        out.channelCount = metadata.channelCount;
		return true;
    }

    bool AudioLoader::loadAudioFile(AudioFileExtension ext, const std::filesystem::path& path, StaticSound& out)
    {
        if (ext == AudioFileExtension::WAV)
        {
            Decoding::WavDecoder decoder;
            return loadAudioFileInternal(decoder, path, out);
        }
        else if (ext == AudioFileExtension::OGG)
        {
            Decoding::OggDecoder decoder;
            return loadAudioFileInternal(decoder, path, out);
        }
        else if (ext == AudioFileExtension::MP3)
        {
            Decoding::Mp3Decoder decoder;
            return loadAudioFileInternal(decoder, path, out);
		}
        return false;
    }
}
