#pragma once
#include "SoundData.h"
#include "Decoders/IAudioDecoder.h"
#include <filesystem>

namespace AudioEngine
{
    class AudioLoader
    {
        static bool loadAudioFileInternal(Decoding::IAudioDecoder& decoder, const std::filesystem::path& path, StaticSound& out);
    public:
        static bool loadAudioFile(AudioFileExtension ext, const std::filesystem::path& path, StaticSound& out);
    };
}

