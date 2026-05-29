#pragma once
#include "miniaudio/miniaudio.h"

namespace AudioEngine
{
    struct AudioConfig
    {
        enum class Mode
        {
            Playback,
            Capture,
            Duplex,
            Loopback,
        };

        enum class Quality
        {
            UltraLow,
            LowLatency,
            Balanced,
            Conservative,
            Safe,
        };

        //enum class SampleFormat
        //{
        //    Native,
        //    F32,
        //    S16,
        //    S24,
        //    S32,
        //    U8,
        //};

        enum class Channels : ma_uint32
        {
            Native     = 0,
            Mono       = 1,
            Stereo     = 2,
            Quad       = 4,
            Surround51 = 6,
            Surround71 = 8,
        };

        Mode         mode         = Mode::Playback;
        Quality      quality      = Quality::Balanced;
        //SampleFormat format       = SampleFormat::Native;
        Channels     channels     = Channels::Native;

        ma_uint32    sampleRate   = 0;   // 0 = native
        ma_uint32    periodFrames = 0;   // 0 = preset from quality
        ma_uint32    periods      = 0;   // 0 = preset from quality

        bool         preferExclusive = false;

        ma_device_id* playbackDeviceId = nullptr;
        ma_device_id* captureDeviceId  = nullptr;

        const char* pulseStreamName = "AudioApp";
    };

    namespace AudioConfigBuilder
    {
        [[nodiscard]] ma_device_config buildDeviceConfig(
            const AudioConfig& config,
            ma_device_data_proc dataCallback,
            void* pUserData = nullptr,
            ma_stop_proc stopCallback = nullptr);
    }
}