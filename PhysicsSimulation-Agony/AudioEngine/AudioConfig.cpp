#include "AudioConfig.h"

namespace AudioEngine::AudioConfigBuilder
{
    [[nodiscard]] static ma_device_type resolveDeviceType(AudioConfig::Mode mode) noexcept
    {
        switch (mode)
        {
        case AudioConfig::Mode::Playback: return ma_device_type_playback;
        case AudioConfig::Mode::Capture:  return ma_device_type_capture;
        case AudioConfig::Mode::Duplex:   return ma_device_type_duplex;
        case AudioConfig::Mode::Loopback: return ma_device_type_loopback;
        }
        return ma_device_type_playback;
    }

    /*[[nodiscard]] ma_format resolveFormat(AudioConfig::SampleFormat format) noexcept
    {
        switch (format)
        {
        case AudioConfig::SampleFormat::Native: return ma_format_unknown;
        case AudioConfig::SampleFormat::F32:    return ma_format_f32;
        case AudioConfig::SampleFormat::S16:    return ma_format_s16;
        case AudioConfig::SampleFormat::S24:    return ma_format_s24;
        case AudioConfig::SampleFormat::S32:    return ma_format_s32;
        case AudioConfig::SampleFormat::U8:     return ma_format_u8;
        }
        return ma_format_unknown;
    }*/

    [[nodiscard]] static ma_uint32 qualityFrames(AudioConfig::Quality quality) noexcept
    {
        switch (quality)
        {
        case AudioConfig::Quality::UltraLow:     return 64;
        case AudioConfig::Quality::LowLatency:   return 256;
        case AudioConfig::Quality::Balanced:     return 512;
        case AudioConfig::Quality::Conservative: return 1024;
        case AudioConfig::Quality::Safe:         return 2048;
        }
        return 512;
    }

    [[nodiscard]] static ma_uint32 qualityPeriods(AudioConfig::Quality quality) noexcept
    {
        switch (quality)
        {
        case AudioConfig::Quality::UltraLow:     return 2;
        case AudioConfig::Quality::LowLatency:   return 2;
        case AudioConfig::Quality::Balanced:     return 3;
        case AudioConfig::Quality::Conservative: return 3;
        case AudioConfig::Quality::Safe:         return 4;
        }
        return 3;
    }

    static void applyQuality(const AudioConfig& config, ma_device_config& cfg)
    {
        cfg.periodSizeInFrames = config.periodFrames ? config.periodFrames : qualityFrames(config.quality);
        cfg.periods = config.periods ? config.periods : qualityPeriods(config.quality);

        cfg.performanceProfile =
            (config.quality == AudioConfig::Quality::UltraLow ||
                config.quality == AudioConfig::Quality::LowLatency)
            ? ma_performance_profile_low_latency
            : ma_performance_profile_conservative;
    }

    ma_device_config buildDeviceConfig(const AudioConfig& config, ma_device_data_proc dataCallback, void* pUserData, ma_stop_proc stopCallback)
    {
        ma_device_config cfg = ma_device_config_init(resolveDeviceType(config.mode));

        cfg.sampleRate = config.sampleRate;

        ma_format format = ma_format_f32;// resolveFormat(config.format);
        cfg.playback.format = format;
        cfg.capture.format = format;

        ma_uint32 channels = static_cast<ma_uint32>(config.channels);
        cfg.playback.channels = channels;
        cfg.capture.channels = channels;

        ma_share_mode share_mode = config.preferExclusive ? ma_share_mode_exclusive : ma_share_mode_shared;
        cfg.playback.shareMode = share_mode;
        cfg.capture.shareMode = share_mode;

        cfg.playback.pDeviceID = config.playbackDeviceId;
        cfg.capture.pDeviceID = config.captureDeviceId;

        applyQuality(config, cfg);

        cfg.dataCallback = dataCallback;
        cfg.stopCallback = stopCallback;
        cfg.pUserData = pUserData;

        cfg.noPreSilencedOutputBuffer = MA_TRUE;
        cfg.noClip = MA_TRUE;
        cfg.noFixedSizedCallback = MA_TRUE;

        cfg.resampling.algorithm = ma_resample_algorithm_linear;
        cfg.resampling.linear.lpfOrder = 4;

        cfg.pulse.pStreamNamePlayback = config.pulseStreamName;
        cfg.pulse.pStreamNameCapture = config.pulseStreamName;

        if (config.quality == AudioConfig::Quality::UltraLow)
        {
            cfg.wasapi.noAutoConvertSRC = MA_TRUE;
            cfg.wasapi.noDefaultQualitySRC = MA_TRUE;
        }

        return cfg;
    }
}