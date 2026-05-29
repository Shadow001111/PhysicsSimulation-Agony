#include "Player.h"

#include "Core/TracyProfiler.h"

#include <iostream>
#include <cmath>
#include <format>

namespace AudioEngine
{
    static inline float lerpf(float a, float b, float t)
    {
        return a + (b - a) * t;
    }


    void Voice::setPan(float pan) noexcept
    {
        constexpr float PI = 3.14159265358979323846f;
        constexpr float HALF_PI = PI * 0.5;

        pan = std::clamp(pan, -1.0f, 1.0f);
        const float panNorm = (pan + 1.0f) * 0.5f;
        const float panNormPi = panNorm * HALF_PI;
        leftGain = std::cos(panNormPi);
        rightGain = std::sin(panNormPi);
    }


    Player& Player::getGlobalInstance()
    {
        static Player instance;
        return instance;
    }

    bool Player::init(AudioConfig* userConfigPtr, uint32_t maxVoices)
    {
        if (mInitialized) return true;

        // Config
        ma_device_config config = ma_device_config_init(ma_device_type_playback);
        if (userConfigPtr)
        {
            userConfigPtr->mode = AudioConfig::Mode::Playback;
            const AudioConfig& configRef = *userConfigPtr;
            config = AudioConfigBuilder::buildDeviceConfig(configRef, nullptr, nullptr, nullptr);
        }
        else
        {
            config.playback.format = ma_format_f32;

            config.noClip = MA_TRUE;                    // disable built-in clipping
            config.noPreSilencedOutputBuffer = MA_TRUE; // skip memset(0) on buffer
            config.noFixedSizedCallback = MA_TRUE;
        }

        config.pUserData = this;
        config.dataCallback = &Player::dataCallback;
        config.stopCallback = nullptr;

        // Initialize device
        ma_result result = ma_device_init(nullptr, &config, &mDevice);
        if (result != MA_SUCCESS)
        {
            std::cerr << "ma_device_init failed: " << result << "\n";
            return false;
        }

        // Wake-up device
        result = ma_device_start(&mDevice);
        if (result != MA_SUCCESS)
        {
            std::cerr << "ma_device_start failed: " << result << "\n";
            ma_device_uninit(&mDevice);
            return false;
        }

        mOutputSampleRate = mDevice.sampleRate;
        mOutputChannels = mDevice.playback.channels;
        mMaxVoices = maxVoices;
        mVoices.resize(mMaxVoices);

        mInitialized = true;
        return true;
    }

    void Player::shutdown()
    {
        if (!mInitialized) return;
        ma_device_uninit(&mDevice);
        mInitialized = false;
    }

    std::optional<VoiceId> Player::playSound(SoundResource sound, float volume, float pitch, float pan, bool loop)
    {
        TRACY_SCOPE_N("Play");

        // Validate the sound is still alive before creating a voice.
        if (!sound) return std::nullopt;

        std::lock_guard<std::mutex> lock(mVoiceMutex);

        auto freeIt = std::find_if(mVoices.begin(), mVoices.end(), [](const Voice& v)
            {
                return !v.isActive();
            });
        if (freeIt == mVoices.end())
        {
            return std::nullopt;
        }

        auto voiceOpt = makeVoice(std::move(sound), volume, pitch, pan, loop);
        if (!voiceOpt.has_value()) [[unlikely]]
        {
            return std::nullopt;
        }

        *freeIt = std::move(voiceOpt.value());
        return freeIt->handle;
    }

    void Player::stopVoice(VoiceId voiceHandle)
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            if (v.handle == voiceHandle && v.isActive())
            {
                setVoiceInactive(v);
                break;
            }
        }
    }

    void Player::stopAllVoices()
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            setVoiceInactive(v);
        }
    }

    void Player::pauseVoice(VoiceId voiceHandle)
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            if (v.handle == voiceHandle && v.isActive() && !v.pause)
            {
                v.pause = true;
                break;
            }
        }
    }

    void Player::resumeVoice(VoiceId voiceHandle)
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            if (v.handle == voiceHandle && v.isActive() && v.pause)
            {
                v.pause = false;
                break;
            }
        }
    }

    void Player::setVoiceVolume(VoiceId voiceHandle, float volume)
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            if (v.handle == voiceHandle && v.isActive())
            {
                v.volume = std::max(0.0f, volume);
                break;
            }
        }
    }

    void Player::setVoicePitch(VoiceId voiceHandle, float pitch)
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            if (v.handle == voiceHandle && v.isActive())
            {
                v.pitch = std::max(0.01f, pitch);
                break;
            }
        }
    }

    void Player::setVoicePan(VoiceId voiceHandle, float pan)
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            if (v.handle == voiceHandle && v.isActive())
            {
                v.setPan(pan);
                return;
            }
        }
    }

    void Player::setVoiceCursor(VoiceId voiceHandle, double cursorInSeconds)
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            if (v.handle == voiceHandle && v.isActive())
            {
                v.cursor = cursorInSeconds * v.sound->getSampleRate();
                break;
            }
        }
    }

    bool Player::isVoiceActive(VoiceId voiceHandle)
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            if (v.handle == voiceHandle && v.isActive())
            {
                return true;
            }
        }
        return false;
    }

    float Player::getVoiceVolume(VoiceId voiceHandle)
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            if (v.handle == voiceHandle && v.isActive())
            {
                return v.volume;
            }
        }
        return 0.0f;
    }

    float Player::getVoicePitch(VoiceId voiceHandle)
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            if (v.handle == voiceHandle && v.isActive())
            {
                return v.pitch;
            }
        }
        return 0.0f;
    }

    double Player::getVoiceCursor(VoiceId voiceHandle)
    {
        std::lock_guard<std::mutex> lock(mVoiceMutex);
        for (auto& v : mVoices)
        {
            if (v.handle == voiceHandle && v.isActive())
            {
                return v.cursor;
            }
        }
        return 0.0;
    }

    std::optional<Voice> Player::makeVoice(SoundResource sound, float volume, float pitch, float pan, bool loop) noexcept
    {
        auto idOpt = voiceIdPool.acquireId();
        if (!idOpt.has_value()) return std::nullopt;

        Voice v;
        v.handle = idOpt.value();
        v.sound = std::move(sound);
        v.loop = loop;
        v.volume = std::max(0.0f, volume);
        v.pitch = std::max(0.01f, pitch);
        v.setPan(pan);
        v.cursor = 0.0;
        return v;
    }

    void Player::setVoiceInactive(Voice& voice)
    {
        voice.sound = nullptr;
        voiceIdPool.releaseId(voice.handle);
    }

    void Player::readSoundFrames(const ISound& sound, size_t startFrame, size_t numFrames, SoundSample* outBuffer, bool loop) const
    {
        const uint16_t ch = sound.getChannelCount();
        const size_t totalFrames = sound.getFrameCount();
        size_t remaining = numFrames;
        size_t samplesWritten = 0;
        size_t currentFrame = startFrame;

        while (remaining > 0)
        {
            if (currentFrame >= totalFrames)
            {
                if (loop)
                {
                    currentFrame %= totalFrames;
                    continue;
                }

                // Silence
                std::fill(outBuffer + samplesWritten,
                    outBuffer + samplesWritten + remaining * ch,
                    0.0f);
                break;
            }

            size_t framesToRead = std::min(remaining, totalFrames - currentFrame);
            size_t sampleOffset = currentFrame * ch;
            sound.readSamples(outBuffer + samplesWritten, sampleOffset, framesToRead * ch);
            samplesWritten += framesToRead * ch;
            currentFrame += framesToRead;
            remaining -= framesToRead;
        }
    }

    void Player::mix(float* out, ma_uint32 frameCount)
    {
        TRACY_SCOPE_N("Mix");

        std::fill(out, out + frameCount * mOutputChannels, 0.0f);

        const float currentMasterVolume = getMasterVolume();
        const float volumeMultiplier = currentMasterVolume;

        {
            std::lock_guard<std::mutex> lock(mVoiceMutex);

            for (auto& voice : mVoices)
            {
                if (!voice.isActive() || voice.pause) continue;

                const ISound* sound = voice.sound.get();

                if (!sound->isValid()) [[unlikely]]
                {
                    setVoiceInactive(voice);
                    continue;
                }

                if (sound->getChannelCount() == 1)
                {
                    mixMonoVoice(voice, *sound, out, frameCount, volumeMultiplier);
                }
                else
                {
                    mixStereoVoice(voice, *sound, out, frameCount, volumeMultiplier);
                }
            }
        }

        // Clip output to [-1, 1].
        // No need for SIMD, compiler already optimizes it
        const size_t totalSamples = static_cast<size_t>(frameCount) * mOutputChannels;
        for (size_t i = 0; i < totalSamples; i++)
        {
            out[i] = std::clamp(out[i], -1.0f, 1.0f);
        }
    }

    template <size_t Channels>
    void Player::mixVoice(Voice& voice, const ISound& sound, float* out, ma_uint32 frameCount, float volumeMultiplier)
    {
        const auto sourceFrameCount = sound.getFrameCount();
        const double step = double(sound.getSampleRate()) / mOutputSampleRate * double(voice.pitch);
        const double endPos = voice.cursor + step * frameCount;

        const auto startFrame = static_cast<size_t>(voice.cursor);
        const auto framesNeeded = static_cast<size_t>(std::ceil(endPos)) + 2 - startFrame;

        static thread_local std::vector<SoundSample> sourceBuffer;
        sourceBuffer.resize(framesNeeded * Channels);
        readSoundFrames(sound, startFrame, framesNeeded, sourceBuffer.data(), voice.loop);

        const float monoGain = voice.volume * volumeMultiplier;
        const float leftGain = voice.leftGain * monoGain;
        const float rightGain = voice.rightGain * monoGain;

        for (ma_uint32 outFrame = 0; outFrame < frameCount; outFrame++)
        {
            const double currentPosition = voice.cursor + step * outFrame;
            const auto currentFrame = static_cast<size_t>(currentPosition);
            const float frac = static_cast<float>(currentPosition - currentFrame);

            const auto currentBase = (currentFrame - startFrame) * Channels;
            const auto nextBase = currentBase + Channels;

            const auto outBase = static_cast<size_t>(outFrame) * mOutputChannels;

            const bool hasNext = nextBase + Channels <= sourceBuffer.size();

            if constexpr (Channels == 1)
            {
                const float s0 = sourceBuffer[currentBase];

                const float s1 = hasNext ? sourceBuffer[nextBase] : s0;

                const float mixed = lerpf(s0, s1, frac);

                if (mOutputChannels == 1)
                {
                    out[outBase + 0] += mixed * monoGain;
                }
                else
                {
                    out[outBase + 0] += mixed * leftGain;
                    out[outBase + 1] += mixed * rightGain;
                }
            }
            else
            {
                const float s0L = sourceBuffer[currentBase + 0];
                const float s0R = sourceBuffer[currentBase + 1];
                
                const float s1L = hasNext ? sourceBuffer[nextBase + 0] : s0L;
                const float s1R = hasNext ? sourceBuffer[nextBase + 1] : s0R;

                const float mixedL = lerpf(s0L, s1L, frac);
                const float mixedR = lerpf(s0R, s1R, frac);

                if (mOutputChannels == 1)
                {
                    out[outBase] += 0.5f * (mixedL * leftGain + mixedR * rightGain);
                }
                else
                {
                    out[outBase + 0] += mixedL * leftGain;
                    out[outBase + 1] += mixedR * rightGain;
                }
            }
        }

        voice.cursor += step * frameCount;
        if (!voice.loop && voice.cursor >= sourceFrameCount)
            setVoiceInactive(voice);
    }

    void Player::mixMonoVoice(Voice& voice, const ISound& sound, float* out, ma_uint32 frameCount, float volumeMultiplier)
    {
        TRACY_SCOPE_N("Mix mono voice");
        mixVoice<1>(voice, sound, out, frameCount, volumeMultiplier);
    }

    void Player::mixStereoVoice(Voice& voice, const ISound& sound, float* out, ma_uint32 frameCount, float volumeMultiplier)
    {
        TRACY_SCOPE_N("Mix stereo voice");
        mixVoice<2>(voice, sound, out, frameCount, volumeMultiplier);
    }
    
    template <typename... Args>
    void print_fmt(std::string_view fmt, Args&&... args)
    {
        std::cout << std::vformat(fmt, std::make_format_args(args...));
    }

    bool Player::listAvailableDevices() const
    {
        // Human-readable representation for miniaudio sample formats
        auto format_str = [](ma_format fmt) -> std::string {
            switch (fmt) {
            case ma_format_f32: return "f32";
            case ma_format_s16: return "s16";
            case ma_format_s24: return "s24";
            case ma_format_s32: return "s32";
            case ma_format_u8:  return "u8";
            default:            return "?";
            }
            };

        // Initialise the audio context
        ma_context context;
        ma_result result = ma_context_init(nullptr, 0, nullptr, &context);
        if (result != MA_SUCCESS) {
            print_fmt("Failed to initialise audio context (error code: {})\n", static_cast<int>(result));
            return false;
        }

        // Retrieve all playback and capture devices
        ma_device_info* pPlayback;
        ma_uint32       playbackCount;
        ma_device_info* pCapture;
        ma_uint32       captureCount;

        result = ma_context_get_devices(&context, &pPlayback, &playbackCount,
            &pCapture, &captureCount);
        if (result != MA_SUCCESS) {
            print_fmt("Failed to enumerate devices (error code: {})\n", static_cast<int>(result));
            ma_context_uninit(&context);
            return false;
        }

        // Lambda that prints details for one device category
        auto print_device_category = [&](ma_device_type type,
            ma_device_info* devices,
            ma_uint32 count,
            const char* label) {
                print_fmt("=== {} ({} found) ===\n", label, count);
                for (ma_uint32 i = 0; i < count; ++i) {
                    const auto& dev = devices[i];
                    print_fmt("  [{}] \"{}\"", i, dev.name);
                    if (dev.isDefault) std::cout << " (Default)";
                    std::cout << '\n';

                    // get detailed info to list native formats
                    ma_device_info detailed;
                    result = ma_context_get_device_info(&context, type, &dev.id, &detailed);
                    if (result == MA_SUCCESS && detailed.nativeDataFormatCount > 0) {
                        std::cout << "    Native data formats:\n";
                        for (ma_uint32 j = 0; j < detailed.nativeDataFormatCount; ++j) {
                            const auto& fmt = detailed.nativeDataFormats[j];
                            print_fmt("      {} : {}ch, {} Hz\n",
                                format_str(fmt.format),
                                fmt.channels,
                                fmt.sampleRate);
                        }
                    }
                }
            };

        // Print both categories using the lambda
        print_device_category(ma_device_type_playback, pPlayback, playbackCount, "Playback Devices");
        print_device_category(ma_device_type_capture, pCapture, captureCount, "Capture Devices");

        ma_context_uninit(&context);
        return true;
    }
}
