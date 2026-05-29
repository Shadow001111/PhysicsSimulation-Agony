#pragma once
#include "AudioConfig.h"
#include "SoundData.h"

#include "Core/IdPool.h"

#include <mutex>
#include <optional>
#include <atomic>

namespace AudioEngine
{
    using VoiceId = uint32_t;

    struct Voice
    {
        SoundResource sound;
        VoiceId handle = 0;

        bool loop = false;
        bool pause = false;
        //bool reserved1, reserved2;

        float volume = 1.0f;
        float pitch = 1.0f;
        float leftGain = 1.0f; // Not multiplied by volume
        float rightGain = 1.0f; // Not multiplied by volume
        double cursor = 0.0;

        void setPan(float pan) noexcept;

        [[nodiscard]] bool isActive() const noexcept { return (bool)sound; }
        //void setInactive() { sound = nullptr; }
    };

    class Player
    {
        ma_device mDevice{};

        bool mInitialized = false;

        uint32_t mOutputSampleRate = 48000;
        uint32_t mOutputChannels = 2;
        uint32_t mMaxVoices = 64;

        std::atomic<float> masterVolume{ 1.0f };

        std::mutex mVoiceMutex;
        std::vector<Voice> mVoices;

        IdPool<VoiceId> voiceIdPool;
    public:
        static Player& getGlobalInstance();

        bool init(AudioConfig* userConfigPtr = nullptr, uint32_t maxVoices = 64);
        void shutdown();

        std::optional<VoiceId> playSound(SoundResource sound, float volume = 1.0f, float pitch = 1.0f, float pan = 0.0f, bool loop = false);

        // Voice stoping / pausing / resuming
        void stopVoice(VoiceId voiceHandle);
        void stopAllVoices();
        void pauseVoice(VoiceId voiceHandle);
        void resumeVoice(VoiceId voiceHandle);

        // Voice setters
        void setVoiceVolume(VoiceId voiceHandle, float volume);
        void setVoicePitch(VoiceId voiceHandle, float pitch);
        void setVoicePan(VoiceId voiceHandle, float pan);
        void setVoiceCursor(VoiceId voiceHandle, double cursorInSeconds);

        // Voice getters
        bool isVoiceActive(VoiceId voiceHandle);
        float getVoiceVolume(VoiceId voiceHandle);
        float getVoicePitch(VoiceId voiceHandle);
        //float getVoicePan(VoiceId voiceHandle);
        double getVoiceCursor(VoiceId voiceHandle);

        // Master setters
        void setMasterVolume(float volume) noexcept
        {
            masterVolume.store(std::clamp(volume, 0.0f, 1.0f), std::memory_order_relaxed);
        };

        // Master getters
        float getMasterVolume() const noexcept
        {
            return masterVolume.load(std::memory_order_relaxed);
        }
    private:
        // Allow only for global instance
        Player() = default;
        ~Player() { shutdown(); }
        Player(const Player& other) = delete;
        Player& operator=(const Player& other) = delete;
        Player(Player&& other) = delete;
        Player& operator=(Player&& other) = delete;

        static void dataCallback(ma_device* device, void* output, const void* /*input*/, ma_uint32 frameCount)
        {
            auto* self = static_cast<Player*>(device->pUserData);
            self->mix(static_cast<float*>(output), frameCount);
        }

        std::optional<Voice> makeVoice(SoundResource sound, float volume, float pitch, float pan, bool loop) noexcept;

        void setVoiceInactive(Voice& voice);

        void readSoundFrames(const ISound& sound, size_t startFrame, size_t numFrames, SoundSample* outBuffer, bool loop) const;

        void mix(float* out, ma_uint32 frameCount);

        template <size_t Channels>
        void mixVoice(Voice& voice, const ISound& sound, float* out, ma_uint32 frameCount, float volumeMultiplier);

        void mixMonoVoice(Voice& voice, const ISound& sound, float* out, ma_uint32 frameCount, float volumeMultiplier);
        void mixStereoVoice(Voice& voice, const ISound& sound, float* out, ma_uint32 frameCount, float volumeMultiplier);
    
        bool listAvailableDevices() const;
    };
}
