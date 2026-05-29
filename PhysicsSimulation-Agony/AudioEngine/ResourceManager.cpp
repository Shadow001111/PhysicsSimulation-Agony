#include "ResourceManager.h"
#include "AudioLoader.h"

#include <iostream>

namespace AudioEngine
{
    ResourceManager::~ResourceManager()
    {
        std::lock_guard lock(resourceMutex);
        soundStorage.clear();
    }

    ResourceManager::ResourceManager(const ResourceManager& other)
    {
        std::lock_guard lock1(other.resourceMutex);
        std::lock_guard lock2(resourceMutex);

        soundStorage = other.soundStorage;
        soundIdPool = other.soundIdPool;
    }

    ResourceManager& ResourceManager::operator=(const ResourceManager& other)
    {
        if (this != &other)
        {
            std::lock_guard lock1(other.resourceMutex);
            std::lock_guard lock2(resourceMutex);

            soundStorage = other.soundStorage;
            soundIdPool = other.soundIdPool;
        }
        return *this;
    }

    ResourceManager::ResourceManager(ResourceManager&& other)
    {
        std::lock_guard lock1(other.resourceMutex);
        std::lock_guard lock2(resourceMutex);

        soundStorage = std::move(other.soundStorage);
        soundIdPool = std::move(other.soundIdPool);
    }

    ResourceManager& ResourceManager::operator=(ResourceManager&& other)
    {
        if (this != &other)
        {
            std::lock_guard lock1(other.resourceMutex);
            std::lock_guard lock2(resourceMutex);

            soundStorage = std::move(other.soundStorage);
            soundIdPool = std::move(other.soundIdPool);
        }
        return *this;
    }

    std::optional<SoundId> ResourceManager::loadSound(AudioFileExtension ext, const std::filesystem::path& path, SoundType soundType)
    {
		std::unique_ptr<ISound> sound;

		if (soundType == SoundType::Static)
        {
			sound = std::make_unique<StaticSound>();
			StaticSound* soundPtr = static_cast<StaticSound*>(sound.get());
            if (!AudioLoader::loadAudioFile(ext, path, *soundPtr))
            {
                std::cerr << "Failed to load audio file: " << path << "\n";
                return std::nullopt;
            }
        }
        else if (soundType == SoundType::Streaming)
        {
			sound = std::make_unique<StreamingSound>(path);
        }

        std::lock_guard<std::mutex> lock(resourceMutex);

        auto idOpt = soundIdPool.acquireId();
        if (!idOpt.has_value()) [[unlikely]]
        {
            std::cerr << "SoundId pool is empty\n";
            return std::nullopt;
        }

        SoundId id = idOpt.value();
        soundStorage[id] = sound.release();
        return id;
    }

    std::optional<SoundId> ResourceManager::loadSound(const std::filesystem::path& path, SoundType soundType)
    {
        auto ext = path.extension().string();
        if (ext.empty()) [[unlikely]]
        {
            std::cerr << "File has no extension: " << path << "\n";
            return std::nullopt;
        }

        ext.erase(0, 1); // Remove the dot from the extension.
        AudioFileExtension fileExt = getFileExtensionFromString(ext);
        if (fileExt == AudioFileExtension::UNKNOWN) [[unlikely]]
        {
            std::cerr << "Unsupported audio file extension: " << ext << "\n";
            return std::nullopt;
        }
		return loadSound(fileExt, path, soundType);
    }

    bool ResourceManager::unloadSound(SoundId soundId)
    {
        std::lock_guard<std::mutex> lock(resourceMutex);
        auto it = soundStorage.find(soundId);
        if (it == soundStorage.end())
        {
            return false;
        }

        soundIdPool.releaseId(soundId);

        soundStorage.erase(it);
        return true;
    }

    SoundResource ResourceManager::getSound(SoundId soundId)
    {
        std::lock_guard<std::mutex> lock(resourceMutex);
        auto it = soundStorage.find(soundId);
        if (it != soundStorage.end())
        {
            return it->second;
        }
        return nullptr;
    }
}
