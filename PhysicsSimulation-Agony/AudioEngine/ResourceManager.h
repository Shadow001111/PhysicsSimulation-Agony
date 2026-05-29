#pragma once
#include "Core/IdPool.h"

#include "SoundData.h"

#include <unordered_map>
#include <mutex>

namespace AudioEngine
{
	using SoundId = uint32_t;

	class ResourceManager
	{
		std::unordered_map<SoundId, SoundResource> soundStorage;
		mutable std::mutex resourceMutex;

		IdPool<SoundId> soundIdPool;
	public:
		ResourceManager() = default;
		~ResourceManager();
		ResourceManager(const ResourceManager& other);
		ResourceManager& operator=(const ResourceManager& other);
		ResourceManager(ResourceManager&& other);
		ResourceManager& operator=(ResourceManager&& other);

		// Loads a sound file and returns its SoundId.
		std::optional<SoundId> loadSound(AudioFileExtension ext, const std::filesystem::path& path, SoundType soundType = SoundType::Static);

		// Overload that infers file extension from the path.
		std::optional<SoundId> loadSound(const std::filesystem::path& path, SoundType soundType = SoundType::Static);

		// Unloads a sound by its SoundId. Returns true on success, false if the SoundId was not found.
		bool unloadSound(SoundId soundId);

		// Returns a weak_ptr to the sound. Expired if the sound has been unloaded.
		SoundResource getSound(SoundId soundId);
	};
}

