#pragma once
#include "AudioEngine/SoundSample.h"
#include "MappedFile.h"
#include <filesystem>

namespace AudioEngine::Decoding
{
	struct AudioFileMetadata
	{
		uint32_t sampleRate = 0;
		uint16_t channelCount = 0;
	};

	struct AudioStreamMetadata
	{
		uint32_t sampleRate = 0;
		uint16_t channelCount = 0;
		uint64_t totalSampleFrames = 0; // 0 if unknown (e.g., live stream)
	};

	class IAudioDecoder
	{
	public:
		virtual std::vector<SoundSample> loadFile(const std::filesystem::path& filePath, AudioFileMetadata& metadata) = 0;
	};

	class IAudioStreamingDecoder
	{
	public:
		virtual ~IAudioStreamingDecoder() = default;

		// Open a file. Returns true if successful, and fills metadata.
		virtual bool open(const std::filesystem::path& filePath, AudioStreamMetadata& metadata) = 0;
		
		virtual size_t readSamples(SoundSample* buffer, size_t numSamples) = 0;
		virtual bool seekToFrame(size_t frameIndex) = 0;
		virtual void close() = 0;
		virtual bool isEOF() const = 0;
	};
}
