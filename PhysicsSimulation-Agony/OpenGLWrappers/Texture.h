#pragma once
#include <glad/glad.h>
#include <cstdint>

namespace TextureCompression
{
	enum class Format
	{
		// No compression.
		NONE,

		// Automatically selects the best available format for the given channel
		// count.
		AUTO,

		// -------------------------------------------------------------------
		// S3TC / DXT  (GL_EXT_texture_compression_s3tc)
		// -------------------------------------------------------------------

		// BC1 (DXT1): RGB or RGBA with 1-bit alpha. ~8:1 ratio.
		// Best for: opaque diffuse maps, environment textures, UI without soft
		// transparency. Avoid when smooth alpha gradients are needed.
		BC1,

		// BC3 (DXT5): RGBA with separately compressed alpha block. ~4:1 ratio.
		// Best for: diffuse textures with smooth transparency (foliage, glass),
		// packed data textures (RGB + smooth alpha mask).
		// Prefer BC7 when quality matters and BPTC is available.
		BC3,

		// -------------------------------------------------------------------
		// RGTC  (GL_ARB_texture_compression_rgtc)
		// Core since OpenGL 3.0. Best choice for 1- and 2-channel data.
		// -------------------------------------------------------------------

		// BC4: Single channel, unsigned. ~8:1 ratio.
		// Best for: greyscale textures, roughness maps, ambient occlusion,
		// height maps, single-channel masks.
		BC4,

		// BC4 signed: single channel, values in [-1, 1]. Same ratio as BC4.
		// Best for: signed single-channel data such as height deltas or
		// single-axis derivative maps.
		BC4_SIGNED,

		// BC5: Two channels, unsigned. ~4:1 ratio.
		// Best for: tangent-space normal maps (store X and Y, reconstruct Z
		// in the shader). Significantly better quality than storing normals
		// in BC3's alpha channel.
		BC5,

		// BC5 signed: two channels, values in [-1, 1]. Same ratio as BC5.
		// Best for: normal maps authored in signed space, velocity/flow maps.
		BC5_SIGNED,

		// -------------------------------------------------------------------
		// BPTC  (GL_ARB_texture_compression_bptc)
		// Available on all DX11-class GPUs (GTX 400+, Radeon HD 5000+).
		// Highest quality block compression available in standard OpenGL.
		// -------------------------------------------------------------------

		// BC6H: RGB HDR, unsigned half-float. ~6:1 ratio.
		// Best for: HDR environment/skybox textures, lightmaps, emissive maps.
		// Values are in [0, +inf).
		BC6H,

		// BC6H signed: RGB HDR, values in (-inf, +inf). Same ratio as BC6H.
		// Best for: HDR data that contains negative values such as signed
		// HDR difference textures or irradiance coefficients.
		BC6H_SIGNED,

		// BC7: RGBA LDR, very high quality. ~4:1 ratio.
		// Best for: high-quality diffuse/albedo maps, any RGBA texture where
		// BC1/BC3 artefacts are unacceptable. Noticeably better than BC3 on
		// sharp colour gradients, text, skin, and fine detail.
		// Also a good choice for RGB (the alpha block is simply unused).
		BC7,

		// -------------------------------------------------------------------
		// ASTC LDR  (GL_KHR_texture_compression_astc_ldr)
		// Standard on all mobile GPUs (Mali, Adreno, Apple). Desktop support
		// is not universal - check Support::astc before using.
		// Smaller block footprint = better quality but higher memory use.
		// -------------------------------------------------------------------

		// ASTC 4x4: 8 bpp. Same ratio as BC7 but more flexible channel support.
		// Best for: high-quality RGBA on mobile, normal maps, UI textures.
		ASTC_4x4,

		// ASTC 6x6: ~3.6 bpp. Good balance of quality and memory savings.
		// Best for: general diffuse and detail textures on mobile where
		// maximum quality is not the top priority.
		ASTC_6x6,

		// ASTC 8x8: ~2 bpp. Aggressive compression, artefacts visible up close.
		// Best for: large low-frequency textures (terrain, sky gradients),
		// textures viewed at distance, or memory-constrained environments.
		ASTC_8x8,
	};

	// Per-channel hint
	enum class Channels : uint32_t { R = 1, RG = 2, RGB = 3, RGBA = 4 };

	// Extension availability
	struct Support
	{
		bool s3tc = false;   // BC1 / BC3
		bool rgtc = false;   // BC4 / BC5
		bool bptc = false;   // BC6H / BC7
		bool astc = false;   // ASTC LDR
	};

	using block_size = uint32_t;
	struct BlockDimensions { block_size w, h; };

	void init();
	const Support& getSupport();
	bool isFormatSupported(Format format);

	// Returns the GL compressed internalFormat enum for a given Format.
	// For AUTO, selects the best available format for the channel count.
	// For NONE, returns GL_NONE (caller must provide their own internalFormat).
	GLenum resolveInternalFormat(Format format, Channels channels, bool isHDR = false);

	// Calculates the byte size of one mip level of pre-compressed data.
	// Returns 0 for NONE / unknown formats.
	size_t calcCompressedSize(Format format, int width, int height);

	// Human-readable name of format
	const char* getName(Format format);

	// Block size of texture format
	BlockDimensions getBlockSize(Format format);
	BlockDimensions getBlockSize(GLenum internalFormat);
}

class Texture
{
	struct Extensions
	{
		bool bindless = false;
	};

	struct GlobalData
	{
		Extensions extensions;

		GLfloat maxAnisotropy = 0;
	};
public:
	struct Parameters
	{
		GLenum minFilter = GL_NEAREST;
		GLenum magFilter = GL_NEAREST;
		GLenum wrapS = GL_CLAMP_TO_EDGE;
		GLenum wrapT = GL_CLAMP_TO_EDGE;
		GLenum wrapR = GL_CLAMP_TO_EDGE;

		float anisotropy = 1.0f;
	};
private:
	static GlobalData globalData;

	GLuint id = 0;
	GLenum type = 0;
	GLenum internalFormat = 0;

	Parameters parametrs;

	using texture_size = uint16_t;
	using mip_level = uint8_t;

	texture_size width = 0;
	texture_size height = 0;
	texture_size depth = 0;
	mip_level mipLevels = 1;

	bool resident = false;
	GLuint64 handle = 0;

	void applyParametrs() const;
public:
	static void initGlobalData();

	Texture() = default;
	~Texture();

	Texture(const Texture&) = delete;
	Texture& operator=(const Texture&) = delete;

	Texture(Texture&& other) noexcept;
	Texture& operator=(Texture&& other) noexcept;

	// Destroy
	void destroy();

	// Texture creation functions for different types
	void create1D(texture_size width, GLenum internalFormat, mip_level mipLevels = 1);
	void create2D(texture_size width, texture_size height, GLenum internalFormat, mip_level mipLevels = 1);
	void create3D(texture_size width, texture_size height, texture_size depth, GLenum internalFormat, mip_level mipLevels = 1);
	void create2DArray(texture_size width, texture_size height, texture_size layers, GLenum internalFormat, mip_level mipLevels = 1);

	void create2DCompressed(
		texture_size width, texture_size height,
		TextureCompression::Channels channels,
		TextureCompression::Format compression = TextureCompression::Format::AUTO,
		mip_level mipLevels = 1,
		bool isHDR = false);
	void create2DArrayCompressed(
		texture_size width, texture_size height, texture_size layers,
		TextureCompression::Channels channels,
		TextureCompression::Format compression = TextureCompression::Format::AUTO,
		mip_level mipLevels = 1,
		bool isHDR = false);

	// Texture resizing functions (it deletes old texture and creates new, since texture is immutable)
	void recreate1D(texture_size width);
	void recreate2D(texture_size width, texture_size height);
	void recreate3D(texture_size width, texture_size height, texture_size depth);

	// Data upload functions
	void uploadData(const void* data, GLenum dataType, mip_level level = 0);

	void uploadSubData1D(
		const void* data, texture_size xOffset, texture_size width,
		GLenum dataType, mip_level level = 0);
	void uploadSubData2D(
		const void* data, texture_size xOffset, texture_size yOffset,
		texture_size width, texture_size height, GLenum dataType, mip_level level = 0);
	void uploadSubData3D(
		const void* data, texture_size xOffset, texture_size yOffset, texture_size zOffset,
		texture_size width, texture_size height, texture_size depth, GLenum dataType, mip_level level = 0);
	void uploadSubData2DArray(
		const void* data, texture_size xOffset, texture_size yOffset, texture_size layer,
		texture_size width, texture_size height, GLenum dataType, mip_level level = 0);

	// Data download functions
	void readData(void* outData, GLsizei bufSize, GLenum dataType, mip_level level = 0) const;

	//
	void generateMipmaps();

	void setParameters(const Parameters& params);

	void bind() const;
	void bind(GLenum target) const;
	void unbind() const;
	static void unbind(GLenum target);
	void bindUnit(GLuint unit) const;

	// Handle managment
	void initHandle();
	void makeResident();
	void makeNonResident();

	// Getters
	[[nodiscard]] GLuint getID() const { return id; }
	[[nodiscard]] GLenum getType() const { return type; }
	[[nodiscard]] GLenum getInternalFormat() const { return internalFormat; }
	[[nodiscard]] texture_size getWidth() const { return width; }
	[[nodiscard]] texture_size getHeight() const { return height; }
	[[nodiscard]] texture_size getDepth() const { return depth; }
	[[nodiscard]] mip_level getMipLevels() const { return mipLevels; }
	[[nodiscard]] bool isResident() const { return resident; }
	[[nodiscard]] GLuint64 getHandle() const { return handle; }

	[[nodiscard]] GLenum getFormatFromInternalFormat() const;

	[[nodiscard]] const char* getInternalFormatName() const;

	[[nodiscard]] static bool isFormatCompressed(GLenum internalFormat);
	[[nodiscard]] bool isCompressed() const;

	[[nodiscard]] static const GlobalData& getGlobalData() { return globalData; }
	[[nodiscard]] static const Extensions& getExtensions() { return globalData.extensions; }
};