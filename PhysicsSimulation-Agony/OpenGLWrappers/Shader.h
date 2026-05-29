#pragma once
#include <string>
#include <vector>
#include "robin_hood.h"
#include <glad/glad.h>
#include <glm/mat4x4.hpp>
#include <optional>

class Shader
{
    GLuint id = 0;
    mutable robin_hood::unordered_flat_map<std::string, GLint> uniformLocationCache;
public:
    struct ShaderSource
    {
        GLenum type;
        std::string path;
    };

    Shader() = default;
    ~Shader();

    Shader(const Shader& other) = delete;
    Shader& operator=(const Shader& other) = delete;

    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    void create(const std::vector<ShaderSource>& sources);
    void destroy();

    void use() const;

    void setBool(const std::string& name, bool value) const;

    void setInt(const std::string& name, int value) const;
	void setIvec2(const std::string& name, int x, int y) const;
	void setIvec3(const std::string& name, int x, int y, int z) const;
	void setIvec4(const std::string& name, int x, int y, int z, int w) const;

	void setUint(const std::string& name, unsigned int value) const;
    void setUvec2(const std::string& name, unsigned int x, unsigned int y) const;
	void setUvec3(const std::string& name, unsigned int x, unsigned int y, unsigned int z) const;
	void setUvec4(const std::string& name, unsigned int x, unsigned int y, unsigned int z, unsigned int w) const;

    void setFloat(const std::string& name, float value) const;
    void setFloatArray(const std::string& name, const float* values, size_t length) const;
	void setVec2(const std::string& name, float x, float y) const;
    void setVec3(const std::string& name, float x, float y, float z) const;
	void setVec4(const std::string& name, float x, float y, float z, float w) const;

    void setMat4(const std::string& name, const glm::mat4& mat) const;

    void setHandleui64ARB(const std::string& name, GLuint64 handle) const;

    [[nodiscard]] GLuint getID() const noexcept { return id; }
    [[nodiscard]] bool isValid() const noexcept { return id != 0; }
private:
    GLint getUniformLocation(const std::string& name) const;

    std::optional<std::string> loadShaderSource(const std::string& filePath) const;
    
    GLuint compileShader(GLenum type, const std::string& source) const;

    [[nodiscard]] bool checkCompileErrors(GLuint shader, const std::string& type) const;
};