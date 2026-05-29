#include "Shader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <glm/gtc/type_ptr.hpp>
#include <utility>

Shader::~Shader()
{
    destroy();
}

Shader::Shader(Shader&& other) noexcept :
    id(std::exchange(other.id, 0)),
    uniformLocationCache(std::move(other.uniformLocationCache))
{
}

Shader& Shader::operator=(Shader&& other) noexcept
{
    if (this != &other)
    {
        if (id) glDeleteProgram(id);

        id = std::exchange(other.id, 0);
        uniformLocationCache = std::move(other.uniformLocationCache);
    }
    return *this;
}

void Shader::create(const std::vector<ShaderSource>& sources)
{
    destroy();

    auto deleteShaders = [](std::vector<GLuint>& ids)
        {
            for (GLuint s : ids) glDeleteShader(s);
            ids.clear();
        };

    std::vector<GLuint> shaderIDs;
    shaderIDs.reserve(sources.size());

    for (const auto& src : sources)
    {
        // Load shader file content
        auto sourceOpt = loadShaderSource(src.path);

        // Check if file was read
        if (!sourceOpt.has_value())
        {
            std::cerr << "[Shader][create]: Failed to load source: '" << src.path << "'\n";
            deleteShaders(shaderIDs);
            return;
        }
        
        // Compile shader
        GLuint shader = compileShader(src.type, sourceOpt.value());
        if (shader == 0)
        {
            std::cerr << "[Shader][create]: Failed to compile: '" << src.path << "'\n";
            deleteShaders(shaderIDs);
            return;
        }

        // Add shader id to dynamic array
        shaderIDs.push_back(shader);
    }

    id = glCreateProgram();

    for (GLuint shader : shaderIDs)
    {
        glAttachShader(id, shader);
    }

    glLinkProgram(id);

    deleteShaders(shaderIDs);

    if (!checkCompileErrors(id, "PROGRAM"))
    {
        destroy();
    }
}

void Shader::destroy()
{
    if (id)
    {
        glDeleteProgram(id);
        id = 0;
    }
    uniformLocationCache.clear();
}

void Shader::use() const
{
    glUseProgram(id);
}

GLint Shader::getUniformLocation(const std::string& name) const
{
    auto it = uniformLocationCache.find(name);
    if (it != uniformLocationCache.end()) return it->second;

    GLint location = glGetUniformLocation(id, name.c_str());
    if (location == -1)
    {
        std::cerr << "[Shader][getUniformLocation]: Uniform '" << name << "' does not exist\n";
    }
    else
    {
        uniformLocationCache.emplace(name, location);
    }
    return location;
}

void Shader::setBool(const std::string& name, bool value) const
{
    glProgramUniform1i(id, getUniformLocation(name), (int)value);
}

void Shader::setInt(const std::string& name, int value) const
{
    glProgramUniform1i(id, getUniformLocation(name), value);
}

void Shader::setIvec2(const std::string& name, int x, int y) const
{
	glProgramUniform2i(id, getUniformLocation(name), x, y);
}

void Shader::setIvec3(const std::string& name, int x, int y, int z) const
{
	glProgramUniform3i(id, getUniformLocation(name), x, y, z);
}

void Shader::setIvec4(const std::string& name, int x, int y, int z, int w) const
{
	glProgramUniform4i(id, getUniformLocation(name), x, y, z, w);
}

void Shader::setUint(const std::string& name, unsigned int value) const
{
	glProgramUniform1ui(id, getUniformLocation(name), value);
}

void Shader::setUvec2(const std::string& name, unsigned int x, unsigned int y) const
{
    glProgramUniform2ui(id, getUniformLocation(name), x, y);
}

void Shader::setUvec3(const std::string& name, unsigned int x, unsigned int y, unsigned int z) const
{
	glProgramUniform3ui(id, getUniformLocation(name), x, y, z);
}

void Shader::setUvec4(const std::string& name, unsigned int x, unsigned int y, unsigned int z, unsigned int w) const
{
	glProgramUniform4ui(id, getUniformLocation(name), x, y, z, w);
}

void Shader::setFloat(const std::string& name, float value) const
{
    glProgramUniform1f(id, getUniformLocation(name), value);
}

void Shader::setFloatArray(const std::string& name, const float* values, size_t length) const
{
    glProgramUniform1fv(id, getUniformLocation(name), length, values);
}

void Shader::setVec2(const std::string& name, float x, float y) const
{
	glProgramUniform2f(id, getUniformLocation(name), x, y);
}

void Shader::setVec3(const std::string& name, float x, float y, float z) const
{
    glProgramUniform3f(id, getUniformLocation(name), x, y, z);
}

void Shader::setVec4(const std::string& name, float x, float y, float z, float w) const
{
	glProgramUniform4f(id, getUniformLocation(name), x, y, z, w);
}

void Shader::setMat4(const std::string& name, const glm::mat4& mat) const
{
    glProgramUniformMatrix4fv(id, getUniformLocation(name), 1, GL_FALSE, glm::value_ptr(mat));
}

void Shader::setHandleui64ARB(const std::string& name, GLuint64 handle) const
{
    glProgramUniformHandleui64ARB(id, getUniformLocation(name), handle);
}

std::optional<std::string> Shader::loadShaderSource(const std::string& filePath) const
{
    std::ifstream file(filePath);
    std::stringstream buffer;
    if (file.is_open())
    {
        buffer << file.rdbuf();
        file.close();
        return buffer.str();
    }
    else
    {
        return std::nullopt;
    }
}

GLuint Shader::compileShader(GLenum type, const std::string& source) const
{
    GLuint shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    std::string typeStr;
    switch (type)
    {
        case GL_VERTEX_SHADER: typeStr = "VERTEX"; break;
        case GL_FRAGMENT_SHADER: typeStr = "FRAGMENT"; break;
        case GL_COMPUTE_SHADER: typeStr = "COMPUTE"; break;
        case GL_GEOMETRY_SHADER: typeStr = "GEOMETRY"; break;
        case GL_TESS_CONTROL_SHADER: typeStr = "TESS_CONTROL"; break;
        case GL_TESS_EVALUATION_SHADER: typeStr = "TESS_EVALUATION"; break;
        default: typeStr = "UNKNOWN"; break;
    }
    bool success = checkCompileErrors(shader, typeStr);
    if (!success)
    {
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool Shader::checkCompileErrors(GLuint shader, const std::string& type) const
{
    GLint success;
    GLchar infoLog[1024];
    if (type != "PROGRAM")
    {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            glGetShaderInfoLog(shader, sizeof(infoLog), nullptr, infoLog);
            std::cerr << "[Shader]: Shader compilation error (" << type << "):\n" << infoLog<< "\n";
        }
    }
    else
    {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success)
        {
            glGetProgramInfoLog(shader, sizeof(infoLog), nullptr, infoLog);
            std::cerr << "[Shader]: Program linking error:\n" << infoLog<< "\n";
        }
    }
    return success;
}