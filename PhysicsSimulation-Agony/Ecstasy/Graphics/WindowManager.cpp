#include "WindowManager.h"

#include <iostream>

WindowManager::WindowManager(const WindowParams& params)
{
    if (!glfwInit())
    {
        std::cerr << "[WindowManager]: Failed to initialize GLFW\n";
        return;
    }

    // OpenGL 4.6 core profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, params.resizable);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, params.openglDebug);
    glfwWindowHint(GLFW_DECORATED, !params.bolderlessFullscreen);

    // Create window
    int choosenWidth, choosenHeight;
    if (params.nativeFullscreen)
    {
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        glfwWindowHint(GLFW_RED_BITS, mode->redBits);
        glfwWindowHint(GLFW_GREEN_BITS, mode->greenBits);
        glfwWindowHint(GLFW_BLUE_BITS, mode->blueBits);
        glfwWindowHint(GLFW_REFRESH_RATE, mode->refreshRate);

        choosenWidth = mode->width;
        choosenHeight = mode->height;

        window = glfwCreateWindow(choosenWidth, choosenHeight, params.title.c_str(), monitor, nullptr);
    }
    else if (params.bolderlessFullscreen)
    {
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        glfwWindowHint(GLFW_RED_BITS, mode->redBits);
        glfwWindowHint(GLFW_GREEN_BITS, mode->greenBits);
        glfwWindowHint(GLFW_BLUE_BITS, mode->blueBits);
        glfwWindowHint(GLFW_REFRESH_RATE, mode->refreshRate);

        choosenWidth = mode->width;
        choosenHeight = mode->height;
        window = glfwCreateWindow(choosenWidth, choosenHeight, params.title.c_str(), nullptr, nullptr);
    }
    else
    {
        choosenWidth = params.width;
        choosenHeight = params.height;
        window = glfwCreateWindow(choosenWidth, choosenHeight, params.title.c_str(), nullptr, nullptr);
    }

    if (!window)
    {
        std::cerr << "[WindowManager]: Failed to create GLFW window\n";
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window);

    // Load OpenGL via GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "[WindowManager]: Failed to initialize GLAD\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return;
    }

    // Store pointer for callbacks
    glfwSetWindowUserPointer(window, this);

    // Setup callbacks
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetCursorPosCallback(window, cursorPosStaticCallback);
    glfwSetMouseButtonCallback(window, mouseButtonStaticCallback);
    glfwSetScrollCallback(window, scrollStaticCallback);

    // Set viewport
    glViewport(0, 0, choosenWidth, choosenHeight);

    // vsync
    glfwSwapInterval(params.vsync);

    // Debug
    if (params.openglDebug)
    {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(glDebugOutput, nullptr);
    
        // Enable all messages by default
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);

        // Disable notifications (less important messages)
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, nullptr, GL_FALSE);
    }

    // Set window size, aspect ratio and the rest
	width = choosenWidth;
	height = choosenHeight;
	aspectRatio = static_cast<float>(width) / static_cast<float>(height);
    vsync = params.vsync;

    // Aspect ratio constraint
    if (params.strictAspectRatio)
    {
        glfwSetWindowAspectRatio(window, width, height);
    }
}

WindowManager::~WindowManager()
{
    if (window)
    {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

void WindowManager::pollEvents() const
{
    glfwPollEvents();
}

void WindowManager::swapBuffers() const
{
    glfwSwapBuffers(window);
}

bool WindowManager::shouldClose() const
{
    return glfwWindowShouldClose(window);
}

void WindowManager::linkFramebuffer(FrameBuffer* fbo)
{
    if (fbo == nullptr)
    {
        std::cout << "[WindowManager][linkFramebuffer]: Cannot link nullptr\n";
        return;
    }
    linkedFramebuffers.insert(fbo);

    fbo->resize(width, height);
}

void WindowManager::unlinkFramebuffer(FrameBuffer* fbo)
{
    linkedFramebuffers.erase(fbo);
}

bool WindowManager::isFramebufferLinked(FrameBuffer* fbo) const
{
    return linkedFramebuffers.contains(fbo);
}

void WindowManager::linkInputManager(InputManager* im)
{
    if (im == nullptr)
    {
        std::cout << "[WindowManager][linkInputManager]: Cannot link nullptr\n";
        return;
    }
    im->widthHeightPtr = &width;
    linkedInputManagers.insert(im);
}

void WindowManager::unlinkInputManager(InputManager* im)
{
    im->widthHeightPtr = nullptr;
    linkedInputManagers.erase(im);
}

bool WindowManager::isInputManagerLinked(InputManager* im) const
{
    return linkedInputManagers.contains(im);
}

void WindowManager::setTitle(const std::string& title) const
{
    glfwSetWindowTitle(window, title.c_str());
}

bool WindowManager::isKeyPressed(int key) const
{
	return glfwGetKey(window, key) == GLFW_PRESS;
}

bool WindowManager::isMouseButtonPressed(int button) const
{
    return glfwGetMouseButton(window, button) == GLFW_PRESS;
}

void WindowManager::getMousePos(float& xpos, float& ypos) const
{
    double xpos_,  ypos_;
	glfwGetCursorPos(window, &xpos_, &ypos_);
    xpos = static_cast<float>(xpos_);
	ypos = static_cast<float>(ypos_);
}

void APIENTRY WindowManager::glDebugOutput(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam)
{
    // Ignore
    if (
        // Non-significant error/warning codes
        id == 131169 || id == 131185 || id == 131218 || id == 131204 ||
        // Pixel sync
        id == 131154 ||
        // Moving memory to HOST
        id == 131186
        ) return;

    std::cout << "---------------\n";
    std::cout << "Debug message (" << id << "): " << message << "\n";

    switch (source)
    {
    case GL_DEBUG_SOURCE_API:             std::cout << "Source: API"; break;
    case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   std::cout << "Source: Window System"; break;
    case GL_DEBUG_SOURCE_SHADER_COMPILER: std::cout << "Source: Shader Compiler"; break;
    case GL_DEBUG_SOURCE_THIRD_PARTY:     std::cout << "Source: Third Party"; break;
    case GL_DEBUG_SOURCE_APPLICATION:     std::cout << "Source: Application"; break;
    case GL_DEBUG_SOURCE_OTHER:           std::cout << "Source: Other"; break;
    } std::cout << "\n";

    switch (type)
    {
    case GL_DEBUG_TYPE_ERROR:               std::cout << "Type: Error"; break;
    case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: std::cout << "Type: Deprecated Behaviour"; break;
    case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  std::cout << "Type: Undefined Behaviour"; break;
    case GL_DEBUG_TYPE_PORTABILITY:         std::cout << "Type: Portability"; break;
    case GL_DEBUG_TYPE_PERFORMANCE:         std::cout << "Type: Performance"; break;
    case GL_DEBUG_TYPE_MARKER:              std::cout << "Type: Marker"; break;
    case GL_DEBUG_TYPE_PUSH_GROUP:          std::cout << "Type: Push Group"; break;
    case GL_DEBUG_TYPE_POP_GROUP:           std::cout << "Type: Pop Group"; break;
    case GL_DEBUG_TYPE_OTHER:               std::cout << "Type: Other"; break;
    } std::cout << "\n";

    switch (severity)
    {
    case GL_DEBUG_SEVERITY_HIGH:         std::cout << "Severity: high"; break;
    case GL_DEBUG_SEVERITY_MEDIUM:       std::cout << "Severity: medium"; break;
    case GL_DEBUG_SEVERITY_LOW:          std::cout << "Severity: low"; break;
    case GL_DEBUG_SEVERITY_NOTIFICATION: std::cout << "Severity: notification"; break;
    } std::cout << "\n";
    std::cout << "\n";
}

void WindowManager::setOpenGLDebugMessageFilter(GLenum source, GLenum type, GLenum severity, GLsizei count, const GLuint* ids, GLboolean enabled)
{
}

void WindowManager::framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    auto* gm = static_cast<WindowManager*>(glfwGetWindowUserPointer(window));
	if (gm) gm->onResize(width, height);
}

void WindowManager::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    auto* gm = static_cast<WindowManager*>(glfwGetWindowUserPointer(window));
    if (gm) gm->onKey(key, scancode, action, mods);
}

void WindowManager::cursorPosStaticCallback(GLFWwindow* window, double xpos, double ypos)
{
    auto* gm = static_cast<WindowManager*>(glfwGetWindowUserPointer(window));
    if (gm) gm->onCursorPos(xpos, ypos);
}

void WindowManager::mouseButtonStaticCallback(GLFWwindow* window, int button, int action, int mods)
{
    auto* gm = static_cast<WindowManager*>(glfwGetWindowUserPointer(window));
    if (gm) gm->onMouseButton(button, action, mods);
}

void WindowManager::scrollStaticCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    auto* gm = static_cast<WindowManager*>(glfwGetWindowUserPointer(window));
    if (gm) gm->onScroll(xoffset, yoffset);
}

void WindowManager::onResize(int width, int height)
{
    this->width = width;
    this->height = height;
    this->aspectRatio = static_cast<float>(width) / static_cast<float>(height);

    if (width > 0 && height > 0)
    {
        glViewport(0, 0, width, height);

        for (auto* framebuffer : linkedFramebuffers)
        {
            framebuffer->resize(width, height);
        }
    }
}

void WindowManager::onKey(int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
        return;
    }

    for (auto* im : linkedInputManagers)
    {
        im->onKey(key, scancode, action, mods);
    }
}

void WindowManager::onCursorPos(double xpos, double ypos)
{
    for (auto* im : linkedInputManagers)
    {
        im->onMousePosition(xpos, ypos);
    }
}

void WindowManager::onMouseButton(int button, int action, int mods)
{
    for (auto* im : linkedInputManagers)
    {
        im->onMouseButton(button, action, mods);
    }
}

void WindowManager::onScroll(double xoffset, double yoffset)
{
    for (auto* im : linkedInputManagers)
    {
        im->onMouseScroll(xoffset, yoffset);
    }
}