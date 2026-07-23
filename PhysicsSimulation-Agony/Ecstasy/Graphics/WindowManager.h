#pragma once
#include "OpenGLWrappers/FrameBuffer.h"

#include "Input/InputManager.h"

struct WindowParams
{
    int width = 800;
    int height = 600;
    std::string title = "OpenGL Window";
    bool nativeFullscreen = false;
    bool bolderlessFullscreen = false;
    bool resizable = false;
    bool vsync = false;
    bool openglDebug = false;
    bool strictAspectRatio = false;
};

class WindowManager
{
    GLFWwindow* window = nullptr;

    robin_hood::unordered_flat_set<FrameBuffer*> linkedFramebuffers;
    robin_hood::unordered_flat_set<InputManager*> linkedInputManagers;

    int width = 0, height = 0;
	float aspectRatio = 1.0f;
    bool vsync = false;
public:
    explicit WindowManager(const WindowParams& params);
	~WindowManager();

	void pollEvents() const;
    void swapBuffers() const;
	bool shouldClose() const;

    // Framebuffer linkage (for resizing)
    void linkFramebuffer(FrameBuffer* fbo);
    void unlinkFramebuffer(FrameBuffer* fbo);
    bool isFramebufferLinked(FrameBuffer* fbo) const;

    // Input manager linkage
    void linkInputManager(InputManager* im);
    void unlinkInputManager(InputManager* im);
    bool isInputManagerLinked(InputManager* im) const;

    // Setters
    void setTitle(const std::string& title) const;

	// Getters
    GLFWwindow* getWindow() const { return window; };
    int getWidth() const { return width; };
    int getHeight() const { return height; };
    float getAspectRatio() const { return aspectRatio; };
    bool getVSYNC() const { return vsync; };

    bool isIconified() const { return glfwGetWindowAttrib(window, GLFW_ICONIFIED); }
    bool isZeroSize() const { return !(width > 0 && height > 0); };

    // Keys and mouse
	bool isKeyPressed(int key) const;
    bool isMouseButtonPressed(int button) const;
	void getMousePos(float& xpos, float& ypos) const;

    // OpenGL debug
    static void APIENTRY glDebugOutput(GLenum source, GLenum type, GLuint id,
        GLenum severity, GLsizei length,
        const GLchar* message, const void* userParam);
    static void setOpenGLDebugMessageFilter(GLenum source = GL_DONT_CARE,
        GLenum type = GL_DEBUG_TYPE_OTHER,
        GLenum severity = GL_DONT_CARE,
        GLsizei count = 0,
        const GLuint* ids = nullptr,
        GLboolean enabled = GL_FALSE);

    // Static forwarding callbacks
	static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
	static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void cursorPosStaticCallback(GLFWwindow* window, double xpos, double ypos);
    static void mouseButtonStaticCallback(GLFWwindow* window, int button, int action, int mods);
    static void scrollStaticCallback(GLFWwindow* window, double xoffset, double yoffset);

    // Instance-level callback handling
	void onResize(int width, int height);
    void onKey(int key, int scancode, int action, int mods);
    void onCursorPos(double xpos, double ypos);
    void onMouseButton(int button, int action, int mods);
    void onScroll(double xoffset, double yoffset);
};

