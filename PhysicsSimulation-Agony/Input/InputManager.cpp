#include "InputManager.h"

void InputManager::onKey(int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
    {
        pressedKeys.insert(key);
    }
    else if (action == GLFW_RELEASE)
    {
        pressedKeys.erase(key);
    }
}

void InputManager::onMouseButton(int button, int action, int mods)
{
    if (button < 0 || button >= pressedMouseButtons.size())
    {
        return;
    }
    pressedMouseButtons[button] = action == GLFW_PRESS;
}

void InputManager::onMousePosition(double xpos, double ypos)
{
    mousePosition = glm::dvec2(xpos, ypos);

    if (firstMouse)
    {
        previousMousePosition = mousePosition;
        firstMouse = false;
    }

    mouseDelta += mousePosition - previousMousePosition;
    previousMousePosition = mousePosition;
}

void InputManager::onMouseScroll(double xoffset, double yoffset)
{
    scrollDelta += glm::dvec2(xoffset, yoffset);
}

void InputManager::processInput()
{
    // Process all keys that are in the keyStates map
    for (auto& [key, state] : keyStates)
    {
        // Store previous pressed state
        state.previousPressed = state.pressed;

        // Check if the key is currently pressed in the current frame
        bool currentlyPressed = pressedKeys.contains(key);

        // Update current pressed state
        state.pressed = currentlyPressed;

        // Determine justPressed: pressed now but wasn't pressed before
        state.justPressed = (uint8_t)((bool)state.pressed && !(bool)state.previousPressed);

        // Determine justReleased: not pressed now but was pressed before
        state.justReleased = (uint8_t)(!(bool)state.pressed && (bool)state.previousPressed);
    }

    // Add new keys that were pressed but aren't in the keyStates map yet
    for (int key : pressedKeys)
    {
        if (!keyStates.contains(key))
        {
            KeyState newState;
            newState.pressed = 1;
            newState.justPressed = 1;  // First time we see this key, it's "just pressed"
            newState.justReleased = 0;
            newState.previousPressed = 0;
            keyStates[key] = newState;
        }
    }

    // Process all mouse buttons
    for (int button = 0; button < mouseButtonStates.size(); button++)
    {
        // Get key state
        auto& state = mouseButtonStates[button];
        
        // Store previous pressed state
        state.previousPressed = state.pressed;

        // Check if the button is currently pressed in the current frame
        bool currentlyPressed = pressedMouseButtons[button];

        // Update current pressed state
        state.pressed = currentlyPressed;

        // Determine justPressed: pressed now but wasn't pressed before
        state.justPressed = (uint8_t)((bool)state.pressed && !(bool)state.previousPressed);

        // Determine justReleased: not pressed now but was pressed before
        state.justReleased = (uint8_t)(!(bool)state.pressed && (bool)state.previousPressed);
    }

    // Reset for next frame
    returnMouseDelta = mouseDelta;
    mouseDelta = glm::dvec2(0.0);

    returnScrollDelta = scrollDelta;
    scrollDelta = glm::dvec2(0.0);
}

bool InputManager::isKeyPressed(int key) const
{
    auto it = keyStates.find(key);
    return it != keyStates.end() && it->second.pressed;
}

bool InputManager::isKeyJustPressed(int key) const
{
    auto it = keyStates.find(key);
    return it != keyStates.end() && it->second.justPressed;
}

bool InputManager::isKeyJustReleased(int key) const
{
    auto it = keyStates.find(key);
    return it != keyStates.end() && it->second.justReleased;
}

bool InputManager::isMouseButtonPressed(int button) const
{
    return (button < mouseButtonStates.size() && button >= 0) && mouseButtonStates[button].pressed;
}

bool InputManager::isMouseButtonJustPressed(int button) const
{
    return (button < mouseButtonStates.size() && button >= 0) && mouseButtonStates[button].justPressed;
}

bool InputManager::isMouseButtonJustReleased(int button) const
{
    return (button < mouseButtonStates.size() && button >= 0) && mouseButtonStates[button].justReleased;
}

glm::dvec2 InputManager::getNormalizedMousePosition() const
{
    if (widthHeightPtr == nullptr)
    {
        return mousePosition;
    }

    const double width = widthHeightPtr[0];
    const double height = widthHeightPtr[1];
    const double aspectRatio = width / height;

    glm::dvec2 norm = mousePosition / glm::dvec2(width, height);
    norm = norm * 2.0 - 1.0;
    norm.x *= aspectRatio;
    norm.y = -norm.y;

    return norm;
}
