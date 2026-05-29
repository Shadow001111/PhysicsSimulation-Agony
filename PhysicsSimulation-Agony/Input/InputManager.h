#pragma once
#include "robin_hood.h"
#include <cstdint>
#include <glm/glm.hpp>
#include <array>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#undef GLFW_INCLUDE_NONE

class InputManager
{
	friend class WindowManager;

	struct KeyState
	{
		uint8_t pressed			: 1 = 0;
		uint8_t previousPressed : 1 = 0;
		uint8_t justPressed		: 1 = 0;
		uint8_t justReleased	: 1 = 0;
	};

	struct MouseButtonState
	{
		uint8_t pressed			: 1 = 0;
		uint8_t previousPressed : 1 = 0;
		uint8_t justPressed		: 1 = 0;
		uint8_t justReleased	: 1 = 0;
	};

	robin_hood::unordered_flat_set<int> pressedKeys;
	std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> pressedMouseButtons = {};

	// TODO: Maybe make it array of size (GLFW_KEY_LAST - GLFW_KEY_SPACE + 1) instead of map?
	robin_hood::unordered_flat_map<uint16_t, KeyState> keyStates;
	std::array<MouseButtonState, GLFW_MOUSE_BUTTON_LAST + 1> mouseButtonStates;

	glm::dvec2 mousePosition = {};
	glm::dvec2 previousMousePosition = {};

	glm::dvec2 mouseDelta = {};
	glm::dvec2 returnMouseDelta = {};

	glm::dvec2 scrollDelta = {};
	glm::dvec2 returnScrollDelta = {};

	bool firstMouse = true;

	int* widthHeightPtr = nullptr;

	void onKey(int key, int scancode, int action, int mods);
	void onMouseButton(int button, int action, int mods);
	void onMousePosition(double xpos, double ypos);
	void onMouseScroll(double xoffset, double yoffset);
public:
	void processInput();

	// Keyboard input
	bool isKeyPressed(int key) const;
	bool isKeyJustPressed(int key) const;
	bool isKeyJustReleased(int key) const;

	// Mouse button input
	bool isMouseButtonPressed(int button) const;
	bool isMouseButtonJustPressed(int button) const;
	bool isMouseButtonJustReleased(int button) const;

	// Mouse vector getters
	const glm::dvec2& getMousePosition() const { return mousePosition; }
	const glm::dvec2& getMouseDelta() const { return returnMouseDelta; }
	const glm::dvec2& getScrollDelta() const { return returnScrollDelta; }

	glm::dvec2 getNormalizedMousePosition() const;
};

