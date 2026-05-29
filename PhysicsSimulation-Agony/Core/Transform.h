#pragma once
#include <glm/glm.hpp>

// Contains position and rotation
template<typename T>
struct Transform
{
	using Vec3Type = glm::vec<3, T>;

	Vec3Type position;
	T yaw, pitch; // Radians

	Transform();
	Transform(const Vec3Type& position, T yaw, T pitch);

	Transform interpolate(const Transform& other, T factor) const;
};

template<typename T>
inline Transform<T>::Transform() :
	position(0.0, 0.0, 0.0), yaw(0.0), pitch(0.0)
{
}

template<typename T>
inline Transform<T>::Transform(const Vec3Type& position, T yaw, T pitch) :
	position(position), yaw(yaw), pitch(pitch)
{
}

template<typename T>
inline Transform<T> Transform<T>::interpolate(const Transform& other, T factor) const
{
	auto iPosition = (1.0f - factor) * position + factor * other.position;
	auto iYaw = (1.0f - factor) * yaw + factor * other.yaw;
	auto iPitch = (1.0f - factor) * pitch + factor * other.pitch;
	return Transform<T>(
		iPosition, iYaw, iPitch
	);
}
