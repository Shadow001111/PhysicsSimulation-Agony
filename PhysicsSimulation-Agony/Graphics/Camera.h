#pragma once
#include "Core/Shapes/Frustum.h"
#include "Core/Transform.h"

class Camera
{
	using FloatType = double;
	using Vec3Type = glm::vec<3, FloatType>;
	using TransformType = Transform<FloatType>;
	using FrustumType = FrustumSimd<FloatType>;

	constexpr static Vec3Type worldUp = { 0, 1, 0 };
	constexpr static FloatType HALF_PI = 1.5707;

	TransformType transform;

	mutable Vec3Type forward = {};
	mutable Vec3Type up = {};
	mutable Vec3Type right = {};

	mutable FrustumType frustum;

	mutable bool vectorsUpdateRequired = true;
	mutable bool frustumUpdateRequired = true;

	FloatType FOV; // Radians
	FloatType aspectRatio;
	FloatType nearPlane, farPlane;

	void updateCameraVectors() const;
	void updateFrustum() const;
public:
	Camera(const Vec3Type& position, FloatType yaw, FloatType pitch, FloatType FOV, FloatType aspectRatio, FloatType nearPlane, FloatType farPlane);

	glm::mat4 getViewMatrix() const;
	glm::mat4 getViewMatrixModified(const Vec3Type& posMod) const;
	glm::mat4 getProjectionMatrix() const;

	void setPosition(const Vec3Type& position);
	void setYaw(FloatType yaw);
	void setPitch(FloatType pitch);
	void setYawPitch(FloatType yaw, FloatType pitch);
	void setTransform(const TransformType& transform);
	void setFOV(FloatType fov);
	void setAspectRatio(FloatType aspect);
	void setFarPlane(FloatType farPlane);

	void move(const Vec3Type& delta);
	void rotate(FloatType deltaYaw, FloatType deltaPitch);

	Vec3Type getPosition() const { return transform.position; };
	FloatType getYaw() const { return transform.yaw; };
	FloatType getPitch() const { return transform.pitch; };
	TransformType getTransform() const { return transform; };
	FloatType getFOV() const { return FOV; };
	FloatType getAspectRatio() const { return aspectRatio; };
	FloatType getNear() const { return nearPlane; };
	FloatType getFar() const { return farPlane; };

	Vec3Type getForward() const;
	Vec3Type getUp() const;
	Vec3Type getRight() const;
	const FrustumType& getFrustum() const;
};

