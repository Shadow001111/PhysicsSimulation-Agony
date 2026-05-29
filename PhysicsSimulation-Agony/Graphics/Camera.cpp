#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>

void Camera::updateCameraVectors() const
{
	if (!vectorsUpdateRequired) return;
	vectorsUpdateRequired = false;

	// Calculate the new Front vector
	glm::dvec3 front;
	front.x = sin(transform.yaw) * cos(transform.pitch);
	front.y = sin(transform.pitch);
	front.z = cos(transform.yaw) * cos(transform.pitch);
	this->forward = glm::normalize(front);

	// Calculate the Right and Up vectors
	right = glm::normalize(glm::cross(this->forward, worldUp));
	up = glm::normalize(glm::cross(right, this->forward));
}

void Camera::updateFrustum() const
{
	if (!frustumUpdateRequired) return;
	frustumUpdateRequired = false;

	updateCameraVectors();

	frustum.update(transform.position, forward, right, up, FOV, aspectRatio, nearPlane, farPlane);
}

Camera::Camera(
	const Vec3Type& position,
	FloatType yaw,
	FloatType pitch,
	FloatType FOV,
	FloatType aspectRatio,
	FloatType nearPlane,
	FloatType farPlane
) :
	transform(position, yaw, pitch), FOV(FOV), aspectRatio(aspectRatio), nearPlane(nearPlane), farPlane(farPlane)
{
}

glm::mat4 Camera::getViewMatrix() const
{
	updateCameraVectors();
	return glm::lookAt(transform.position, transform.position + forward, up);
}

glm::mat4 Camera::getViewMatrixModified(const glm::dvec3& posMod) const
{
	updateCameraVectors();
	glm::dvec3 modifiedPos = glm::mod(transform.position, posMod);
	return glm::lookAt(modifiedPos, modifiedPos + forward, up);
}

glm::mat4 Camera::getProjectionMatrix() const
{
	return glm::perspective(FOV, aspectRatio, nearPlane, farPlane);
}

void Camera::setPosition(const glm::dvec3& position)
{
	transform.position = position;
	frustumUpdateRequired = true;
}

void Camera::setYaw(FloatType yaw)
{
	transform.yaw = yaw;
	vectorsUpdateRequired = true;
	frustumUpdateRequired = true;
}

void Camera::setPitch(FloatType pitch)
{
	transform.pitch = glm::clamp(pitch, -HALF_PI, HALF_PI);
	vectorsUpdateRequired = true;
	frustumUpdateRequired = true;
}

void Camera::setYawPitch(FloatType yaw, FloatType pitch)
{
	transform.yaw = yaw;
	transform.pitch = glm::clamp(pitch, -HALF_PI, HALF_PI);
	vectorsUpdateRequired = true;
	frustumUpdateRequired = true;
}

void Camera::setTransform(const TransformType& transform)
{
	this->transform = transform;
	this->transform.pitch = glm::clamp(transform.pitch, -HALF_PI, HALF_PI);
	vectorsUpdateRequired = true;
	frustumUpdateRequired = true;
}

void Camera::setFOV(FloatType fov)
{
	if (fov < 1.0f) fov = 1.0f;
	if (fov > 90.0f) fov = 90.0f;
	FOV = fov;
	frustumUpdateRequired = true;
}

void Camera::setAspectRatio(FloatType aspect)
{
	aspectRatio = aspect;
	frustumUpdateRequired = true;
}

void Camera::setFarPlane(FloatType farPlane)
{
	this->farPlane = farPlane;
	frustumUpdateRequired = true;
}

void Camera::move(const Vec3Type& delta)
{
	transform.position += delta;
	frustumUpdateRequired = true;
}

void Camera::rotate(FloatType deltaYaw, FloatType deltaPitch)
{
	transform.yaw += deltaYaw;
	transform.pitch = glm::clamp(transform.pitch + deltaPitch, -HALF_PI, HALF_PI);
	vectorsUpdateRequired = true;
	frustumUpdateRequired = true;
}

Camera::Vec3Type Camera::getForward() const
{
	updateCameraVectors();
	return forward;
}

Camera::Vec3Type Camera::getUp() const
{
	updateCameraVectors();
	return up;
}

Camera::Vec3Type Camera::getRight() const
{
	updateCameraVectors();
	return right;
}

const Camera::FrustumType& Camera::getFrustum() const
{
	updateFrustum();
	return frustum;
}
