#include "Camera3D.h"
#include <glm/gtc/matrix_transform.hpp>

void Camera3D::updateCameraVectors() const
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

void Camera3D::updateFrustum() const
{
	if (!frustumUpdateRequired) return;
	frustumUpdateRequired = false;

	updateCameraVectors();

	frustum.update(transform.position, forward, right, up, FOV, aspectRatio, nearPlane, farPlane);
}

Camera3D::Camera3D(
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

glm::mat4 Camera3D::getViewMatrix() const
{
	updateCameraVectors();
	return glm::lookAt(transform.position, transform.position + forward, up);
}

glm::mat4 Camera3D::getViewMatrixModified(const glm::dvec3& posMod) const
{
	updateCameraVectors();
	glm::dvec3 modifiedPos = glm::mod(transform.position, posMod);
	return glm::lookAt(modifiedPos, modifiedPos + forward, up);
}

glm::mat4 Camera3D::getProjectionMatrix() const
{
	return glm::perspective(FOV, aspectRatio, nearPlane, farPlane);
}

void Camera3D::setPosition(const glm::dvec3& position)
{
	transform.position = position;
	frustumUpdateRequired = true;
}

void Camera3D::setYaw(FloatType yaw)
{
	transform.yaw = yaw;
	vectorsUpdateRequired = true;
	frustumUpdateRequired = true;
}

void Camera3D::setPitch(FloatType pitch)
{
	transform.pitch = glm::clamp(pitch, -HALF_PI, HALF_PI);
	vectorsUpdateRequired = true;
	frustumUpdateRequired = true;
}

void Camera3D::setYawPitch(FloatType yaw, FloatType pitch)
{
	transform.yaw = yaw;
	transform.pitch = glm::clamp(pitch, -HALF_PI, HALF_PI);
	vectorsUpdateRequired = true;
	frustumUpdateRequired = true;
}

void Camera3D::setTransform(const TransformType& transform)
{
	this->transform = transform;
	this->transform.pitch = glm::clamp(transform.pitch, -HALF_PI, HALF_PI);
	vectorsUpdateRequired = true;
	frustumUpdateRequired = true;
}

void Camera3D::setFOV(FloatType fov)
{
	if (fov < 1.0f) fov = 1.0f;
	if (fov > 90.0f) fov = 90.0f;
	FOV = fov;
	frustumUpdateRequired = true;
}

void Camera3D::setAspectRatio(FloatType aspect)
{
	aspectRatio = aspect;
	frustumUpdateRequired = true;
}

void Camera3D::setFarPlane(FloatType farPlane)
{
	this->farPlane = farPlane;
	frustumUpdateRequired = true;
}

void Camera3D::move(const Vec3Type& delta)
{
	transform.position += delta;
	frustumUpdateRequired = true;
}

void Camera3D::rotate(FloatType deltaYaw, FloatType deltaPitch)
{
	transform.yaw += deltaYaw;
	transform.pitch = glm::clamp(transform.pitch + deltaPitch, -HALF_PI, HALF_PI);
	vectorsUpdateRequired = true;
	frustumUpdateRequired = true;
}

Camera3D::Vec3Type Camera3D::getForward() const
{
	updateCameraVectors();
	return forward;
}

Camera3D::Vec3Type Camera3D::getUp() const
{
	updateCameraVectors();
	return up;
}

Camera3D::Vec3Type Camera3D::getRight() const
{
	updateCameraVectors();
	return right;
}

const Camera3D::FrustumType& Camera3D::getFrustum() const
{
	updateFrustum();
	return frustum;
}
