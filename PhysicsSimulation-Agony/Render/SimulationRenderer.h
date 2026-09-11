#pragma once
#include "Physics/Camera2D.h"
#include "Physics/ObjectSoA.h"

#include "Ecstasy/OpenGL/Shader.h"
#include "Ecstasy/OpenGL/ImmutableBuffer.h"
#include "Ecstasy/OpenGL/VertexArray.h"

namespace PS_AGONY
{
	class Simulation;
}

namespace Render
{
	using namespace Ecstasy::OpenGL;
	using namespace PS_AGONY;

	class SimulationRenderer
	{
		struct CircleInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			float radius;
			uint32_t color; // 3 bytes used.
		};

		struct BoxInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			float halfWidth, halfHeight;
			uint32_t color; // 3 bytes used.
		};

		struct CircleRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
			std::vector<CircleInstanceData> instanceData;
		};

		struct BoxRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
			std::vector<BoxInstanceData> instanceData;
		};

		struct LineVertex
		{
			float x, y;
			uint32_t color;
		};

		struct SpringRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			Shader shader;
			std::vector<LineVertex> vertexData;
		};

		struct FloatAABB
		{
			float minX, minY;
			float maxX, maxY;
		};

		struct AABBResources
		{
			VertexArray vao;
			ImmutableBuffer vbo;
			ImmutableBuffer instanceVbo;
			Shader shader;
			std::vector<AABB> aabbs;
			std::vector<FloatAABB> instanceData;
		};

		struct DrawArraysIndirectCommand
		{
			uint32_t count;
			uint32_t instanceCount;
			uint32_t first;
			uint32_t baseInstance;
		};

		struct PolygonInstanceData
		{
			float positionX, positionY;
			float localCOMX, localCOMY;
			float rotation;
			uint32_t color;
			uint32_t textureId;
		};

		struct PolygonRenderResources
		{
			VertexArray vao;
			ImmutableBuffer vertexVbo;
			ImmutableBuffer instanceVbo;
			ImmutableBuffer indirectBuf;
			Shader shader;
			std::vector<glm::vec2> vertexData;
			std::vector<PolygonInstanceData> instanceData;
			std::vector<DrawArraysIndirectCommand> drawCommands;
		};

		// Resources.
		std::vector<ColliderIndex> foundColliders;
		std::vector<ColliderIndex> foundColliderShapes[(size_t)BodyType::COUNT];

		CircleRenderResources circleResources;
		BoxRenderResources boxResources;
		PolygonRenderResources polygonResources;
		SpringRenderResources springResources;

		AABBResources aabbResources;

		// Camera.
		Camera2D camera;

		// Simulation references.
		BodySoAViewer bodies;
		ColliderSoAViewer colliders;

		CircleSoAViewer circles;
		BoxSoAViewer boxes;
		PolygonSoAViewer polygons;
	public:
		SimulationRenderer() = default;
		~SimulationRenderer() = default;
		SimulationRenderer(const SimulationRenderer&) = delete;
		SimulationRenderer& operator=(const SimulationRenderer&) = delete;
		SimulationRenderer(SimulationRenderer&&) = delete;
		SimulationRenderer& operator=(SimulationRenderer&&) = delete;

		void init();
		void renderSimulation(const Simulation& simulation, Real simRenderAlpha, const AABB& cameraAABB);

		void renderObjectPreview(const void* params, BodyType type);

		Camera2D& getCamera() noexcept { return camera; }

		size_t getMemoryUsage() const noexcept;
		size_t getVideoMemoryUsage() const noexcept;
	private:
		void initShaders();
		void initBuffers();

		void queryCollidersForRender(const Simulation& simulation, const AABB& cameraAABB);

		// Collect data and render.

		void renderColliders(const Mat4& viewProjectionMatrix, Real simRenderAlpha);
		void renderBodyCentersOfMass(const Mat4& viewProjectionMatrix);
		void renderBodyPositions(const Mat4& viewProjectionMatrix);
		void renderBodyTruePositions(const Mat4& viewProjectionMatrix);

		void renderCircleColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, Real simRenderAlpha);
		void renderBoxColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, Real simRenderAlpha);
		void renderPolygonColliders(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix, Real simRenderAlpha);

		void renderColliderAABBs(const std::vector<ColliderIndex>& givenColliders, const Mat4& viewProjectionMatrix);
		void renderBroadPhaseAABBs(const Simulation& simulation, const Mat4& viewProjectionMatrix, const AABB& cameraAABB);
		void renderContactPoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, const AABB& cameraAABB);

		void renderSprings(const Simulation& simulation, const Mat4& viewProjectionMatrix, Real simRenderAlpha);
		void renderJoints(const Simulation& simulation, const Mat4& viewProjectionMatrix, Real simRenderAlpha);

		// Render shapes.

		void renderCircleShapes(const Mat4& viewProjectionMatrix);
		void renderBoxShapes(const Mat4& viewProjectionMatrix);
		void renderPolygonShapes(const Mat4& viewProjectionMatrix);

		void renderAABBs(const glm::vec3& color, const Mat4& viewProjectionMatrix);

		// Buffer helpers.

		void ensureCircleInstanceVboCapacity(size_t count);
		void ensureBoxInstanceVboCapacity(size_t count);
		void ensurePolygonBufferCapacity(size_t vertexCount, size_t polygonCount);

		void ensureAABBInstanceVboCapacity(size_t count);

		void ensureSpringBufferCapacity(size_t vertexCount);
	};
}