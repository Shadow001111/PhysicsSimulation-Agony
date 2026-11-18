#include "ContinuousCollisionDetector.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"

#include "Ecstasy/Core/Portablity.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

/*
 * CONTINUOUS COLLISION: DESIGN NOTES
 *
 * Sweep = ray of the core circle center vs the static shape inflated by the core radius (Minkowski sum with a disk):
 *   circle  -> circle (R + rc)
 *   capsule -> two end circles + two side slabs (capsule local x axis = collider rotation, like the narrow phase)
 *   box/polygon (convex) -> per edge: offset line hit inside the edge span, per vertex: circle (rc) hit. Exact.
 * A start already inside the inflated shape never hits.
 * Result time is in [0, 1], NO_HIT otherwise.
 */

namespace PS_AGONY2D
{
	namespace
	{
		constexpr Real NO_HIT = Real(2);
		constexpr uint64_t TRAVERSAL_STACK_SIZE = BroadPhaseInternal::MAX_STACK_CAPACITY;

		struct StaticShapes
		{
			const Real* posX;
			const Real* posY;
			const Real* cosR;
			const Real* sinR;
			const ColliderShapeType* type;
			const ObjectDenseIndex* shapeIndex;
			const Real* circleRadius;
			const Real* boxHalfWidth;
			const Real* boxHalfHeight;
			const Real* capsuleHalfLength;
			const Real* capsuleRadius;
			const PolygonGeometry* polygons;
		};

		// Ray (start m relative to circle center, direction d, dd = dot(d, d) > 0) vs circle. Entering root only.
		__forceinline Real castCircle(Real mx, Real my, Real dx, Real dy, Real dd, Real radius)
		{
			const Real c = mx * mx + my * my - radius * radius;
			const Real b = mx * dx + my * dy;
			if (c <= Real(0) || b >= Real(0)) return NO_HIT; // Inside or moving away.

			const Real disc = b * b - dd * c;
			if (disc < Real(0)) return NO_HIT;

			return (-b - std::sqrt(disc)) / dd;
		}

		// TODO: Improve.
		Real castBox(Vec2 q0, Vec2 dl, Real dd, Real hw, Real hh, Real radius)
		{
			// Mirror into the +x/+y quadrant.
			const Real sx = std::copysign(Real(1), q0.x);
			const Real sy = std::copysign(Real(1), q0.y);
			const Real qx = q0.x * sx;
			const Real qy = q0.y * sy;
			const Real vx = dl.x * sx;
			const Real vy = dl.y * sy;

			const Real w = hw + radius;
			const Real h = hh + radius;
			const Real gapX = qx - w;
			const Real gapY = qy - h;
			if (gapX <= Real(0) && gapY <= Real(0)) return NO_HIT; // Starts inside.

			// Slab entry time per axis: 0 when already between its planes, never when moving away.
			Real tx = Real(0);
			if (gapX > Real(0))
			{
				if (vx >= Real(0)) return NO_HIT;
				tx = gapX / -vx;
			}
			Real ty = Real(0);
			if (gapY > Real(0))
			{
				if (vy >= Real(0)) return NO_HIT;
				ty = gapY / -vy;
			}

			const Real t = std::fmax(tx, ty);
			if (t > Real(1)) return NO_HIT;

			// Entry point. Only the axis that did not set 't' needs a range check.
			const Real px = qx + vx * t;
			const Real py = qy + vy * t;
			if (tx >= ty ? std::fabs(py) > h : std::fabs(px) > w) return NO_HIT;

			// Entry inside a corner square: the rounded corner decides, one circle only.
			if (std::fabs(px) > hw && std::fabs(py) > hh)
			{
				return castCircle(qx - std::copysign(hw, px), qy - std::copysign(hh, py), vx, vy, dd, radius);
			}
			return t;
		}

		// Ray vs the face of an edge offset by 'radius' along its outward normal, limited to the edge span.
		__forceinline Real castEdge(Vec2 q0, Vec2 dl, Vec2 a, Vec2 b, Vec2 normal, Real radius)
		{
			const Real approach = glm::dot(normal, dl);
			if (approach >= Real(0)) return NO_HIT;

			const Real gap = glm::dot(normal, q0 - a) - radius;
			if (gap < Real(0)) return NO_HIT;

			const Real t = gap / -approach;
			if (t > Real(1)) return NO_HIT;

			const Vec2 ab = b - a;
			const Real u = glm::dot(q0 + dl * t - a, ab);
			if (u < Real(0) || u > glm::dot(ab, ab)) return NO_HIT;

			return t;
		}

		Real castPolygon(Vec2 q0, Vec2 dl, Real dd, const PolygonGeometry& g, Real radius)
		{
			const Vec2* vertices = g.vertices();
			const Vec2* normals = g.normals();
			const size_t count = g.size();

			Real t = NO_HIT;
			size_t next = 1;
			for (size_t i = 0; i < count; i++, next++)
			{
				if (next == count) next = 0;
				t = std::fmin(t, castEdge(q0, dl, vertices[i], vertices[next], normals[i], radius));
			}

			// An edge hit is the entry point of a convex shape, so corners matter only without one.
			if (t < NO_HIT) return t;

			for (size_t i = 0; i < count; i++)
			{
				t = std::fmin(t, castCircle(q0.x - vertices[i].x, q0.y - vertices[i].y, dl.x, dl.y, dd, radius));
			}
			return t;
		}

		Real castCapsule(Vec2 q0, Vec2 dl, Real dd, Real halfLength, Real capsuleRadius, Real radius)
		{
			const Real inflated = capsuleRadius + radius;

			// Side slabs y = +-inflated, limited to |x| <= halfLength.
			const auto slab = [&](Real approachSpeed, Real distance) -> Real
				{
					if (approachSpeed >= Real(0)) return NO_HIT;
					const Real gap = distance - inflated;
					if (gap < Real(0)) return NO_HIT;

					const Real time = gap / -approachSpeed;
					if (time > Real(1)) return NO_HIT;
					if (std::fabs(q0.x + dl.x * time) > halfLength) return NO_HIT;
					return time;
				};

			Real t = std::fmin(slab(dl.y, q0.y), slab(-dl.y, -q0.y));

			// A slab hit is the entry point of a convex shape, so the end circles matter only without one.
			if (t < NO_HIT) return t;

			t = castCircle(q0.x + halfLength, q0.y, dl.x, dl.y, dd, inflated);
			return std::fmin(t, castCircle(q0.x - halfLength, q0.y, dl.x, dl.y, dd, inflated));
		}

		// Earliest hit time of the core (world start p0, world displacement d) against static collider 'c'.
		Real castCore(const StaticShapes& s, ObjectDenseIndex colliderIndex, Vec2 start, Vec2 d, Real dd, Real coreRadius)
		{
			// shapes, staticCollider, start, d, dd, coreRadius
			const Real cs = s.cosR[colliderIndex];
			const Real sn = s.sinR[colliderIndex];
			const Real rx = start.x - s.posX[colliderIndex];
			const Real ry = start.y - s.posY[colliderIndex];

			// Into the static collider's local frame (inverse rotation).
			const Vec2 q0 = { cs * rx + sn * ry, cs * ry - sn * rx };
			const Vec2 dl = { cs * d.x + sn * d.y, cs * d.y - sn * d.x };

			const ObjectDenseIndex shape = s.shapeIndex[colliderIndex];
			switch (s.type[colliderIndex])
			{
			case ColliderShapeType::Circle:
				return castCircle(q0.x, q0.y, dl.x, dl.y, dd, s.circleRadius[shape] + coreRadius);
			case ColliderShapeType::Box:
				return castBox(q0, dl, dd, s.boxHalfWidth[shape], s.boxHalfHeight[shape], coreRadius);
			case ColliderShapeType::Capsule:
				return castCapsule(q0, dl, dd, s.capsuleHalfLength[shape], s.capsuleRadius[shape], coreRadius);
			case ColliderShapeType::Polygon:
				return castPolygon(q0, dl, dd, s.polygons[shape], coreRadius);
			}
			return NO_HIT;
		}
	}

	void ContinuousCollisionDetector::snapshotPoses(const BodySoA& bodies, const ColliderSoA& colliders)
	{
		const size_t bodyCount = bodies.getCount();
		if (prevOffsetX.size() < bodyCount)
		{
			prevOffsetX.resize(bodyCount);
			prevOffsetY.resize(bodyCount);
		}

		const IndexRange dynamic = bodies.range(BodyGroup::Dynamic);
		std::copy_n(bodies.offsetX.data() + dynamic.begin, dynamic.size(), prevOffsetX.data() + dynamic.begin);
		std::copy_n(bodies.offsetY.data() + dynamic.begin, dynamic.size(), prevOffsetY.data() + dynamic.begin);

		const size_t colliderCount = colliders.worldPosX.size();
		if (prevColliderX.size() < colliderCount)
		{
			prevColliderX.resize(colliderCount);
			prevColliderY.resize(colliderCount);
		}

		const IndexRange cr = colliders.range(BodyGroup::Dynamic);
		std::copy_n(colliders.worldPosX.data() + cr.begin, cr.size(), prevColliderX.data() + cr.begin);
		std::copy_n(colliders.worldPosY.data() + cr.begin, cr.size(), prevColliderY.data() + cr.begin);
	}

	struct ContinuousCollisionDetector::StaticSweepData
	{
		StaticShapes shapes;
		const BroadPhaseInternal::BvhNode* nodes;
		const ObjectDenseIndex* treeColliders;
	};

	template<typename ShapeSoA>
	void ContinuousCollisionDetector::processShapeRange(
		const ShapeSoA& soa,
		const BodySoA& bodies,
		const ColliderSoA& colliders,
		const StaticSweepData& data
	)
	{
		using Node = BroadPhaseInternal::BvhNode;

		const ObjectDenseIndex* ECSTASY_RESTRICT ownerPtr = colliders.bodyIndex.data();
		const Real* ECSTASY_RESTRICT worldPosXPtr = colliders.worldPosX.data();
		const Real* ECSTASY_RESTRICT worldPosYPtr = colliders.worldPosY.data();
		const Real* ECSTASY_RESTRICT minExtentPtr = colliders.minExtent.data();
		const Real* ECSTASY_RESTRICT aabbMinXPtr = colliders.aabbMinX.data();
		const Real* ECSTASY_RESTRICT aabbMinYPtr = colliders.aabbMinY.data();
		const Real* ECSTASY_RESTRICT aabbMaxXPtr = colliders.aabbMaxX.data();
		const Real* ECSTASY_RESTRICT aabbMaxYPtr = colliders.aabbMaxY.data();

		const Real* ECSTASY_RESTRICT offsetXPtr = bodies.offsetX.data();
		const Real* ECSTASY_RESTRICT offsetYPtr = bodies.offsetY.data();

		const Real* ECSTASY_RESTRICT prevOffsetXPtr = prevOffsetX.data();
		const Real* ECSTASY_RESTRICT prevOffsetYPtr = prevOffsetY.data();
		const Real* ECSTASY_RESTRICT prevColliderXPtr = prevColliderX.data();
		const Real* ECSTASY_RESTRICT prevColliderYPtr = prevColliderY.data();

		const ObjectDenseIndex* ECSTASY_RESTRICT colliderIndexPtr = soa.colliderIndices.data();

		const Node* ECSTASY_RESTRICT nodes = data.nodes;
		const ObjectDenseIndex* ECSTASY_RESTRICT treeColliders = data.treeColliders;
		Real* ECSTASY_RESTRICT hitTimePtr = bodyHitTime.data();

		const Real fastFactor = settings.fastFactor;
		const Real coreFraction = settings.coreFraction;

		const IndexRange shapeRange = soa.range(BodyGroup::Dynamic);
		const ObjectDenseIndex bodyRangeBegin = bodies.range(BodyGroup::Dynamic).begin;

		uint32_t stack[TRAVERSAL_STACK_SIZE];

		for (size_t shapeIndex = shapeRange.begin; shapeIndex < shapeRange.end; shapeIndex++)
		{
			const ObjectDenseIndex colliderIndex = colliderIndexPtr[shapeIndex];
			const ObjectDenseIndex body = ownerPtr[colliderIndex];

			// Cheap reject on the body origin displacement.
			const Real bodyDx = offsetXPtr[body] - prevOffsetXPtr[body];
			const Real bodyDy = offsetYPtr[body] - prevOffsetYPtr[body];
			const Real bodyDeltaSquaredLength = bodyDx * bodyDx + bodyDy * bodyDy;
			if (bodyDeltaSquaredLength <= MIN_DISPLACEMENT_SQ) continue;

			// Min extent of the shape (core circle is centered at the collider origin).
			const Real minExtent = minExtentPtr[colliderIndex];
			if (minExtent <= Real(0)) continue;

			// This check removes most candidates.
			const Real threshold = fastFactor * minExtent;
			const Real thresholdSq = threshold * threshold;
			if (bodyDeltaSquaredLength <= thresholdSq) continue;

			// Previous collider origin: offset + com + R(localOffset - com), with the previous rotation.
			const Vec2 start = { prevColliderXPtr[colliderIndex], prevColliderYPtr[colliderIndex] };
			const Vec2 end = { worldPosXPtr[colliderIndex], worldPosYPtr[colliderIndex] };
			const Vec2 d = end - start;
			const Real dd = glm::dot(d, d);
			if (dd <= thresholdSq) continue;

			// Polygon origin (local 0,0) must lie inside the polygon: dot(n_i, 0 - v_i) <= 0 for all edges.
			if constexpr (std::is_same_v<std::remove_cv_t<ShapeSoA>, PolygonSoA>)
			{
				const PolygonGeometry& g = soa.geometry[shapeIndex];
				const Vec2* vertices = g.vertices();
				const Vec2* normals = g.normals();
				const size_t count = g.size();

				Real minInside = std::numeric_limits<Real>::max();
				for (size_t i = 0; i < count; i++)
				{
					minInside = std::fmin(minInside, glm::dot(normals[i], vertices[i]));
				}
				if (minInside < Real(0)) continue;
			}

			const Real coreRadius = coreFraction * minExtent;

			// Swept AABB of the core.
			const Real sweepMinX = std::fmin(start.x, end.x) - coreRadius;
			const Real sweepMaxX = std::fmax(start.x, end.x) + coreRadius;
			const Real sweepMinY = std::fmin(start.y, end.y) - coreRadius;
			const Real sweepMaxY = std::fmax(start.y, end.y) + coreRadius;

			// Static tree traversal.
			Real bestTime = NO_HIT;
			uint32_t stackSize = 0;
			stack[stackSize++] = 0;

			while (stackSize != 0)
			{
				const Node& node = nodes[stack[--stackSize]];
				if (node.maxX < sweepMinX || node.minX > sweepMaxX || node.maxY < sweepMinY || node.minY > sweepMaxY) continue;

				if (node.leftChildIndex != Node::INVALID_INDEX) // Has children.
				{
					stack[stackSize++] = node.leftChildIndex + 1;
					stack[stackSize++] = node.leftChildIndex;
					continue;
				}

				// Leaf node.
				for (uint32_t k = node.start; k < node.end; k++)
				{
					const ObjectDenseIndex staticCollider = treeColliders[k];
					if (aabbMaxXPtr[staticCollider] < sweepMinX || aabbMinXPtr[staticCollider] > sweepMaxX ||
						aabbMaxYPtr[staticCollider] < sweepMinY || aabbMinYPtr[staticCollider] > sweepMaxY) continue;

					bestTime = std::fmin(bestTime, castCore(data.shapes, staticCollider, start, d, dd, coreRadius));
				}
			}

			if (bestTime < Real(1))
			{
				Real& t = hitTimePtr[body - bodyRangeBegin];
				if (t >= Real(1)) hitBodies.push_back(body); // First hit of this body.
				t = std::fmin(t, bestTime);
			}
		}
	}

	bool ContinuousCollisionDetector::resolve(
		BodySoA& bodies,
		ColliderSoA& colliders,
		const CircleSoA& circles,
		const BoxSoA& boxes,
		const PolygonSoA& polygons,
		const CapsuleSoA& capsules,
		const BroadPhaseInternal::BvhTree& staticTree
	)
	{
		TRACY_SCOPE_N("Continuous collision");

		if (staticTree.empty()) return false;

		const IndexRange colliderRange = colliders.range(BodyGroup::Dynamic);
		if (colliderRange.empty()) return false;

		const IndexRange bodyRange = bodies.range(BodyGroup::Dynamic);

		const StaticShapes shapes = {
			colliders.worldPosX.data(), colliders.worldPosY.data(),
			colliders.worldRotationCos.data(), colliders.worldRotationSin.data(),
			colliders.shapeType.data(), colliders.shapeIndex.data(),
			circles.radius.data(),
			boxes.halfWidth.data(), boxes.halfHeight.data(),
			capsules.halfLength.data(), capsules.radius.data(),
			polygons.geometry.data()
		};

		const StaticSweepData data = { shapes, staticTree.nodeData(), staticTree.colliderIndexData() };

		hitBodies.clear();
		if (bodyHitTime.size() < bodyRange.size()) bodyHitTime.resize(bodyRange.size(), Real(1));

		// TODO: Try rewriting in stages to use SIMD.
		// TODO: Pass bestTime into castCore and the cast functions, and early-out when gap / approach already exceeds it. This also skips sqrt calls.
		// TODO: In tree traversal, replace the swept - AABB node test with a ray - vs - AABB slab test(nodes inflated by coreRadius).Diagonal fast movers get a much tighter sweep than the AABB.Also prune nodes whose entry time is greater than bestTime, and push the nearer child last so it is visited first.
		// TODO: Polygon: replace the per - edge and per - vertex casts with half - plane clipping(tEnter / tExit), and do the vertex circles only for corner hits.

		processShapeRange(circles, bodies, colliders, data);
		processShapeRange(boxes, bodies, colliders, data);
		processShapeRange(polygons, bodies, colliders, data);
		processShapeRange(capsules, bodies, colliders, data);

		if (hitBodies.empty()) return false;

		// Move bodies back along their own displacement. Translation only: rotation (cos/sin) stays valid.
		for (const ObjectDenseIndex body : hitBodies)
		{
			const ObjectDenseIndex bodyLocalIndex = body - bodyRange.begin;

			const Real back = bodyHitTime[bodyLocalIndex] - Real(1);
			bodyHitTime[bodyLocalIndex] = Real(1); // Restore invariant.

			const Real shiftX = (bodies.offsetX[body] - prevOffsetX[body]) * back;
			const Real shiftY = (bodies.offsetY[body] - prevOffsetY[body]) * back;

			bodies.offsetX[body] += shiftX;
			bodies.offsetY[body] += shiftY;
			bodies.worldCenterX[body] += shiftX;
			bodies.worldCenterY[body] += shiftY;

			for (const ObjectDenseIndex collider : bodies.colliderIndices[body])
			{
				colliders.worldPosX[collider] += shiftX;
				colliders.worldPosY[collider] += shiftY;
			}
		}

		return true;
	}

	size_t ContinuousCollisionDetector::getMemoryUsage() const
	{
		return
			getVectorMemoryUsage(prevOffsetX) +
			getVectorMemoryUsage(prevOffsetY) +
			getVectorMemoryUsage(prevColliderX) +
			getVectorMemoryUsage(prevColliderY) +
			getVectorMemoryUsage(hitBodies) +
			getVectorMemoryUsage(bodyHitTime);
	}
}