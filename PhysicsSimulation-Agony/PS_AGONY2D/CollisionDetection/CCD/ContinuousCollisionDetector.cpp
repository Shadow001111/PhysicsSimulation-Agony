#include "ContinuousCollisionDetector.h"

#include "PS_AGONY2D/Profiling/TracyProfiler.h"
#include "PS_AGONY2D/Limits.h"

#include "Ecstasy/Core/Portablity.h"
#include "Ecstasy/Core/Simd.h"

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

		__forceinline Real castCircle(Real mx, Real my, Real dx, Real dy, Real dd, Real radius, Real best)
		{
			const Real c = mx * mx + my * my - radius * radius;
			const Real b = mx * dx + my * dy;
			if (c <= Real(0) || b >= Real(0)) return NO_HIT; // Inside or moving away.

			// t >= c / (-2b). Reject before sqrt.
			if (c >= -Real(2) * b * best) return NO_HIT;

			const Real disc = b * b - dd * c;
			if (disc < Real(0)) return NO_HIT;

			return c / (std::sqrt(disc) - b);
		}

		Real castBox(Vec2 q0, Vec2 dl, Real dd, Real hw, Real hh, Real radius, Real best)
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

			// Slab entry time per axis: 0 when already between its planes, never when moving away or not before 'best'.
			Real tx = Real(0);
			if (gapX > Real(0))
			{
				if (vx >= Real(0) || gapX >= best * -vx) return NO_HIT;
				tx = gapX / -vx;
			}
			Real ty = Real(0);
			if (gapY > Real(0))
			{
				if (vy >= Real(0) || gapY >= best * -vy) return NO_HIT;
				ty = gapY / -vy;
			}

			const Real t = std::fmax(tx, ty); // Already < best.

			// Entry point. Only the axis that did not set 't' needs a range check.
			const Real px = qx + vx * t;
			const Real py = qy + vy * t;
			if (tx >= ty ? std::fabs(py) > h : std::fabs(px) > w) return NO_HIT;

			// Entry inside a corner square: the rounded corner decides, one circle only.
			if (std::fabs(px) > hw && std::fabs(py) > hh)
			{
				return castCircle(qx - std::copysign(hw, px), qy - std::copysign(hh, py), vx, vy, dd, radius, best);
			}
			return t;
		}

		// Ray vs the face of an edge offset by 'radius' along its outward normal, limited to the edge span.
		__forceinline Real castEdge(Vec2 q0, Vec2 dl, Vec2 a, Vec2 b, Vec2 normal, Real radius, Real best)
		{
			const Real approach = glm::dot(normal, dl);
			if (approach >= Real(0)) return NO_HIT;

			const Real gap = glm::dot(normal, q0 - a) - radius;
			if (gap < Real(0)) return NO_HIT;
			if (gap >= best * -approach) return NO_HIT; // t >= best, no division.

			const Real t = gap / -approach;

			const Vec2 ab = b - a;
			const Real u = glm::dot(q0 + dl * t - a, ab);
			if (u < Real(0) || u > glm::dot(ab, ab)) return NO_HIT;

			return t;
		}

		// Convex polygon inflated by 'radius': intersect the offset half-planes (tEnter / tExit), then let the rounded corner decide.
		// Returns the time of impact < best, otherwise NO_HIT.
		Real castPolygon(Vec2 q0, Vec2 dl, Real dd, const PolygonGeometry& g, Real radius, Real best)
		{
			const Vec2* vertices = g.vertices();
			const Vec2* normals = g.normals();
			const size_t count = g.size();

			Real tEnter = Real(0);
			Real tExit = best;
			size_t enterEdge = 0;

			alignas(64) Real gapArray[MAX_POLYGON_VERTEX_COUNT];

			for (size_t i = 0; i < count; i++)
			{
				const Vec2 n = normals[i];
				const Real speed = glm::dot(n, dl); // < 0: entering the plane, > 0: leaving it.
				const Real gap = glm::dot(n, q0 - vertices[i]) - radius; // > 0: outside.

				gapArray[i] = gap;

				if (speed < Real(0))
				{
					if (gap > Real(0))
					{
						// gap = t * approach
						// if (t * approach >= tExit * approach)
						// if (t >= tExit)

						const Real approach = -speed; // Positive, bigger than zero.
						if (gap >= tExit * approach) // gap / approach >= tExit, no division.
						{
							return NO_HIT;
						}
						const Real t = gap / approach;
						if (t > tEnter)
						{
							tEnter = t;
							enterEdge = i;
						}
					}
				}
				else if (gap > Real(0))
				{
					return NO_HIT;  // Outside and not approaching.
				}
				else if (speed > Real(0))
				{
					tExit = std::fmin(tExit, -gap / speed);
				}
			}

			if (tEnter >= tExit)
			{
				return NO_HIT;
			}

			// Edge that owns the entry point.
			size_t edge = enterEdge;
			const bool startedInside = tEnter == Real(0);
			if (startedInside)
			{
				// Start inside the plane polygon: the nearest plane decides (inside the shape or in a corner zone).
				Real maxGap = std::numeric_limits<Real>::lowest();
				for (size_t i = 0; i < count; i++)
				{
					const Real gap = gapArray[i];
					if (gap > maxGap)
					{
						maxGap = gap;
						edge = i;
					}
				}
			}

			const Vec2 p = q0 + dl * tEnter;

			const size_t next = edge + 1 == count ? 0 : edge + 1;
			const Vec2 a = vertices[edge];
			const Vec2 b = vertices[next];
			const Vec2 ab = b - a;
			const Real u = glm::dot(p - a, ab);

			// Entry within the edge span: a face hit. A start there is already inside the shape.
			if (u >= Real(0) && u <= glm::dot(ab, ab))
			{
				return startedInside ? NO_HIT : tEnter;
			}

			// Entry beyond the span: the rounded corner decides, one circle only.
			const Vec2 corner = u < Real(0) ? a : b;
			return castCircle(q0.x - corner.x, q0.y - corner.y, dl.x, dl.y, dd, radius, tExit);
		}

		Real castCapsule(Vec2 q0, Vec2 dl, Real dd, Real halfLength, Real capsuleRadius, Real radius, Real best)
		{
			const Real inflated = capsuleRadius + radius;

			// Slab on q0's side.
			const Real nvy = -std::copysign(Real(1), q0.y) * dl.y;
			const Real gap = std::fabs(q0.y) - inflated;

			// gap < best * -vy guarantees time < best.
			if (nvy > Real(0) && gap >= Real(0) && gap < best * nvy)
			{
				const Real time = gap / nvy;
				if (std::fabs(q0.x + dl.x * time) <= halfLength) return time; // Convex entry point.
			}

			// castCircle may return a time above best, so clamp.
			const Real t = std::fmin(castCircle(q0.x + halfLength, q0.y, dl.x, dl.y, dd, inflated, best), best);
			return std::fmin(castCircle(q0.x - halfLength, q0.y, dl.x, dl.y, dd, inflated, t), t);
		}

		// Earliest hit time of the core against static collider 'colliderIndex'. Only hits earlier than 'best' matter.
		Real castCore(const StaticShapes& s, ObjectDenseIndex colliderIndex, Vec2 start, Vec2 d, Real dd, Real coreRadius, Real best)
		{
			const Real cs = s.cosR[colliderIndex];
			const Real sn = s.sinR[colliderIndex];
			const Real rx = start.x - s.posX[colliderIndex];
			const Real ry = start.y - s.posY[colliderIndex];

			// Into the static collider's local frame (inverse rotation).
			const Vec2 q0 = { cs * rx + sn * ry, cs * ry - sn * rx };
			const Vec2 dl = { cs * d.x + sn * d.y, cs * d.y - sn * d.x };

			const ObjectDenseIndex shape = s.shapeIndex[colliderIndex];
			const ColliderShapeType type = s.type[colliderIndex];
			Real result = NO_HIT;
			switch (type)
			{
			case ColliderShapeType::Circle:
				result = castCircle(q0.x, q0.y, dl.x, dl.y, dd, s.circleRadius[shape] + coreRadius, best);
				break;
			case ColliderShapeType::Box:
				result = castBox(q0, dl, dd, s.boxHalfWidth[shape], s.boxHalfHeight[shape], coreRadius, best);
				break;
			case ColliderShapeType::Polygon:
				result = castPolygon(q0, dl, dd, s.polygons[shape], coreRadius, best);
				break;
			case ColliderShapeType::Capsule:
				result = castCapsule(q0, dl, dd, s.capsuleHalfLength[shape], s.capsuleRadius[shape], coreRadius, best);
				break;
			}
			return result;
		}
	}

	void ContinuousCollisionDetector::snapshotPoses(const BodySoA& bodies, const ColliderSoA& colliders)
	{
		const size_t bodyCount = bodies.getCount();
		if (prevComX.size() < bodyCount)
		{
			prevComX.resize(bodyCount);
			prevComY.resize(bodyCount);
		}

		const IndexRange dynamic = bodies.range(BodyGroup::Dynamic);
		std::copy_n(bodies.comX.data() + dynamic.begin, dynamic.size(), prevComX.data() + dynamic.begin);
		std::copy_n(bodies.comY.data() + dynamic.begin, dynamic.size(), prevComY.data() + dynamic.begin);

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
		BroadPhaseInternal::BvhTreeView treeView;
	};

	void ContinuousCollisionDetector::processShapeRange(
		IndexRange shapeRange,
		const ObjectDenseIndex* ECSTASY_RESTRICT colliderIndexPtr,
		const BodySoA& bodies,
		const ColliderSoA& colliders,
		const StaticSweepData& data
	)
	{
		using Node = BroadPhaseInternal::BvhNode;
		using RealSimd = Ecstasy::Core::Simd<Real>;

		const ObjectDenseIndex* ECSTASY_RESTRICT ownerPtr = colliders.bodyIndex.data();
		const Real* ECSTASY_RESTRICT worldPosXPtr = colliders.worldPosX.data();
		const Real* ECSTASY_RESTRICT worldPosYPtr = colliders.worldPosY.data();
		const Real* ECSTASY_RESTRICT minExtentPtr = colliders.minExtent.data();

		const Real* ECSTASY_RESTRICT comXPtr = bodies.comX.data();
		const Real* ECSTASY_RESTRICT comYPtr = bodies.comY.data();

		const Real* ECSTASY_RESTRICT prevComXPtr = prevComX.data();
		const Real* ECSTASY_RESTRICT prevComYPtr = prevComY.data();
		const Real* ECSTASY_RESTRICT prevColliderXPtr = prevColliderX.data();
		const Real* ECSTASY_RESTRICT prevColliderYPtr = prevColliderY.data();

		const Node* ECSTASY_RESTRICT nodes = data.treeView.nodes;
		const ObjectDenseIndex* ECSTASY_RESTRICT treeColliders = data.treeView.indices;

		const ObjectDenseIndex bodyRangeBegin = bodies.range(BodyGroup::Dynamic).begin;
		Real* ECSTASY_RESTRICT shiftedHitTimePtr = bodyHitTime.data() - bodyRangeBegin;

		const Real fastFactor = settings.fastFactor;
		const Real coreFraction = settings.coreFraction;

		uint32_t stack[TRAVERSAL_STACK_SIZE];

		for (size_t shapeIndex = shapeRange.begin; shapeIndex < shapeRange.end; shapeIndex++)
		{
			const ObjectDenseIndex colliderIndex = colliderIndexPtr[shapeIndex];
			const ObjectDenseIndex body = ownerPtr[colliderIndex];

			// Cheap reject on the body origin displacement.
			const Real bodyDx = comXPtr[body] - prevComXPtr[body];
			const Real bodyDy = comYPtr[body] - prevComYPtr[body];
			const Real bodyDeltaSquaredLength = bodyDx * bodyDx + bodyDy * bodyDy;
			if (bodyDeltaSquaredLength <= MIN_DISPLACEMENT_SQ) continue;

			// Min extent of the shape (core circle is centered at the collider origin).
			const Real minExtent = minExtentPtr[colliderIndex];
			if (minExtent <= Real(0)) continue;

			// This check removes most candidates.
			const Real threshold = fastFactor * minExtent;
			const Real thresholdSq = threshold * threshold;
			if (bodyDeltaSquaredLength <= thresholdSq) continue;

			// Previous collider origin: previous COM + R(previous rotation) * comOffset.
			const Vec2 start = { prevColliderXPtr[colliderIndex], prevColliderYPtr[colliderIndex] };
			const Vec2 end = { worldPosXPtr[colliderIndex], worldPosYPtr[colliderIndex] };
			const Vec2 d = end - start;
			const Real dd = glm::dot(d, d);
			if (dd <= thresholdSq) continue;

			// No polygon origin-inside check needed: minExtent is the polygon in-radius about the collider origin,
			// which is 0 when the origin is outside (skipped above).

			const Real coreRadius = coreFraction * minExtent;

			// Swept AABB of the core.
			const Real sweepMinX = std::fmin(start.x, end.x) - coreRadius;
			const Real sweepMaxX = std::fmax(start.x, end.x) + coreRadius;
			const Real sweepMinY = std::fmin(start.y, end.y) - coreRadius;
			const Real sweepMaxY = std::fmax(start.y, end.y) + coreRadius;

			const RealSimd vSweepMinX(sweepMinX);
			const RealSimd vSweepMaxX(sweepMaxX);
			const RealSimd vSweepMinY(sweepMinY);
			const RealSimd vSweepMaxY(sweepMaxY);

			// Static tree traversal.
			Real bestTime = 1;
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
				const uint32_t srcIndex = node.leafIndex * Node::KD_LEAF_SIZE;
				const Real* ECSTASY_RESTRICT leafMinXPtr = data.treeView.leafMinX + srcIndex;
				const Real* ECSTASY_RESTRICT leafMinYPtr = data.treeView.leafMinY + srcIndex;
				const Real* ECSTASY_RESTRICT leafMaxXPtr = data.treeView.leafMaxX + srcIndex;
				const Real* ECSTASY_RESTRICT leafMaxYPtr = data.treeView.leafMaxY + srcIndex;

				uint32_t mask = 0;
				for (uint32_t j = 0; j < Node::KD_LEAF_SIZE; j += RealSimd::lanes)
				{
					const RealSimd vMinX = RealSimd::load(leafMinXPtr + j);
					const RealSimd vMaxX = RealSimd::load(leafMaxXPtr + j);
					const RealSimd vMinY = RealSimd::load(leafMinYPtr + j);
					const RealSimd vMaxY = RealSimd::load(leafMaxYPtr + j);

					const RealSimd overlap =
						(vSweepMinX < vMaxX) & (vSweepMaxX > vMinX) &
						(vSweepMinY < vMaxY) & (vSweepMaxY > vMinY);

					mask |= overlap.movemask() << j;
				}
				while (mask)
				{
					const uint32_t lane = std::countr_zero(mask);
					mask &= mask - 1;

					const ObjectDenseIndex staticCollider = treeColliders[node.start + lane];
					bestTime = std::fmin(bestTime, castCore(data.shapes, staticCollider, start, d, dd, coreRadius, bestTime));
				}
			}

			if (bestTime < Real(1))
			{
				Real& t = shiftedHitTimePtr[body];
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
		if (colliders.range(BodyGroup::Dynamic).empty()) return false;

		const StaticShapes shapes = {
			colliders.worldPosX.data(), colliders.worldPosY.data(),
			colliders.worldRotationCos.data(), colliders.worldRotationSin.data(),
			colliders.shapeType.data(), colliders.shapeIndex.data(),
			circles.radius.data(),
			boxes.halfWidth.data(), boxes.halfHeight.data(),
			capsules.halfLength.data(), capsules.radius.data(),
			polygons.geometry.data()
		};

		const StaticSweepData data = { shapes, staticTree.view() };

		hitBodies.clear();
		const IndexRange bodyRange = bodies.range(BodyGroup::Dynamic);
		if (bodyHitTime.size() < bodyRange.size()) bodyHitTime.resize(bodyRange.size(), Real(1));

		// TODO: Try rewriting in stages to use SIMD.
		// TODO: In tree traversal, replace the swept - AABB node test with a ray - vs - AABB slab test(nodes inflated by coreRadius).
		//		 Diagonal fast movers get a much tighter sweep than the AABB.
		//		 Also prune nodes whose entry time is greater than bestTime, and push the nearer child last so it is visited first.
		//	     (Tried that, but didn't work out).

		{
			#define PASS(temp) temp.range(BodyGroup::Dynamic), temp.colliderIndices.data(), bodies, colliders, data
			{
				TRACY_SCOPE_N("CCD: Circles");
				processShapeRange(PASS(circles));
			}
			{
				TRACY_SCOPE_N("CCD: Boxes");
				processShapeRange(PASS(boxes));
			}
			{
				TRACY_SCOPE_N("CCD: Polygons");
				processShapeRange(PASS(polygons));
			}
			{
				TRACY_SCOPE_N("CCD: Capsules");
				processShapeRange(PASS(capsules));
			}
			#undef PASS
		}

		if (hitBodies.empty()) return false;

		// Move bodies back along their own displacement. Translation only: rotation (cos/sin) stays valid.
		Real* ECSTASY_RESTRICT shiftedHitTimePtr = bodyHitTime.data() - bodyRange.begin;
		Real* ECSTASY_RESTRICT comXPtr = bodies.comX.data();
		Real* ECSTASY_RESTRICT comYPtr = bodies.comY.data();
		Real* ECSTASY_RESTRICT worldPosXPtr = colliders.worldPosX.data();
		Real* ECSTASY_RESTRICT worldPosYPtr = colliders.worldPosY.data();
		const Real* ECSTASY_RESTRICT prevComXPtr = prevComX.data();
		const Real* ECSTASY_RESTRICT prevComYPtr = prevComY.data();
		{
			TRACY_SCOPE_N("CCD: Shift hit bodies");
			for (const ObjectDenseIndex body : hitBodies)
			{
				const Real back = shiftedHitTimePtr[body] - Real(1);
				shiftedHitTimePtr[body] = Real(1); // Restore invariant.

				const Real shiftX = (comXPtr[body] - prevComXPtr[body]) * back;
				const Real shiftY = (comYPtr[body] - prevComYPtr[body]) * back;

				comXPtr[body] += shiftX;
				comYPtr[body] += shiftY;

				for (const ObjectDenseIndex collider : bodies.colliderIndices[body])
				{
					worldPosXPtr[collider] += shiftX;
					worldPosYPtr[collider] += shiftY;
				}
			}
		}
		return true;
	}

	size_t ContinuousCollisionDetector::getMemoryUsage() const
	{
		return
			getVectorMemoryUsage(prevComX) +
			getVectorMemoryUsage(prevComY) +
			getVectorMemoryUsage(prevColliderX) +
			getVectorMemoryUsage(prevColliderY) +
			getVectorMemoryUsage(hitBodies) +
			getVectorMemoryUsage(bodyHitTime);
	}
}