#pragma once
#include "PS_AGONY2D/ContainerUtilities.h"
#include "PS_AGONY2D/HandleTable.h"
#include "PS_AGONY2D/Types/Types.h"

#include <cstdint>
#include <vector>

// Define all scalar SoA fields.
#define PS_AGONY_BODY_SOA_SCALAR_FIELDS(F) \
    F(renderOldOffsetX, Real) \
    F(renderOldOffsetY, Real) \
    F(renderOldRotation, Real) \
    F(renderRotationWrapCount, Real) \
    F(offsetX, Real) \
    F(offsetY, Real) \
    F(localCenterOfMassX, Real) \
    F(localCenterOfMassY, Real) \
    F(worldCenterX, Real) \
    F(worldCenterY, Real) \
    F(velocityX, Real) \
    F(velocityY, Real) \
    F(rotation, Real) \
    F(angularVelocity, Real) \
    F(mass, Real) \
    F(invMass, Real) \
    F(inertia, Real) \
    F(invInertia, Real) \
    F(rotationCos, Real) \
    F(rotationSin, Real) \
    F(isStatic, uint8_t)

// Define complex vector-of-vector fields.
#define PS_AGONY_BODY_SOA_VECTOR_FIELDS(F) \
    F(attachments, BodyAttachment) \
    F(colliderIndices, ColliderDenseIndex)

namespace PS_AGONY2D
{
    struct BodySoA
    {
        #define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
        PS_AGONY_BODY_SOA_SCALAR_FIELDS(DECLARE_FIELD)
            #undef DECLARE_FIELD

            #define DECLARE_VECTOR_FIELD(name, innerType) CacheLineAlignedVector<std::vector<innerType>> name;
            PS_AGONY_BODY_SOA_VECTOR_FIELDS(DECLARE_VECTOR_FIELD)
            #undef DECLARE_VECTOR_FIELD

            HandleTable handles;

        Handle append(
            Vec2 pos, Vec2 vel, Real rot, Real anglVel,
            Real mass, Real invMass, Real inertia, Real invInertia,
            Vec2 localCenterOfMass
        );

        void swapWithBack(size_t index);
        void popBack();

        void addAttachment(size_t bodyIndex, ConstraintType type, uint32_t objectIndex);
        void removeAttachment(size_t bodyIndex, ConstraintType type, uint32_t objectIndex);

        void addCollider(size_t bodyIndex, ColliderDenseIndex colliderIndex);
        void removeCollider(size_t bodyIndex, ColliderDenseIndex colliderIndex);

        bool isValid(Handle h) const noexcept { return handles.isValid(h); }
        uint32_t indexOf(Handle h) const noexcept { return handles.indexOf(h); }
        uint32_t indexOfUnchecked(Handle h) const noexcept { return handles.indexOfUnchecked(h); }
        Handle handleAt(size_t index) const noexcept { return handles.handleAt(index); }

        size_t getCount() const noexcept { return offsetX.size(); }
        size_t getMemoryUsage() const noexcept;
    };

    class BodySoAViewer
    {
        size_t count = 0;
    public:
        #define DECLARE_VIEWER_FIELD(name, type) const type* name = nullptr;
        PS_AGONY_BODY_SOA_SCALAR_FIELDS(DECLARE_VIEWER_FIELD)
            #undef DECLARE_VIEWER_FIELD

            #define DECLARE_VIEWER_VECTOR_FIELD(name, innerType) const std::vector<innerType>* name = nullptr;
            PS_AGONY_BODY_SOA_VECTOR_FIELDS(DECLARE_VIEWER_VECTOR_FIELD)
            #undef DECLARE_VIEWER_VECTOR_FIELD

            BodySoAViewer() = default;
        explicit BodySoAViewer(const BodySoA& data) noexcept;

        size_t getCount() const noexcept { return count; }
    };
}