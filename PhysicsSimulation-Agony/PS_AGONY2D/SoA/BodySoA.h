#pragma once
#include "PS_AGONY2D/ContainerUtilities.h"
#include "PS_AGONY2D/HandleTable.h"
#include "PS_AGONY2D/Types/Types.h"

#include <cstdint>
#include <vector>
#include <optional>

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
    F(sleepingTimer, float) \
    F(islandId, ObjectDenseIndex) \
    F(sleepIslandId, ObjectDenseIndex)

// Define complex vector-of-vector fields.
#define PS_AGONY_BODY_SOA_VECTOR_FIELDS(F) \
    F(attachments, BodyAttachment) \
    F(colliderIndices, ObjectDenseIndex)

namespace PS_AGONY2D
{
    // Memory order of bodies is always follows the enum order. (Pls, keep unused always at the back).
    enum class BodyGroup : uint8_t { Dynamic, Static, Sleeping, Unused };
    //enum class BodyGroup : uint8_t { Static, Dynamic, Sleeping, Unused };

    struct IndexRange
    {
        ObjectDenseIndex begin = 0;
        ObjectDenseIndex end = 0;
        ObjectDenseIndex size() const noexcept { return end - begin; }
        bool empty() const noexcept { return begin == end; }
    };

    struct BodyData
    {
        Vec2 offset;
        Vec2 localCenterOfMass;
        Vec2 worldCenter;
        Vec2 velocity;
        Real rotation;
        Real angularVelocity;
        Real mass, invMass;
        Real inertia, invInertia;
        Real rotationCos, rotationSin;
        Handle handle;
        // TODO: If user will be able to change body data, rotation cache must be kept in sync with changes.
        // We can compute sine and cosine everytime, but better only recompute when dirty on demand.
    };

    struct BodySoA
    {
        #define DECLARE_FIELD(name, type) CacheLineAlignedVector<type> name;
        PS_AGONY_BODY_SOA_SCALAR_FIELDS(DECLARE_FIELD)
        #undef DECLARE_FIELD

        #define DECLARE_VECTOR_FIELD(name, innerType) CacheLineAlignedVector<std::vector<innerType>> name;
        PS_AGONY_BODY_SOA_VECTOR_FIELDS(DECLARE_VECTOR_FIELD)
        #undef DECLARE_VECTOR_FIELD

        HandleTable handles;

        // Changed by ObjectManager::moveBodyToGroup only.
        uint32_t groupCounts[size_t(BodyGroup::Unused)]{};
        uint32_t groupStarts[size_t(BodyGroup::Unused) + 1]{};

        IndexRange range(BodyGroup group) const noexcept
        {
            const size_t index = static_cast<size_t>(group);
            if (index < size_t(BodyGroup::Unused)) [[likely]]
            {
                return { groupStarts[index], groupStarts[index + 1] };
            }
            return { groupStarts[size_t(BodyGroup::Unused)], static_cast<ObjectDenseIndex>(getCount()) };
        }

        IndexRange liveRange() const noexcept
        {
            return { 0, groupStarts[size_t(BodyGroup::Unused)]};
        }

        BodyGroup groupOf(ObjectDenseIndex index) const noexcept
        {
            for (size_t i = 0; i < size_t(BodyGroup::Unused); i++)
            {
                if (index >= groupStarts[i] && index < groupStarts[i + 1])
                {
                    return static_cast<BodyGroup>(i);
                }
            }
            return BodyGroup::Unused;
        }

        // Swaps two bodies in every field (handles included). Does not touch groups or external references.
        void swap(ObjectDenseIndex a, ObjectDenseIndex b);

        ObjectDenseIndex append(
            Vec2 pos, Vec2 vel, Real rot, Real anglVel,
            Real mass, Real invMass, Real inertia, Real invInertia,
            Vec2 localCenterOfMass
        );

        void swapWithBack(ObjectDenseIndex index);
        void popBack();

        void addAttachment(ObjectDenseIndex bodyIndex, ConstraintType type, ObjectDenseIndex objectIndex);
        void removeAttachment(ObjectDenseIndex bodyIndex, ConstraintType type, ObjectDenseIndex objectIndex);

        void addCollider(ObjectDenseIndex bodyIndex, ObjectDenseIndex colliderIndex);
        void removeCollider(ObjectDenseIndex bodyIndex, ObjectDenseIndex colliderIndex);

        bool isValid(Handle h) const noexcept { return handles.isValid(h); }
        ObjectDenseIndex indexOf(Handle h) const noexcept { return handles.indexOf(h); }
        ObjectDenseIndex indexOfUnchecked(Handle h) const noexcept { return handles.indexOfUnchecked(h); }
        Handle handleAt(ObjectDenseIndex index) const noexcept { return handles.handleAt(index); }

        std::optional<BodyData> getBodyData(ObjectDenseIndex index) const noexcept;

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