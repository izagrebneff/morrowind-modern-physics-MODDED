#ifndef OPENMW_MWPHYSICS_LIMBSEVERING_H
#define OPENMW_MWPHYSICS_LIMBSEVERING_H

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <LinearMath/btTransform.h>

namespace MWPhysics::LimbSevering
{
    inline constexpr unsigned LeftLeg = 1, RightLeg = 2, LeftArm = 4, RightArm = 8;
    inline constexpr std::size_t PartCount = 11;
    inline constexpr std::size_t PoseBytes = 8 + PartCount * 7 * sizeof(float);
    inline constexpr btScalar MaximumOffset = 131072;
    inline constexpr btScalar SeparationSpeed = 40;
    inline constexpr std::array<std::size_t, 4> Roots{ 8, 10, 3, 5 };

    inline bool validMask(unsigned mask) { return mask != 0 && (mask & ~15u) == 0; }

    // All part indexes match the native eleven-body ragdoll. Shoulder fragments
    // retain their elbow; knees release a calf whose shape already owns the foot.
    inline unsigned fragment(std::size_t part, unsigned mask)
    {
        if ((mask & LeftLeg) && part == 8) return 1;
        if ((mask & RightLeg) && part == 10) return 2;
        if ((mask & LeftArm) && (part == 3 || part == 4)) return 3;
        if ((mask & RightArm) && (part == 5 || part == 6)) return 4;
        return 0;
    }

    inline btVector3 separationVelocity(const btVector3& direction)
    {
        if (!std::isfinite(direction.x()) || !std::isfinite(direction.y()) || !std::isfinite(direction.z())
            || !std::isfinite(direction.length2()) || direction.length2() < btScalar(1e-12))
            return btVector3(0, 0, 0);
        return direction.normalized() * SeparationSpeed;
    }

    inline bool finiteRigid(const btTransform& transform)
    {
        const auto& p = transform.getOrigin();
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) return false;
        const auto& basis = transform.getBasis();
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
                if (!std::isfinite(basis[row][column])) return false;
            if (btFabs(basis[row].length2() - 1) > btScalar(.001)) return false;
            for (int other = 0; other < row; ++other)
                if (btFabs(basis[row].dot(basis[other])) > btScalar(.001)) return false;
        }
        return btFabs(basis.determinant() - 1) <= btScalar(.001);
    }

    inline void appendUnsigned(std::string& out, std::uint32_t value)
    {
        for (unsigned shift = 0; shift < 32; shift += 8)
            out.push_back(static_cast<char>((value >> shift) & 0xff));
    }

    inline std::string encodePose(unsigned mask, const std::array<btTransform, PartCount>& world,
        const btTransform& actorFrame)
    {
        if (!validMask(mask) || !finiteRigid(actorFrame)) return {};
        std::string bytes("LSP1");
        bytes.reserve(PoseBytes);
        appendUnsigned(bytes, mask);
        for (const auto& transform : world)
        {
            if (!finiteRigid(transform)) return {};
            const auto relative = actorFrame.inverse() * transform;
            const auto& p = relative.getOrigin();
            if (btFabs(p.x()) > MaximumOffset || btFabs(p.y()) > MaximumOffset || btFabs(p.z()) > MaximumOffset)
                return {};
            auto q = relative.getRotation();
            q.normalize();
            for (btScalar value : { p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w() })
                appendUnsigned(bytes, std::bit_cast<std::uint32_t>(static_cast<float>(value)));
        }
        return bytes;
    }

    inline std::optional<std::array<btTransform, PartCount>> decodePose(
        std::string_view bytes, unsigned mask, const btTransform& actorFrame)
    {
        if (bytes.size() != PoseBytes || bytes.substr(0, 4) != "LSP1" || !validMask(mask)
            || !finiteRigid(actorFrame)) return {};
        std::size_t cursor = 4;
        const auto readUnsigned = [&]() {
            std::uint32_t bits = 0;
            for (unsigned shift = 0; shift < 32; shift += 8)
                bits |= std::uint32_t(static_cast<unsigned char>(bytes[cursor++])) << shift;
            return bits;
        };
        if (readUnsigned() != mask) return {};
        const auto read = [&]() { return std::bit_cast<float>(readUnsigned()); };
        std::array<btTransform, PartCount> world;
        for (auto& transform : world)
        {
            const float x = read(), y = read(), z = read();
            const float qx = read(), qy = read(), qz = read(), qw = read();
            const btVector3 p(x, y, z);
            btQuaternion q(qx, qy, qz, qw);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)
                || btFabs(p.x()) > MaximumOffset || btFabs(p.y()) > MaximumOffset || btFabs(p.z()) > MaximumOffset
                || !std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw)
                || !std::isfinite(q.length2()) || btFabs(q.length2() - 1) > btScalar(.001)) return {};
            q.normalize();
            transform = actorFrame * btTransform(q, p);
            if (!finiteRigid(transform)) return {};
        }
        return world;
    }
}
#endif
