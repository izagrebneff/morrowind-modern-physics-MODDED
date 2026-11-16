#include "npcblood.hpp"
#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwmechanics/actorutil.hpp"
#include "../mwworld/class.hpp"
#include "npcanimation.hpp"
#include "vismask.hpp"
#include <chrono>
#include <components/debug/debuglog.hpp>
#include <components/esm3/loadarmo.hpp>
#include <components/resource/imagemanager.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/sceneutil/decalprojector.hpp>
#include <components/sceneutil/depth.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/sceneutil/texturetype.hpp>
#include <components/vfs/manager.hpp>
#include <cstring>
#include <limits>
#include <map>
#include <osg/BlendFunc>
#include <osg/Image>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <osg/ColorMask>
#include <osg/PolygonOffset>
#include <osg/TexMat>
#include <osg/Texture2D>
#include <osg/TriangleIndexFunctor>
#include <osgUtil/CullVisitor>

namespace MWRender
{
    namespace
    {
        using Anchors = std::vector<SceneUtil::SurfaceDecals::Anchor>;
        constexpr std::size_t MaxWounds = 24, MaxActorVertices = 8192, MaxSaveBytes = 400000;
        const char* const Styles[]{ "slash", "small", "big", "tiny" };

        // Ordinary Geometry is often shared between actors. Replace only this
        // actor's parent edge, keeping source arrays/materials immutable.
        class BloodMarkedGeometry final : public osg::Geometry
        {
        public:
            BloodMarkedGeometry() = default;
            explicit BloodMarkedGeometry(const osg::Geometry& source)
                : osg::Geometry(source, osg::CopyOp::SHALLOW_COPY)
            {
            }
            BloodMarkedGeometry(const BloodMarkedGeometry& source, const osg::CopyOp& op)
                : osg::Geometry(source, op)
            {
            }
            META_Object(MWRender, BloodMarkedGeometry)
            osg::ref_ptr<SceneUtil::SurfaceDecals> decals;
            void accept(osg::NodeVisitor& visitor) override
            {
                if (!visitor.validNodeMask(*this))
                    return;
                osg::Geometry::accept(visitor);
                if (decals)
                    if (auto* cull = dynamic_cast<osgUtil::CullVisitor*>(&visitor))
                    {
                        visitor.pushOntoNodePath(this);
                        decals->cull(*this, *cull);
                        visitor.popFromNodePath();
                    }
            }
        };

        struct Triangles
        {
            std::vector<SceneUtil::DecalProjector::Triangle> triangles;
            void operator()(unsigned a, unsigned b, unsigned c) { triangles.push_back({ a, b, c }); }
        };
        struct Surface
        {
            int part = 0;
            unsigned ordinal = 0;
            std::string model;
            osg::ref_ptr<osg::Drawable> drawable;
            osg::ref_ptr<osg::Geometry> source;
            osg::NodePath path;
            std::vector<SceneUtil::DecalProjector::Triangle> triangles;
            std::uint64_t fingerprint = 0;

            SceneUtil::SurfaceDecals* marks() const
            {
                if (auto* rig = dynamic_cast<SceneUtil::RigGeometry*>(drawable.get()))
                    return rig->getSurfaceDecals();
                if (auto* morph = dynamic_cast<SceneUtil::MorphGeometry*>(drawable.get()))
                    return morph->getSurfaceDecals();
                if (auto* plain = dynamic_cast<BloodMarkedGeometry*>(drawable.get()))
                    return plain->decals;
                return nullptr;
            }
            SceneUtil::SurfaceDecals* attach()
            {
                if (auto* rig = dynamic_cast<SceneUtil::RigGeometry*>(drawable.get()))
                    return rig->getOrCreateSurfaceDecals();
                if (auto* morph = dynamic_cast<SceneUtil::MorphGeometry*>(drawable.get()))
                    return morph->getOrCreateSurfaceDecals();
                if (auto* plain = dynamic_cast<BloodMarkedGeometry*>(drawable.get()))
                {
                    if (!plain->decals)
                        plain->decals = new SceneUtil::SurfaceDecals;
                    return plain->decals;
                }
                if (path.size() < 2 || !path[path.size() - 2]->asGroup())
                    return nullptr;
                auto* plain = drawable->asGeometry();
                if (!plain)
                    return nullptr;
                osg::ref_ptr<BloodMarkedGeometry> replacement = new BloodMarkedGeometry(*plain);
                replacement->decals = new SceneUtil::SurfaceDecals;
                auto* parent = path[path.size() - 2]->asGroup();
                if (!parent->replaceChild(drawable, replacement))
                    return nullptr;
                drawable = replacement;
                source = replacement;
                path.back() = replacement;
                return replacement->decals;
            }
        };

        struct SurfacesVisitor final : osg::NodeVisitor
        {
            std::vector<Surface>& out;
            int part;
            std::string model;
            unsigned ordinal = 0;
            SurfacesVisitor(std::vector<Surface>& output, int p, std::string m)
                : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
                , out(output)
                , part(p)
                , model(std::move(m))
            {
            }
            void collect(osg::Node* root, osg::Node* actor)
            {
                // Preserve the entire model path through skinning skeletons,
                // without any Brutal Morrowind equipment-release dependencies.
                const auto paths = root->getParentalNodePaths();
                const osg::NodePath prefix = paths.empty() ? osg::NodePath{} : paths.front();
                for (std::size_t i = 0; i + 1 < prefix.size(); ++i)
                    pushOntoNodePath(prefix[i]);
                root->accept(*this);
                for (std::size_t i = 0; i + 1 < prefix.size(); ++i)
                    popFromNodePath();
            }
            void apply(osg::Drawable& drawable) override
            {
                Surface s;
                s.part = part;
                s.model = model;
                s.ordinal = ordinal++;
                s.drawable = &drawable;
                s.path = getNodePath();
                if (auto* rig = dynamic_cast<SceneUtil::RigGeometry*>(&drawable))
                    s.source = rig->getSourceGeometry();
                else if (auto* morph = dynamic_cast<SceneUtil::MorphGeometry*>(&drawable))
                    s.source = morph->getSourceGeometry();
                else if (typeid(drawable) == typeid(osg::Geometry) || dynamic_cast<BloodMarkedGeometry*>(&drawable))
                    s.source = drawable.asGeometry();
                if (!s.source)
                    return;
                const auto* vertices = dynamic_cast<const osg::Vec3Array*>(s.source->getVertexArray());
                if (!vertices || vertices->empty() || vertices->size() > 65535)
                    return;
                osg::TriangleIndexFunctor<Triangles> functor;
                s.source->accept(functor);
                s.triangles = std::move(functor.triangles);
                std::uint64_t hash = 14695981039346656037ull;
                const auto bytes = [&](const void* data, std::size_t size) {
                    auto* p = static_cast<const unsigned char*>(data);
                    while (size--)
                    {
                        hash ^= *p++;
                        hash *= 1099511628211ull;
                    }
                };
                for (const auto& p : *vertices)
                    for (unsigned i = 0; i < 3; ++i)
                    {
                        const float value = p[i];
                        bytes(&value, sizeof(value));
                    }
                for (const auto& t : s.triangles)
                    for (auto i : t)
                        bytes(&i, sizeof(i));
                s.fingerprint = hash;
                out.push_back(std::move(s));
            }
        };

        osg::ref_ptr<osg::Geometry> posed(
            const Surface& s, const std::function<std::optional<osg::Matrixf>(std::string_view)>& bones)
        {
            if (auto* rig = dynamic_cast<SceneUtil::RigGeometry*>(s.drawable.get()))
                return rig->snapshotWorldGeometry(s.path, bones, true);
            const auto* source = dynamic_cast<const osg::Vec3Array*>(s.source->getVertexArray());
            if (!source)
                return {};
            osg::ref_ptr<osg::Geometry> result = new osg::Geometry;
            osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(*source);
            if (auto* morph = dynamic_cast<SceneUtil::MorphGeometry*>(s.drawable.get()))
            {
                const auto& targets = morph->getMorphTargetList();
                if (!targets.empty())
                {
                    if (targets[0].getOffsets()->size() != vertices->size())
                        return {};
                    *vertices = *targets[0].getOffsets();
                    for (std::size_t i = 1; i < targets.size(); ++i)
                    {
                        if (targets[i].getOffsets()->size() != vertices->size())
                            return {};
                        for (std::size_t j = 0; j < vertices->size(); ++j)
                            (*vertices)[j] += (*targets[i].getOffsets())[j] * targets[i].getWeight();
                    }
                }
            }
            const auto matrix = osg::computeLocalToWorld(s.path);
            for (auto& p : *vertices)
                p = p * matrix;
            result->setVertexArray(vertices);
            return result;
        }

        struct Writer
        {
            std::string data = "BMD1";
            void u32(std::uint32_t n)
            {
                for (unsigned i = 0; i < 4; ++i)
                    data.push_back(char((n >> (i * 8)) & 255));
            }
            void f32(float f)
            {
                std::uint32_t n;
                std::memcpy(&n, &f, 4);
                u32(n);
            }
            void string(const std::string& s)
            {
                u32(s.size());
                data += s;
            }
        };
        struct Reader
        {
            std::string_view data;
            std::size_t p = 4;
            std::uint32_t u32()
            {
                if (p + 4 > data.size())
                    throw std::runtime_error("Truncated NPC blood record");
                std::uint32_t n = 0;
                for (unsigned i = 0; i < 4; ++i)
                    n |= std::uint32_t(static_cast<unsigned char>(data[p++])) << (i * 8);
                return n;
            }
            float f32()
            {
                auto n = u32();
                float f;
                std::memcpy(&f, &n, 4);
                if (!std::isfinite(f))
                    throw std::runtime_error("Nonfinite NPC blood record");
                return f;
            }
            std::string string()
            {
                const auto size = u32();
                if (size > 1024 || p + size > data.size())
                    throw std::runtime_error("Invalid NPC blood model");
                std::string s(data.substr(p, size));
                p += size;
                return s;
            }
        };
    }

    bool isRigidBloodSurface(const osg::Geometry* geometry)
    {
        return dynamic_cast<const BloodMarkedGeometry*>(geometry) != nullptr;
    }

    struct NpcAnimation::BloodDecals
    {
        struct Record
        {
            int part = 0;
            unsigned ordinal = 0, style = 0;
            std::string model;
            std::uint64_t fingerprint = 0;
            Anchors anchors;
        };
        std::vector<Record> records;
        std::array<osg::ref_ptr<osg::StateSet>, 4> states;
        bool prepared = false;
        std::uint64_t added = 0, rejected = 0, evicted = 0;
        std::size_t attached = 0, vertices = 0;
        double lastDistance = 0, lastMilliseconds = 0;
    };

    namespace
    {
        // Self-contained procedural stains: no Dark Messiah import or external game assets.
        osg::ref_ptr<osg::Image> makeBloodImage(unsigned style)
        {
            constexpr int N = 128;
            osg::ref_ptr<osg::Image> img = new osg::Image;
            img->allocateImage(N, N, 1, GL_RGBA, GL_UNSIGNED_BYTE);
            for (int y = 0; y < N; ++y)
                for (int x = 0; x < N; ++x)
                {
                    const float fx = (x - 63.5f) / 63.5f, fy = (y - 63.5f) / 63.5f;
                    const float sx = style == 0 ? fx * 0.55f : fx;
                    const float sy = style == 0 ? fy * 1.3f : fy;
                    const float dist = std::sqrt(sx * sx + sy * sy);
                    const float ragged = 0.84f + .11f * std::sin(17.f * fx + 7.f * fy + style * 2.f)
                        + .06f * std::sin(38.f * fy - 15.f * fx + style * 4.f);
                    const float opacity = std::clamp((ragged - dist) * 5.f, 0.f, 1.f)
                        * (0.65f + .2f * std::sin(11.f * fx + 13.f * fy));
                    unsigned char* p = img->data(x, y);
                    p[0] = 157; // red component above neutral; dark green/blue below
                    p[1] = 39;
                    p[2] = 37;
                    p[3] = static_cast<unsigned char>(std::clamp(opacity * 255.f, 0.f, 255.f));
                }
            return img;
        }
    }

    void NpcAnimation::prewarmBloodDecals()
    {
        if (mPtr == MWMechanics::getPlayer() || mViewMode != VM_Normal)
            return;
        if (!mBloodDecals)
            mBloodDecals = std::make_shared<BloodDecals>();
        auto& blood = *mBloodDecals;
        if (blood.prepared)
            return;
        blood.prepared = true;
        for (unsigned i = 0; i < 4; ++i)
        {
            osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D(makeBloodImage(i));
            texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
            texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
            mResourceSystem->getSceneManager()->applyFilterSettings(texture);
            osg::ref_ptr<osg::Geometry> geometry =
                osg::createTexturedQuadGeometry(osg::Vec3f(), osg::Vec3f(1, 0, 0), osg::Vec3f(0, 0, 1));
            auto* state = geometry->getOrCreateStateSet();
            const auto flags = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;
            state->setTextureAttributeAndModes(0, texture, flags);
            state->setTextureAttribute(0, new SceneUtil::TextureType("diffuseMap"), flags);
            state->setTextureAttribute(0, new osg::TexMat, flags);
            state->setAttributeAndModes(new osg::BlendFunc(GL_DST_COLOR, GL_SRC_COLOR, GL_ZERO, GL_ONE), flags);
            state->setAttributeAndModes(new SceneUtil::AutoDepth(osg::Depth::LEQUAL, 0, 1, false), flags);
            const float offset = SceneUtil::AutoDepth::isReversed() ? 1.f : -1.f;
            state->setAttributeAndModes(new osg::PolygonOffset(offset, offset), flags);
            state->setMode(GL_CULL_FACE,
                osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
            state->setAttribute(new osg::ColorMask(true, true, true, false), flags);
            state->setRenderBinDetails(1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
            mResourceSystem->getSceneManager()->setUpNormalsRTForStateSet(state, false);
            mResourceSystem->getSceneManager()->recreateShaders(geometry, "blooddecal");
            state = geometry->getStateSet();
            if (auto* program = state->getAttribute(osg::StateAttribute::PROGRAM))
                state->setAttributeAndModes(program, flags);
            blood.states[i] = state;
        }
    }

    void NpcAnimation::rebuildBloodDecals()
    {
        if (!mBloodDecals)
            return;
        auto& blood = *mBloodDecals;
        blood.attached = 0;
        blood.vertices = 0;
        std::vector<Surface> surfaces;
        for (int part = 0; part < ESM::PRT_Count; ++part)
            if (part != ESM::PRT_Weapon && part != ESM::PRT_Shield && mObjectParts[part])
            {
                SurfacesVisitor visitor(surfaces, part, std::string(mObjectParts[part]->getModel().value()));
                                visitor.collect(mObjectParts[part]->getNode(), mPtr.getRefData().getBaseNode());
            }
        for (auto& s : surfaces)
            if (auto* marks = s.marks())
                marks->clear();
        for (const auto& record : blood.records)
            for (auto& s : surfaces)
                if (record.part == s.part && record.ordinal == s.ordinal && record.model == s.model
                    && record.fingerprint == s.fingerprint)
                    if (auto* marks = s.attach())
                        if (marks->addPatch(record.anchors, blood.states[record.style], Mask_Effect))
                        {
                            ++blood.attached;
                            blood.vertices += record.anchors.size();
                            break;
                        }
    }

    bool NpcAnimation::addBloodDecal(
        const osg::Vec3f& position, const osg::Vec3f& outward, std::string_view kind, bool power, int hitPart)
    {
        prewarmBloodDecals();
        if (!mBloodDecals)
            return false;
        auto& blood = *mBloodDecals;
        const unsigned style = kind == "slash" || kind == "chop" ? 0 : power ? 2 : 1;
        if (!blood.states[style])
            return false;
        const auto start = std::chrono::steady_clock::now();
        try
        {
            std::vector<Surface> surfaces;
            for (int part = 0; part < ESM::PRT_Count; ++part)
                if (part != ESM::PRT_Weapon && part != ESM::PRT_Shield && part != ESM::PRT_Hair && mObjectParts[part]
                    && (hitPart < 0 || hitPart == part))
                {
                    SurfacesVisitor visitor(surfaces, part, std::string(mObjectParts[part]->getModel().value()));
                    visitor.collect(mObjectParts[part]->getNode(), mPtr.getRefData().getBaseNode());
                }
            const auto bones = [this](std::string_view name) { return getBoneWorldMatrix(name); };
            SceneUtil::DecalProjector::Result best;
            Surface* selected = nullptr;
            auto tangent = osg::Vec3d(0, 0, 1) ^ osg::Vec3d(outward);
            if (kind == "chop")
                tangent = osg::Vec3d(0, 0, 1);
            // Native units: the DM slash is 64 px at .1 Source inches/px.
            const double size = (style == 0 ? 15. : 24.) * (power ? 1.35 : 1.);
            for (auto& s : surfaces)
                if (auto geometry = posed(s, bones))
                {
                    const auto* vertices = dynamic_cast<const osg::Vec3Array*>(geometry->getVertexArray());
                    if (!vertices)
                        continue;
                    std::vector<osg::Vec3d> world(vertices->begin(), vertices->end());
                    auto projection = SceneUtil::DecalProjector::project(
                        world, s.triangles, osg::Vec3d(position), osg::Vec3d(outward), tangent, size, size);
                    if (!projection.vertices.empty() && projection.distance < best.distance)
                    {
                        best = std::move(projection);
                        selected = &s;
                    }
                }
            if (!selected)
            {
                ++blood.rejected;
                return false;
            }
            BloodDecals::Record record{ selected->part, selected->ordinal, style, selected->model,
                selected->fingerprint, std::move(best.vertices) };
            std::size_t total = 0;
            for (const auto& r : blood.records)
                total += r.anchors.size();
            while (!blood.records.empty()
                && (blood.records.size() >= MaxWounds || total + record.anchors.size() > MaxActorVertices))
            {
                total -= blood.records.front().anchors.size();
                blood.records.erase(blood.records.begin());
                ++blood.evicted;
            }
            blood.records.push_back(std::move(record));
            ++blood.added;
            blood.lastDistance = best.distance;
            rebuildBloodDecals();
            blood.lastMilliseconds
                = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            return true;
        }
        catch (const std::exception& error)
        {
            ++blood.rejected;
            Log(Debug::Warning) << "NPC blood projection rejected: " << error.what();
            return false;
        }
    }

    std::string NpcAnimation::getBloodDecalData() const
    {
        if (!mBloodDecals || mBloodDecals->records.empty())
            return {};
        Writer out;
        out.u32(mBloodDecals->records.size());
        for (const auto& r : mBloodDecals->records)
        {
            out.u32(r.part);
            out.u32(r.ordinal);
            out.u32(r.style);
            out.string(r.model);
            out.u32(r.fingerprint & 0xffffffff);
            out.u32(r.fingerprint >> 32);
            out.u32(r.anchors.size());
            for (const auto& a : r.anchors)
            {
                for (auto i : a.indices)
                    out.u32(i);
                for (unsigned i = 0; i < 3; ++i)
                    out.f32(a.weights[i]);
                out.f32(a.uv.x());
                out.f32(a.uv.y());
            }
        }
        return out.data;
    }

    bool NpcAnimation::restoreBloodDecals(std::string_view data)
    {
        prewarmBloodDecals();
        if (!mBloodDecals)
            return false;
        try
        {
            std::vector<BloodDecals::Record> records;
            if (!data.empty())
            {
                if (data.size() > MaxSaveBytes || data.substr(0, 4) != "BMD1")
                    return false;
                Reader in{ data };
                const auto count = in.u32();
                if (count > MaxWounds)
                    return false;
                std::size_t total = 0;
                for (unsigned j = 0; j < count; ++j)
                {
                    BloodDecals::Record r;
                    r.part = in.u32();
                    r.ordinal = in.u32();
                    r.style = in.u32();
                    r.model = in.string();
                    r.fingerprint = in.u32();
                    r.fingerprint |= std::uint64_t(in.u32()) << 32;
                    const auto size = in.u32();
                    if (r.part < 0 || r.part >= ESM::PRT_Count || r.ordinal > 4096 || r.style >= 4 || !size || size % 3
                        || size > 3072 || (total += size) > MaxActorVertices)
                        return false;
                    r.anchors.reserve(size);
                    for (unsigned i = 0; i < size; ++i)
                    {
                        SceneUtil::SurfaceDecals::Anchor a;
                        for (auto& index : a.indices)
                        {
                            index = in.u32();
                            if (index > 65535)
                                return false;
                        }
                        for (unsigned k = 0; k < 3; ++k)
                            a.weights[k] = in.f32();
                        const float u = in.f32(), v = in.f32();
                        a.uv = { u, v };
                        r.anchors.push_back(a);
                    }
                    osg::ref_ptr<SceneUtil::SurfaceDecals> validate = new SceneUtil::SurfaceDecals;
                    if (!validate->addPatch(r.anchors, new osg::StateSet, Mask_Effect))
                        return false;
                    records.push_back(std::move(r));
                }
                if (in.p != data.size())
                    return false;
            }
            mBloodDecals->records = std::move(records);
            rebuildBloodDecals();
            return true;
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    std::map<std::string, double> NpcAnimation::getBloodDecalStats() const
    {
        if (!mBloodDecals)
            return {{"enabled", 1.}, {"available", 0.}, {"records", 0.}};
        const auto& b = *mBloodDecals;
        return {{"enabled", 1.}, {"available", b.states[0] ? 1. : 0.},
            {"records", double(b.records.size())}, {"attached", double(b.attached)},
            {"added", double(b.added)}, {"rejected", double(b.rejected)}, {"evicted", double(b.evicted)}};
    }

}
