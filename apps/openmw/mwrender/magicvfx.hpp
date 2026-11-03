#ifndef OPENMW_MWRENDER_MAGICVFX_H
#define OPENMW_MWRENDER_MAGICVFX_H

// Procedural, texture-free magical effects shared by spell casting, hits,
// persistent auras and moving magic bolts.  This intentionally has no physics
// dependencies: projectile collision still uses the original collision mesh.

#include <algorithm>
#include <cctype>
#include <iterator>
#include <string>
#include <string_view>

#include <osg/AutoTransform>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/DrawArrays>
#include <osg/FrameStamp>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Program>
#include <osg/Shader>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/Vec4>

#include "vismask.hpp"

namespace MWRender::MagicVfx
{
    enum class Phase
    {
        Projectile,
        CastOrImpact,
        Persistent,
    };

    // The ESM effect colour is available for projectiles. For other effects
    // the asset name provides a stable, data-file-independent colour fallback.
    inline osg::Vec4f colorFromName(std::string_view model, std::string_view texture = {})
    {
        std::string key(model);
        key.append(" ");
        key.append(texture);
        std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) { return std::tolower(c); });

        const auto has = [&key](std::string_view word) { return key.find(word) != std::string::npos; };
        if (has("fire") || has("flame") || has("burn"))
            return { 1.f, 0.27f, 0.045f, 1.f };
        if (has("frost") || has("ice") || has("cold"))
            return { 0.25f, 0.74f, 1.f, 1.f };
        if (has("shock") || has("lightning") || has("electric"))
            return { 0.49f, 0.68f, 1.f, 1.f };
        if (has("poison") || has("acid") || has("disease") || has("blight"))
            return { 0.25f, 0.92f, 0.28f, 1.f };
        if (has("restore") || has("heal") || has("cure"))
            return { 1.f, 0.77f, 0.28f, 1.f };
        if (has("conjur") || has("summon") || has("bound"))
            return { 0.72f, 0.24f, 1.f, 1.f };
        if (has("alter") || has("shield") || has("levitat"))
            return { 0.29f, 0.88f, 0.82f, 1.f };
        if (has("illusion") || has("chameleon") || has("invis"))
            return { 0.36f, 0.44f, 1.f, 1.f };
        return { 0.72f, 0.43f, 1.f, 1.f };
    }

    inline osg::ref_ptr<osg::Program> getProgram()
    {
        static osg::ref_ptr<osg::Program> program;
        if (program)
            return program;

        static constexpr const char* vertexSource = R"GLSL(
            #version 120
            varying vec2 vMagicUv;
            void main()
            {
                vMagicUv = gl_MultiTexCoord0.st;
                gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
            }
        )GLSL";

        static constexpr const char* fragmentSource = R"GLSL(
            #version 120
            varying vec2 vMagicUv;
            uniform vec4 uMagicColor;
            uniform float uMagicTime;
            uniform float uMagicPhase;

            float magicHash(vec2 p)
            {
                return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
            }

            float magicNoise(vec2 p)
            {
                vec2 cell = floor(p);
                vec2 f = fract(p);
                f = f * f * (3.0 - 2.0 * f);
                return mix(mix(magicHash(cell), magicHash(cell + vec2(1.0, 0.0)), f.x),
                           mix(magicHash(cell + vec2(0.0, 1.0)), magicHash(cell + vec2(1.0, 1.0)), f.x), f.y);
            }

            void main()
            {
                vec2 p = vMagicUv * 2.0 - 1.0;
                float r = length(p);
                if (r >= 1.0)
                    discard;

                float t = uMagicTime;
                float angle = atan(p.y, p.x);
                float n = magicNoise(p * 5.5 + vec2(t * 0.68, -t * 0.47));
                float warp = 0.065 * (n - 0.5);
                float rr = max(0.0, r + warp);

                // Orb core, twisting wisps, orbiting runes and an expanding
                // impact ring. Entirely procedural; no animated DDS atlas.
                float core = exp(-23.0 * rr * rr);
                float innerGlow = exp(-6.5 * rr * rr);
                float arms = sin(angle * 5.0 - t * 5.0 + rr * 16.0 + n * 3.2);
                float wisps = pow(max(0.0, arms), 3.0) * exp(-4.2 * rr);
                float sparks = pow(max(0.0, sin(angle * 13.0 + t * 2.6 + rr * 23.0)), 16.0)
                               * exp(-11.0 * (rr - 0.53) * (rr - 0.53));

                float impact = clamp(uMagicPhase, 0.0, 1.0);
                float ringRadius = 0.24 + 0.52 * fract(t * 1.7);
                float ring = exp(-175.0 * pow(rr - ringRadius, 2.0))
                             * (1.0 - fract(t * 1.7)) * impact;
                float shell = exp(-62.0 * pow(rr - 0.54, 2.0));

                float edge = 1.0 - smoothstep(0.68, 0.99, rr);
                float energy = 0.48 * innerGlow + 0.40 * wisps + 0.22 * sparks
                               + 0.72 * core + 0.55 * ring + 0.10 * shell;
                float pulse = 0.91 + 0.09 * sin(t * 8.0 + rr * 14.0);
                float alpha = clamp(energy * pulse * edge, 0.0, 0.95);

                vec3 color = uMagicColor.rgb * (0.7 + 0.62 * wisps + 0.45 * ring);
                color += vec3(1.0) * (0.56 * core + 0.17 * sparks);
                gl_FragColor = vec4(color, alpha);
            }
        )GLSL";

        program = new osg::Program;
        program->setName("OpenMW procedural magic VFX");
        program->addShader(new osg::Shader(osg::Shader::VERTEX, vertexSource));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentSource));
        return program;
    }

    class TimeCallback final : public osg::NodeCallback
    {
    public:
        explicit TimeCallback(osg::Uniform* time)
            : mTime(time)
        {
        }

        void operator()(osg::Node* node, osg::NodeVisitor* visitor) override
        {
            if (visitor && visitor->getFrameStamp())
            {
                const double now = visitor->getFrameStamp()->getSimulationTime();
                if (mStartTime < 0.0)
                    mStartTime = now;
                mTime->set(static_cast<float>(now - mStartTime));
            }
            traverse(node, visitor);
        }

    private:
        osg::ref_ptr<osg::Uniform> mTime;
        double mStartTime = -1.0;
    };

    inline osg::ref_ptr<osg::Node> create(const osg::Vec4f& color, float radius, Phase phase)
    {
        osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec2Array> uv = new osg::Vec2Array;

        const osg::Vec3f positions[] = {
            { -radius, -radius, 0.f }, { radius, -radius, 0.f }, { radius, radius, 0.f },
            { -radius, -radius, 0.f }, { radius, radius, 0.f }, { -radius, radius, 0.f },
        };
        const osg::Vec2f texCoords[] = {
            { 0.f, 0.f }, { 1.f, 0.f }, { 1.f, 1.f },
            { 0.f, 0.f }, { 1.f, 1.f }, { 0.f, 1.f },
        };
        vertices->assign(std::begin(positions), std::end(positions));
        uv->assign(std::begin(texCoords), std::end(texCoords));
        quad->setVertexArray(vertices);
        quad->setTexCoordArray(0, uv);
        quad->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, 6));
        quad->setUseDisplayList(false);
        quad->setUseVertexBufferObjects(true);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(quad);

        osg::StateSet* state = geode->getOrCreateStateSet();
        state->setAttributeAndModes(getProgram().get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        state->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
        state->setMode(GL_BLEND, osg::StateAttribute::ON);
        state->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
        state->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE), osg::StateAttribute::ON);
        osg::ref_ptr<osg::Depth> depth = new osg::Depth;
        depth->setWriteMask(false);
        state->setAttributeAndModes(depth, osg::StateAttribute::ON);
        state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        state->addUniform(new osg::Uniform("uMagicColor", color));
        state->addUniform(new osg::Uniform("uMagicPhase", static_cast<float>(phase)));
        osg::ref_ptr<osg::Uniform> time = new osg::Uniform("uMagicTime", 0.f);
        state->addUniform(time);

        osg::ref_ptr<osg::AutoTransform> billboard = new osg::AutoTransform;
        billboard->setNodeMask(Mask_Effect);
        billboard->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
        billboard->addChild(geode);
        billboard->addUpdateCallback(new TimeCallback(time));
        return billboard;
    }
}

#endif
