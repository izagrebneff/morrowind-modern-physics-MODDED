#ifndef OPENMW_MWRENDER_BRUTALCUTMATERIAL_H
#define OPENMW_MWRENDER_BRUTALCUTMATERIAL_H

#include "decapitation.hpp"
#include <components/resource/resourcesystem.hpp>
#include <components/resource/imagemanager.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/sceneutil/depth.hpp>
#include <components/sceneutil/texturetype.hpp>
#include <components/vfs/manager.hpp>
#include <osg/Texture2D>
#include <osg/Image>
#include <osg/Material>
#include <osg/PolygonOffset>
#include <osg/BlendFunc>

namespace MWRender
{
    // Match Brutal Morrowind's VFS material preparation for procedural cut caps.
    inline osg::ref_ptr<osg::StateSet> prepareBrutalCutMaterial(Resource::ResourceSystem* resources, bool fragment)
    {
        const VFS::Path::NormalizedView original(fragment ? "textures/brutal_dm/cut_head.png" : "textures/brutal_dm/cut_body.png");
        const VFS::Path::NormalizedView fallback("textures/brutal/neck_cut.png");
        const auto path = resources->getVFS()->exists(original) ? original : fallback;
        osg::ref_ptr<osg::Image> image;
        if (resources->getVFS()->exists(path))
            image = resources->getImageManager()->getImage(path);
        else
        {
            image = new osg::Image;
            image->allocateImage(1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE);
            auto* p = image->data();
            p[0] = 65; p[1] = 9; p[2] = 8; p[3] = 255;
        }
        osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D(image);
        texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        texture->setResizeNonPowerOfTwoHint(false);
        resources->getSceneManager()->applyFilterSettings(texture);
        osg::ref_ptr<osg::StateSet> state = new osg::StateSet;
        osg::ref_ptr<osg::Material> material = new osg::Material;
        material->setColorMode(osg::Material::OFF);
        material->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0,0,0,1));
        material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(.65f,.65f,.65f,1));
        material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(.35f,.35f,.35f,1));
        material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0,0,0,1));
        constexpr auto protect = osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;
        state->setAttributeAndModes(material, osg::StateAttribute::ON | protect);
        state->setMode(GL_LIGHTING, osg::StateAttribute::ON);
        state->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | protect);
        state->setMode(GL_BLEND, osg::StateAttribute::OFF | protect);
        state->setMode(GL_ALPHA_TEST, osg::StateAttribute::OFF | protect);
        state->setAttributeAndModes(new SceneUtil::AutoDepth);
        state->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON | protect);
        state->setTextureAttribute(0, new SceneUtil::TextureType("diffuseMap"));
        const std::vector<DecapitationGeometry::Vertex> vertices{
            {osg::Vec3f(0,0,0),osg::Vec3f(0,0,1),osg::Vec2f(0,0),osg::Vec4f(1,1,1,1)},
            {osg::Vec3f(1,0,0),osg::Vec3f(0,0,1),osg::Vec2f(1,0),osg::Vec4f(1,1,1,1)},
            {osg::Vec3f(0,1,0),osg::Vec3f(0,0,1),osg::Vec2f(0,1),osg::Vec4f(1,1,1,1)}};
        auto warm = DecapitationGeometry::makeGeometry(vertices, state, osg::Matrixf::identity());
        resources->getSceneManager()->recreateShaders(warm);
        osg::ref_ptr<osg::StateSet> prepared = warm->getStateSet();
        // Keep the cap opaque and independent of the equipment material. Do not
        // share mutable shader state between the two sides of the cut.
        prepared->setRenderBinToInherit();
        prepared->setRenderingHint(osg::StateSet::OPAQUE_BIN);
        prepared->setMode(GL_BLEND, osg::StateAttribute::OFF | protect);
        prepared->setMode(GL_ALPHA_TEST, osg::StateAttribute::OFF | protect);
        return prepared;
    }
}
#endif
