#version 120
uniform sampler2D diffuseMap;
uniform float bloodScale;
uniform bool bloodFeather;
varying vec2 bloodUV;
varying vec3 bloodViewPos;
varying vec3 bloodParameters;
void main()
{
    // A growing pool uses the same projected geometry throughout.
    float scale = bloodScale > 0.0 ? bloodScale : 1.0;
    vec4 stain = texture2D(diffuseMap, vec2(0.5) + (bloodUV - vec2(0.5)) / scale);
    stain.a *= clamp(gl_FrontMaterial.diffuse.a,0.0,1.0)
        * (1.0 - clamp(bloodParameters.r,0.0,1.0)) * bloodParameters.b;
    // DecalModulate textures can have an opaque neutral background. Ignore
    // that background (including compression noise) before tinting dry blood.
    vec3 fromNeutral = abs(stain.rgb - vec3(128.0 / 255.0));
    float contrast = max(max(fromNeutral.r, fromNeutral.g), fromNeutral.b);
    stain.a *= smoothstep(2.0 / 255.0, 10.0 / 255.0, contrast);
    if (bloodFeather)
    {
        // Leave a gradual edge even when the decal texture reaches its border.
        vec2 edge = min(bloodUV, vec2(1.0) - bloodUV);
        vec2 feather = smoothstep(vec2(0.0), vec2(0.22, 0.13), edge);
        stain.a *= feather.x * feather.y;
    }
    // Dried blood is darker and browner than a fresh smear.
    stain.rgb = mix(stain.rgb, stain.rgb * vec3(0.62, 0.5, 0.42), clamp(bloodParameters.g,0.0,1.0));
    // Source DecalModulate uses DST_COLOR + SRC_COLOR. RGB128 is
    // the neutral value in the original 8-bit textures. Respect alpha even
    // when transparent edge pixels contain saturated blood colours.
    vec3 modulation = vec3(0.5) + (stain.rgb - vec3(128.0 / 255.0)) * stain.a;
#if @radialFog
    float dist = length(bloodViewPos);
#else
    float dist = abs(bloodViewPos.z);
#endif
#if @exponentialFog
    float fog = 1.0 - exp(-2.0 * max(0.0, dist - gl_Fog.start/2.0) / (gl_Fog.end-gl_Fog.start/2.0));
#else
    float fog = clamp((dist-gl_Fog.start)*gl_Fog.scale,0.0,1.0);
#endif
    // The already lit/fogged NPC supplies lighting. Distant decals approach
    // a neutral multiplier instead of painting the fog a second time.
    gl_FragData[0] = vec4(mix(modulation,vec3(0.5),fog),0.0);
}
