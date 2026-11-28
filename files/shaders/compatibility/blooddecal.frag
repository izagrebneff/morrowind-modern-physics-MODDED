#version 120
uniform sampler2D diffuseMap;
uniform float bloodScale;
uniform vec3 bloodTint;
uniform bool bloodFeather;
varying vec2 bloodUV;
varying vec3 bloodViewPos;
varying vec3 bloodParameters;
void main()
{
    float scale = bloodScale > 0.0 ? bloodScale : 1.0;
    vec4 stain = texture2D(diffuseMap, vec2(0.5) + (bloodUV - vec2(0.5)) / scale);
    vec3 delta = abs(stain.rgb - vec3(128.0 / 255.0));
    float contrast = max(max(delta.r, delta.g), delta.b);
    float alpha = smoothstep(0.025, 0.24, contrast) * stain.a;
    alpha *= (1.0 - clamp(bloodParameters.r,0.0,1.0)) * bloodParameters.b;
    if (bloodFeather)
    {
        vec2 edge = min(bloodUV, vec2(1.0) - bloodUV);
        vec2 feather = smoothstep(vec2(0.0), vec2(0.22, 0.13), edge);
        alpha *= feather.x * feather.y;
    }
    if (alpha < 0.005) discard;
    vec3 color = bloodTint * mix(0.55, 1.0, clamp(contrast * 2.0, 0.0, 1.0));
    color = mix(color, color * vec3(0.62, 0.5, 0.42), clamp(bloodParameters.g,0.0,1.0));
    gl_FragData[0] = vec4(color, clamp(alpha, 0.0, 1.0));
}
