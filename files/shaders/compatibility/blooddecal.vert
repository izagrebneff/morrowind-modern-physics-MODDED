#version 120
#include "lib/core/vertex.h.glsl"
varying vec2 bloodUV;
varying vec3 bloodViewPos;
uniform float bloodFade;
uniform float bloodDepthBias;
uniform float bloodDry;
uniform bool bloodVertexData;
varying vec3 bloodParameters;
void main()
{
    gl_Position = modelToClip(gl_Vertex);
    gl_Position.z += bloodDepthBias * gl_Position.w;
    vec4 viewPos = modelToView(gl_Vertex);
    gl_ClipVertex = viewPos;
    bloodViewPos = viewPos.xyz;
    bloodUV = gl_MultiTexCoord0.xy;
    bloodParameters = bloodVertexData ? gl_Color.rgb : vec3(bloodFade, bloodDry, 1.0);
}
