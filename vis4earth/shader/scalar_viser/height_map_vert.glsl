#version 130
#extension GL_EXT_geometry_shader4 : enable

uniform sampler2D heightMapTex;
uniform vec2 heightRange;

out vec3 vertex;
out float height;

void main() {
    vertex = gl_Vertex.xyz;
    height = texture(heightMapTex, gl_MultiTexCoord0.xy).r;
    height = height * (heightRange.y - heightRange.x);
}
