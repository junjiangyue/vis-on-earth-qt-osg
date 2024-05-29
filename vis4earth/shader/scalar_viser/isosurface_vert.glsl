#version 130

uniform sampler1D tfTex0;
uniform sampler1D tfTex1;
uniform int colorMappingMode;

out vec3 vertex;
out vec3 normal;
out vec3 color;

void main() {
    vertex = gl_Vertex.xyz;
    normal = gl_Normal;
    if (colorMappingMode == 0) {
        if (gl_MultiTexCoord0.x == 0.f)
            color = texture(tfTex0, gl_MultiTexCoord0.y).rgb;
        else
            color = texture(tfTex1, gl_MultiTexCoord0.y).rgb;
    } else {
        if (gl_MultiTexCoord0.x == 0.f)
            color = vec3(172.f, 232.f, 111.f) / 255.f;
        else
            color = vec3(50.f, 214.f, 234.f) / 255.f;
    }

    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
