#version 130

uniform sampler1D tfTex0;
uniform sampler1D tfTex1;
uniform int colorMappingMode;
uniform mat3 rotMat;
uniform float latitudeMin;
uniform float latitudeMax;
uniform float longtitudeMin;
uniform float longtitudeMax;
uniform float heightMin;
uniform float heightMax;

out vec3 vertex;
out vec3 normal;
out vec3 color;

void main() {
    {
        float lon = longtitudeMin + gl_Vertex.x * (longtitudeMax - longtitudeMin);
        float lat = latitudeMin + gl_Vertex.y * (latitudeMax - latitudeMin);
        float h = heightMin + gl_Vertex.z * (heightMax - heightMin);
        vertex.z = h * sin(lat);
        h *= cos(lat);
        vertex.y = h * sin(lon);
        vertex.x = h * cos(lon);
    }
    normal = rotMat * gl_Normal;
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

    gl_Position = gl_ModelViewProjectionMatrix * vec4(vertex, 1.f);
}
