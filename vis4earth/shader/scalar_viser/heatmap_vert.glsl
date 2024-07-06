#version 130

uniform sampler1D tfTex;
uniform sampler2D volSliceTex;
uniform int volHeight;
uniform int height;
uniform float latitudeMin;
uniform float latitudeMax;
uniform float longtitudeMin;
uniform float longtitudeMax;
uniform float heightMin;
uniform float heightMax;

out vec3 vertex;
out vec3 color;

void main() {
    {
        float lon = longtitudeMin + gl_Vertex.x * (longtitudeMax - longtitudeMin);
        float lat = latitudeMin + gl_Vertex.y * (latitudeMax - latitudeMin);
        float h = heightMin + (1.f * height / volHeight) * (heightMax - heightMin);
        vertex.z = h * sin(lat);
        h *= cos(lat);
        vertex.y = h * sin(lon);
        vertex.x = h * cos(lon);
    }

    float scalar = texture(volSliceTex, gl_Vertex.xy).r;
    color = texture(tfTex, scalar).rgb;

    gl_Position = gl_ModelViewProjectionMatrix * vec4(vertex, 1.f);
}
