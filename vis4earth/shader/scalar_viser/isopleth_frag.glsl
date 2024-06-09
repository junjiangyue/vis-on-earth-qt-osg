#version 130

uniform float ka;
uniform float kd;
uniform float ks;
uniform float shininess;
uniform float lightPosX;
uniform float lightPosY;
uniform float lightPosZ;
uniform bool useShading;
uniform vec3 eyePos;

in vec3 vertex;
in vec3 normal;
in vec3 color;

void main() {
    if (useShading) {
        vec3 p2e = normalize(eyePos - vertex);
        vec3 p2l = normalize(vec3(lightPosX, lightPosY, lightPosZ) - vertex);
        vec3 hfDir = normalize(p2e + p2l);

        vec3 absNorm = dot(normal, p2l) < 0.f ? -normal : normal;

        vec3 ambient = ka * vec3(1.f, 1.f, 1.f);
        vec3 diffuse = kd * dot(absNorm, p2l) * vec3(1.f, 1.f, 1.f);
        vec3 specular = ks * pow(max(0.f, dot(absNorm, hfDir)), shininess) * vec3(1.f, 1.f, 1.f);
        vec3 shadingColor = (ambient + diffuse + specular) * color;

        gl_FragColor = vec4(shadingColor, 1.f);
    } else
        gl_FragColor = vec4(color, 1.f);
}
