#version 130

in vec3 vertex;
in vec3 color;

void main() { gl_FragColor = vec4(color, 1.f); }
