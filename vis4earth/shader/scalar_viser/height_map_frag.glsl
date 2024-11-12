#version 130
#extension GL_EXT_geometry_shader4 : enable

uniform vec2 heightRange;
uniform int useShading;
uniform float ka;
uniform float kd;
uniform float ks;
uniform float shininess;
uniform vec3 eyePos;
uniform vec3 lightPos;

in float heightG2F;
in vec3 vertexG2F;
in vec3 normalG2F;

void main() {
	/*vec3 color = abs(normal);

	if (useShading != 0) {
		vec3 p2e = normalize(eyePos - vertex);
		vec3 p2l = normalize(lightPos - vertex);
		vec3 hfDir = normalize(p2e + p2l);

		vec3 ambient = ka * vec3(1.f, 1.f, 1.f);
		vec3 diffuse = kd * max(0, dot(normal, p2l)) * vec3(1.f, 1.f, 1.f);
		vec3 specular = ks * pow(max(0, dot(normal, hfDir)), shininess) * vec3(1.f, 1.f, 1.f);
		color = (ambient + diffuse + specular) * color;

		gl_FragColor = vec4(color, 1.f);
	}
	else
		gl_FragColor = vec4(color, 1.f);*/
	float h = heightG2F / (heightRange.y - heightRange.x);
	gl_FragColor = vec4(h, h, h, 1.f);
}
