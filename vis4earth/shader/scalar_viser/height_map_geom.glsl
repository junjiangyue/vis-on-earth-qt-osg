#version 130
#extension GL_EXT_geometry_shader4 : enable

uniform int isPillar;

in vec3 vertex[];
in float height[];

out float heightG2F;
out vec3 vertexG2F;
out vec3 normalG2F;

void main() {
	vec3 newPos[3];

	if (isPillar == 0)
	{
		newPos[0] = vertex[0] + height[0] * normalize(vertex[0]);
		newPos[1] = vertex[1] + height[1] * normalize(vertex[1]);
		newPos[2] = vertex[2] + height[2] * normalize(vertex[2]);

		vec3 e0 = newPos[1] - newPos[0];
		vec3 e1 = newPos[2] - newPos[0];
		normalG2F = normalize(cross(e0, e1));

		heightG2F = height[0];
		vertexG2F = newPos[0];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		heightG2F = height[1];
		vertexG2F = newPos[1];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		heightG2F = height[2];
		vertexG2F = newPos[2];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		EndPrimitive();
	}
	else {
		heightG2F = .5f * (height[0] + height[2]);

		newPos[0] = vertex[0] + heightG2F * normalize(vertex[0]);
		newPos[1] = vertex[1] + heightG2F * normalize(vertex[1]);
		newPos[2] = vertex[2] + heightG2F * normalize(vertex[2]);

		vec3 e0 = vertex[1] - vertex[0];
		vec3 e1 = newPos[1] - vertex[0];
		normalG2F = normalize(cross(e0, e1));

		vertexG2F = newPos[0];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		vertexG2F = vertex[0];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		vertexG2F = newPos[1];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		vertexG2F = vertex[1];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		EndPrimitive();

		e0 = vertex[2] - vertex[1];
		e1 = newPos[2] - vertex[1];
		normalG2F = normalize(cross(e0, e1));

		vertexG2F = newPos[1];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		vertexG2F = vertex[1];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		vertexG2F = newPos[2];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		vertexG2F = vertex[2];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		EndPrimitive();

		normalG2F = normalize(vertex[0] + vertex[2]);

		vertexG2F = newPos[0];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		vertexG2F = newPos[1];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		vertexG2F = newPos[2];
		gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexG2F, 1.f);
		EmitVertex();

		EndPrimitive();
	}
}
