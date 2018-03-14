#version 330

layout (location = 0) in vec4 vertPos;
layout (location = 1) in vec3 vertNor;

uniform mat4 P;
uniform mat4 V;
uniform mat4 M;

out vec3 fWorldPos;
out vec3 fNormal;


void main()
{
	// compute world space position
	vec4 Position = M * vertPos;
	fWorldPos = vec3(Position);

	// write out clip space
	gl_Position = P * V * Position;

	// compute the normal in world space
	fNormal = vec3(M * vec4(normalize(vertNor), 0.0));
}
