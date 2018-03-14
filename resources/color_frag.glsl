#version 330

uniform vec3 uColor;

out vec4 fragColor;


void main()
{
	fragColor.a = 1.0;
	fragColor.rgb = uColor;
}
