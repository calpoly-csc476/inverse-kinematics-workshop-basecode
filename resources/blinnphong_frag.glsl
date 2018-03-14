#version 330

uniform vec3 uColor;
uniform vec3 uLightPos;
uniform vec3 uCameraPos;

in vec3 fWorldPos;
in vec3 fNormal;

out vec4 fragColor;


float saturate(float a)
{
	return clamp(a, 0.0, 1.0);
}

void main()
{
	vec3 k_a = 0.3 * uColor;
	vec3 k_d = 0.7 * uColor;
	vec3 k_s = vec3(0.5);
	const float alpha = 100.0;

	vec3 L = normalize(uLightPos);
	vec3 N = normalize(fNormal);
	vec3 V = normalize(uCameraPos - fWorldPos);
	vec3 H = normalize(V + L);

	fragColor.a = 1.0;
	fragColor.rgb = k_a + k_d * saturate(dot(N, L)) + k_s * pow(saturate(dot(H, N)), alpha);
}
