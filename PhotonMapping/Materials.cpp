#include "Materials.h"

inline float max(float a, float b) {
	return a > b ? a : b;
}

vec3 Material::CalcColor(vec3 normal, vec3 lightDir, vec3 viewDir) const
{
	vec3 ambient = Ka * color;
	vec3 diffuse = Kd * _CalcDiffuse(normal, lightDir);
	vec3 specular = Ks * _CalcSpecular(normal, lightDir, viewDir);

	return ambient + diffuse + specular;
}

vec3 Material::_CalcDiffuse(vec3 normal, vec3 lightDir) const
{
	float strength = lightDir * normal;
	strength = max(strength, 0.0f);
	return color * strength;
}

vec3 Material::_CalcSpecular(vec3 normal, vec3 lightDir, vec3 viewDir) const
{
	vec3 h = -viewDir + lightDir;
	float strength = h.normalize() * normal;
	strength = pow(max(strength, 0.0f), specularity);

	return (1-color) * strength;
}