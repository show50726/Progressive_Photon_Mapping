#pragma once
#include <iostream>
#include <algorithm>
#include "algebra3.h"

enum ReflectionType { DIFFUSE, SPECULAR, REFRACTION };

struct Material {
public:
	vec3 color;
	float Ka, Kd, Ks;
	float specularity;
	float reflectionRadio;
	ReflectionType rType;

	Material() { }

	Material(vec3 c, ReflectionType t) {
		color = c;
		rType = t;
	}

	vec3 CalcColor(vec3 normal, vec3 lightDir, vec3 viewDir) const;

private:
	vec3 _CalcDiffuse(vec3 normal, vec3 lightDir) const;
	vec3 _CalcSpecular(vec3 normal, vec3 lightDir, vec3 viewDir) const;
};
