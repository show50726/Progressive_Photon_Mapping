#include "stdafx.h"
#include <iostream>
#include <cstdio>
#include <vector>
#include "Foundation.h"
#include "BroadPhase.h"
#include "ImageIO.h"
#include "Shape.h"
#include "MathUtils.h"

using namespace std;

Pixel Vec2Pixel(vec3 color)
{
	int rr = ToInt(color[0]);
	int gg = ToInt(color[1]);
	int bb = ToInt(color[2]);

	unsigned char r = static_cast<unsigned char>(rr);
	unsigned char g = static_cast<unsigned char>(gg);
	unsigned char b = static_cast<unsigned char>(bb);

	return Pixel(r, g, b);
}

int main()
{
	int width = 1024;
	int height = 768;
	int samps = 1000;

	unsigned int num_hash, pixel_index, num_photon;

	Ray cam(vec3(50, 48, 295.6), vec3(0, -0.042612, -1).normalize());
	vec3 cx = vec3(width*0.5135 / height, 0, 0);
	vec3 cy = (cx^cam.direction).normalize()*0.5135;
	vector<vec3> c(width*height, vec3(0, 0, 0));
	vec3 vw;

	vector<Shape*> shapes;
	Material m1(vec3(0.75, 0.25, 0.25), DIFFUSE);
	Material m1_1(vec3(0.25, 0.25, 0.75), DIFFUSE);
	Material m1_2(vec3(0.75, 0.75, 0.75), DIFFUSE);
	Material m1_3(vec3(0, 0, 0), DIFFUSE);
	Material m2(vec3(0.999, 0.999, 0.999), SPECULAR);
	Material m3(vec3(0.999, 0.999, 0.999), REFRACTION);
	Material m4(vec3(0.999, 0.999, 0.999), DIFFUSE);
	shapes.push_back(new Sphere(vec3(1e5 + 1, 40.8, 81.6), 1e5, m1));
	shapes.push_back(new Sphere(vec3(-1e5 + 99, 40.8, 81.6), 1e5, m1_1));
	shapes.push_back(new Sphere(vec3(50, 40.8, 1e5), 1e5, m1_2));
	shapes.push_back(new Sphere(vec3(50, 40.8, -1e5 + 170), 1e5, m1_3));
	shapes.push_back(new Sphere(vec3(50, 1e5, 81.6), 1e5, m1_2));
	shapes.push_back(new Sphere(vec3(50, -1e5 + 81.6, 81.6), 1e5, m1_2));
	shapes.push_back(new Sphere(vec3(27, 16.5, 47), 16.5, m2));
	shapes.push_back(new Sphere(vec3(73, 16.5, 88), 16.5, m3));
	shapes.push_back(new Sphere(vec3(50, 8.5, 60), 8.5, m4));

	vector<HitInfo> list;

	for (int i = 0; i < height; i++) {
		fprintf(stderr, "\rHitPointPass %5.2f%%", 100.0*i / (height - 1));
		for (int j = 0; j < width; j++) {
			pixel_index = i * width + j;
			vec3 d = cx * ((j + 0.5) / width - 0.5) + cy * (-(i + 0.5) / height + 0.5) + cam.direction;
			Ray ray = Ray(cam.startPoint + d * 140, d.normalize());
			ray.CastEyeRay(shapes, 0, vec3(0, 0, 0), vec3(1, 1, 1), 0, pixel_index, list);
		}
	}
	fprintf(stderr, "\n");

	double hashS, numHash;
	BoundingBox bbox;
	vector<HitInfoNode*> grid;

	// build hash
	HashGrid hashGrid = HashGrid(width, height, list);
	hashS = hashGrid.hashS;
	numHash = hashGrid.numHash;
	bbox = *hashGrid.boundingBox;
	grid = hashGrid.grid;

	num_photon = samps;
	vw = vec3(1, 1, 1);

#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < 1000; i++) {
		double p = 100.0*(i + 1) / 1000;
		fprintf(stderr, "\rPhotonPass %5.2f%%", p);
		int m = num_photon * i;
		Ray ray(vec3(0, 0, 0), vec3(0, 0, 0));
		vec3 f = vec3(0, 0, 0);
		for (int j = 0; j < num_photon; j++) {
			//cout << "Generate Photon Ray" << endl;
			ray.GeneratePhotonRay(f, m+j);
			//cout << "Cast Photon Ray" << endl;
			ray.CastPhotonRay(shapes, 0, f, vw, m + j, &bbox, numHash, hashS, grid);
		}
	}

	for (auto& hp : list) {
		int i = hp.pix;
		c[i] = c[i] + hp.flux*(1.0f / (PI*hp.r2*num_photon*1000.0));
	}

	ColorImage image;
	image.init(width, height);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			image.writePixel(j, i, Vec2Pixel(c[i*width + j]));
		}
	}
	image.outputPPM("PhotonMapping.ppm");

	system("pause");
    return 0;
}

