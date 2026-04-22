#define _CRT_SECURE_NO_WARNINGS 1
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <map>
#include <string>
#include <fstream>
//#include <omp.h> 

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#ifndef M_PI
#define M_PI 3.14159265358979323856
#endif

static std::default_random_engine engine[32];
static std::uniform_real_distribution<double> uniform(0, 1);

double sqr(double x) { return x * x; };

class Vector {
public:
	explicit Vector(double x = 0, double y = 0, double z = 0) {
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}
	double norm2() const {
		return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
	}
	double norm() const {
		return sqrt(norm2());
	}
	void normalize() {
		double n = norm();
		data[0] /= n;
		data[1] /= n;
		data[2] /= n;
	}
	double operator[](int i) const { return data[i]; };
	double& operator[](int i) { return data[i]; };
	double data[3];
};

Vector operator+(const Vector& a, const Vector& b) {
	return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const double a, const Vector& b) {
	return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const double b) {
	return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator/(const Vector& a, const double b) {
	return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
	return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

class Ray {
public:
	Ray(const Vector& origin, const Vector& unit_direction) : O(origin), u(unit_direction) {};
	Vector O, u;
};

class Object {
public:
	Object(const Vector& albedo, bool mirror = false, bool transparent = false) : albedo(albedo), mirror(mirror), transparent(transparent) {};

	virtual bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const = 0;

	Vector albedo;
	bool mirror, transparent;
};

class Sphere : public Object {
public:
	Sphere(const Vector& center, double radius, const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent), C(center), R(radius) {};

	// returns true iif there is an intersection between the ray and the sphere
	// if there is an intersection, also computes the point of intersection P, 
	// t>=0 the distance between the ray origin and P (i.e., the parameter along the ray)
	// and the unit normal N
	bool intersect(const Ray& ray, Vector& P, double &t, Vector& N) const {
		 // TODO (lab 1) : compute the intersection (just true/false at the begining of lab 1, then P, t and N as well)
		double delta = dot(ray.u, ray.O-C)*dot(ray.u, ray.O-C) - ((ray.O-C).norm2() - R*R); 
		if (delta < 0){
			return false;
		}
		// we want the smallest positive one 
		else if (delta == 0){
			double t0 = dot(ray.u,C-ray.O);
			if (t0 >=0){
				t = t0;
				P = ray.O + t*ray.u;
				N = P-C;
				N.normalize();
				return true;
			}
		}
		else {
			double t1 = dot(ray.u,C-ray.O) + sqrt(delta);
			double t2 = dot(ray.u,C-ray.O) - sqrt(delta);
			if (t1 <= t2){
				if (t1>=0){
					t = t1;
					P = ray.O + t*ray.u;
					N = P-C;
					N.normalize();
					return true;
				}
				else if (t2 >= 0){
					t = t2;
					P = ray.O + t*ray.u;
					N = P-C;
					N.normalize();
					return true;
				}
			}
			else if (t2 >= 0){
				t = t2;
				P = ray.O + t*ray.u;
				N = P-C;
				N.normalize();
				return true;
			}
			else if (t1 >=0){
				t = t1;
				P = ray.O + t*ray.u;
				N = P-C;
				N.normalize();
				return true;
			}
		}
		return false;
	}

	double R;
	Vector C;
};


// Class only used in labs 3 and 4 
class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1) {
		vtx[0] = vtxi; vtx[1] = vtxj; vtx[2] = vtxk;
		uv[0] = uvi; uv[1] = uvj; uv[2] = uvk;
		n[0] = ni; n[1] = nj; n[2] = nk;
		this->group = group;
	};
	int vtx[3]; // indices within the vertex coordinates array
	int uv[3];  // indices within the uv coordinates array
	int n[3];   // indices within the normals array
	int group;  // face group
};


// Class only used in labs 3 and 4 
class TriangleMesh : public Object {
public:
	TriangleMesh(const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent) {};

	// first scale and then translate the current object
	void scale_translate(double s, const Vector& t) {
		for (int i = 0; i < vertices.size(); i++) {
			vertices[i] = vertices[i] * s + t;
		}
	}

	// read an .obj file
	void readOBJ(const char* obj) {
		std::ifstream f(obj);
		if (!f) return;

		std::map<std::string, int> mtls;
		int curGroup = -1, maxGroup = -1;

		// OBJ indices are 1-based and can be negative (relative), this normalizes them
		auto resolveIdx = [](int i, int size) {
			return i < 0 ? size + i : i - 1;
		};

		auto setFaceVerts = [&](TriangleIndices& t, int i0, int i1, int i2) {
			t.vtx[0] = resolveIdx(i0, vertices.size());
			t.vtx[1] = resolveIdx(i1, vertices.size());
			t.vtx[2] = resolveIdx(i2, vertices.size());
		};
		auto setFaceUVs = [&](TriangleIndices& t, int j0, int j1, int j2) {
			t.uv[0] = resolveIdx(j0, uvs.size());
			t.uv[1] = resolveIdx(j1, uvs.size());
			t.uv[2] = resolveIdx(j2, uvs.size());
		};
		auto setFaceNormals = [&](TriangleIndices& t, int k0, int k1, int k2) {
			t.n[0] = resolveIdx(k0, normals.size());
			t.n[1] = resolveIdx(k1, normals.size());
			t.n[2] = resolveIdx(k2, normals.size());
		};

		std::string line;
		while (std::getline(f, line)) {
			// Trim trailing whitespace
			line.erase(line.find_last_not_of(" \r\t\n") + 1);
			if (line.empty()) continue;

			const char* s = line.c_str();

			if (line.rfind("usemtl ", 0) == 0) {
				std::string matname = line.substr(7);
				auto result = mtls.emplace(matname, maxGroup + 1);
				if (result.second) {
					curGroup = ++maxGroup;
				} else {
					curGroup = result.first->second;
				}
			} else if (line.rfind("vn ", 0) == 0) {
				Vector v;
				sscanf(s, "vn %lf %lf %lf", &v[0], &v[1], &v[2]);
				normals.push_back(v);
			} else if (line.rfind("vt ", 0) == 0) {
				Vector v;
				sscanf(s, "vt %lf %lf", &v[0], &v[1]);
				uvs.push_back(v);
			} else if (line.rfind("v ", 0) == 0) {
				Vector pos, col;
				if (sscanf(s, "v %lf %lf %lf %lf %lf %lf", &pos[0], &pos[1], &pos[2], &col[0], &col[1], &col[2]) == 6) {
					for (int i = 0; i < 3; i++) col[i] = std::min(1.0, std::max(0.0, col[i]));
					vertexcolors.push_back(col);
				} else {
					sscanf(s, "v %lf %lf %lf", &pos[0], &pos[1], &pos[2]);
				}
				vertices.push_back(pos);
			}
			else if (line[0] == 'f') {
				int i[4], j[4], k[4], offset, nn;
				const char* cur = s + 1;
				TriangleIndices t;
				t.group = curGroup;

				// Try each face format: v/vt/vn, v/vt, v//vn, v
				if ((nn = sscanf(cur, "%d/%d/%d %d/%d/%d %d/%d/%d%n", &i[0], &j[0], &k[0], &i[1], &j[1], &k[1], &i[2], &j[2], &k[2], &offset)) == 9) {
					setFaceVerts(t, i[0], i[1], i[2]); 
					setFaceUVs(t, j[0], j[1], j[2]); 
					setFaceNormals(t, k[0], k[1], k[2]);
				} else if ((nn = sscanf(cur, "%d/%d %d/%d %d/%d%n", &i[0], &j[0], &i[1], &j[1], &i[2], &j[2], &offset)) == 6) {
					setFaceVerts(t, i[0], i[1], i[2]); 
					setFaceUVs(t, j[0], j[1], j[2]);
				} else if ((nn = sscanf(cur, "%d//%d %d//%d %d//%d%n", &i[0], &k[0], &i[1], &k[1], &i[2], &k[2], &offset)) == 6) {
					setFaceVerts(t, i[0], i[1], i[2]); 
					setFaceNormals(t, k[0], k[1], k[2]);
				} else if ((nn = sscanf(cur, "%d %d %d%n", &i[0], &i[1], &i[2], &offset)) == 3) {
					setFaceVerts(t, i[0], i[1], i[2]);
				}
				else continue;

				indices.push_back(t);
				cur += offset;

				// Fan triangulation for polygon faces (4+ vertices)
				while (*cur && *cur != '\n') {
					TriangleIndices t2;
					t2.group = curGroup;
					if ((nn = sscanf(cur, " %d/%d/%d%n", &i[3], &j[3], &k[3], &offset)) == 3) {
						setFaceVerts(t2, i[0], i[2], i[3]); 
						setFaceUVs(t2, j[0], j[2], j[3]); 
						setFaceNormals(t2, k[0], k[2], k[3]);
					} else if ((nn = sscanf(cur, " %d/%d%n", &i[3], &j[3], &offset)) == 2) {
						setFaceVerts(t2, i[0], i[2], i[3]); 
						setFaceUVs(t2, j[0], j[2], j[3]);
					} else if ((nn = sscanf(cur, " %d//%d%n", &i[3], &k[3], &offset)) == 2) {
						setFaceVerts(t2, i[0], i[2], i[3]); 
						setFaceNormals(t2, k[0], k[2], k[3]);
					} else if ((nn = sscanf(cur, " %d%n", &i[3], &offset)) == 1) {
						setFaceVerts(t2, i[0], i[2], i[3]);
					} else { 
						cur++; 
						continue; 
					}

					indices.push_back(t2);
					cur += offset;
					i[2] = i[3]; j[2] = j[3]; k[2] = k[3];
				}
			}
		}
	}
	

	// TODO ray-mesh intersection (labs 3 and 4)
	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const {
		
		// lab 3 : for each triangle, compute the ray-triangle intersection with Moller-Trumbore algorithm
		// lab 3 : once done, speed it up by first checking against the mesh bounding box
		// lab 4 : recursively apply the bounding-box test from a BVH datastructure


		return false;
	}


	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
};


class Scene {
public:
	Scene() {};
	void addObject(const Object* obj) {
		objects.push_back(obj);
	}

	// returns true iif there is an intersection between the ray and any object in the scene
    // if there is an intersection, also computes the point of the *nearest* intersection P, 
    // t>=0 the distance between the ray origin and P (i.e., the parameter along the ray)
    // and the unit normal N. 
	// Also returns the index of the object within the std::vector objects in object_id
	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N, int &object_id) const  {

		// TODO (lab 1): iterate through the objects and check the intersections with all of them, 
		// and keep the closest intersection, i.e., the one if smallest positive value of t
		bool res = false;
		for (size_t i = 0; i < objects.size(); ++i){
			double better_t{};
			Vector better_P{};
			Vector better_N{};

			if (objects[i]->intersect(ray, better_P, better_t, better_N)){
				res = true;
				if (better_t <= t){
					t = better_t;
					P = better_P;
					N = better_N;
					object_id = i;
				}
			}

		}
		return res;
	}


	// return the radiance (color) along ray
	Vector getColor(const Ray& ray, int recursion_depth) {

		if (recursion_depth >= max_light_bounce) return Vector(0, 0, 0);
		if (recursion_depth<0) return Vector(0, 0, 0);

		// TODO (lab 1) : if intersect with ray, use the returned information to compute the color ; otherwise black 
		// in lab 1, the color only includes direct lighting with shadows

		Vector P, N;
		double t = 1e9;
		int object_id;
		if (intersect(ray, P, t, N, object_id)) {
			//std::cout << object_id << std::endl;
			if (objects[object_id]->mirror) {

				// return getColor in the reflected direction, with recursion_depth+1 (recursively)
				Ray next_ray(P+0.001*N, ray.u - 2*dot(ray.u,N)*N);
				return getColor(next_ray,recursion_depth+1);
			}

			if (objects[object_id]->transparent) { // optional

				// return getColor in the refraction direction, with recursion_depth+1 (recursively)
			} // else 
			Vector Lo (0., 0.,0.) ;
			//initialising new ray from point P to Light 
			Vector LP = light_position - P; 
			double d = LP.norm();
            Vector new_ray_u = LP/d;
            Ray newRay(P+0.0001*N, new_ray_u); 

			// We check if there is a shadow = there is an intersection between our new ray and an object 
			// First we need to initialize the variables in which we will stock the object that the ray instersects, if any
			// REMINIDER : returns true iif there is an intersection between the ray and any object in the scene. if there is an intersection, also computes the point of the *nearest* intersection P, t>=0 the distance between the ray origin and P (i.e., the parameter along the ray) and the unit normal N. 
			// REMINIDER : Also returns the index of the object within the std::vector objects in object_id
            Vector inter_P, inter_N;
            double inter_t = 1e9;
            int inter_index;
            bool is_intersection = intersect(newRay, inter_P, inter_t, inter_N, inter_index);
			bool is_shadow = is_intersection && (inter_t < d); // true if there is a shadow 
			double cos_theta = std::max(0., dot(N, new_ray_u));
            if (!is_shadow && cos_theta > 0) { //if no shadow, compute the formula with dot product 
				Lo = (light_intensity / (4 * M_PI * d * d)) * (objects[object_id]->albedo / M_PI) * cos_theta;
				//return (light_intensity / (4 * M_PI * d * d)) * (objects[object_id]->albedo / M_PI) * cos_theta;
            }

			//(lab 2) : add indirect lighting component with a recursive call
			// sample random direction omega_i with pdf dot(omega_i, n)/pi 
			double r1 = uniform(engine[0]);
			double r2 = uniform(engine[0]);
			double x = cos(2*M_PI*r1)*sqrt(1-r2);
			double y = sin(2*M_PI*r1)*sqrt(1-r2);
			double z = sqrt(r2);
			// change of frame
			Vector T1;
			if (std::abs(N[0]) >= std::abs(N[2]) && std::abs(N[1]) >= std::abs(N[2]) ){ // if Nz is the smallest
				T1 = Vector(-N[1],N[0],0);
			}
			
			else if (std::abs(N[2]) >= std::abs(N[0]) && std::abs(N[1]) >= std::abs(N[0]) ){// if Nx is the smallest
				T1 = Vector(0,N[2],-N[1]);
			}

			else if (std::abs(N[2]) >= std::abs(N[1]) && std::abs(N[0]) >= std::abs(N[1])){// if Ny is the smallest
				T1 = Vector(-N[2],0,N[0]);
			}
			T1.normalize();
			Vector T2 = cross(N,T1);
			Vector random_vector = x*T1 + y*T2 + z*N;
			random_vector.normalize();
			Ray random_ray(P+0.0001*N,random_vector);
			Vector albedo = objects[object_id]->albedo;
			Vector v = getColor(random_ray, recursion_depth+1);
			Vector product(albedo[0] * v[0],albedo[1] * v[1],albedo[2] * v[2]);
			Lo = Lo + product;
			return Lo;
		}

		return Vector(0, 0, 0);
	}

	std::vector<const Object*> objects;

	Vector camera_center, light_position;
	double fov, gamma, light_intensity;
	int max_light_bounce;
};


int main() {
	int W = 512;
	int H = 512;

	for (int i = 0; i<32; i++) {
		engine[i].seed(i);
	}

	Sphere center_sphere(Vector(0, 0, 0), 10., Vector(0.8, 0.8, 0.8),true);
	Sphere wall_left(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
	Sphere wall_right(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
	Sphere wall_front(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere wall_behind(Vector(0, 0, 1000), 940, Vector(0.8, 0.2, 0.9));
	Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));
	Sphere floor(Vector(0, -1000, 0), 990, Vector(0.6, 0.5, 0.7));

	Scene scene;
	scene.camera_center = Vector(0, 0, 55);
	scene.light_position = Vector(-10,20,40);
	scene.light_intensity = 3E7;
	scene.fov = 60 * M_PI / 180.;
	scene.gamma = 2.2;    // TODO (lab 1) : play with gamma ; typically, gamma = 2.2
	scene.max_light_bounce = 8;

	scene.addObject(&center_sphere);


	scene.addObject(&wall_left);
	scene.addObject(&wall_right);
	scene.addObject(&wall_front);
	scene.addObject(&wall_behind);
	scene.addObject(&ceiling);
	scene.addObject(&floor);


	std::vector<unsigned char> image(W * H * 3, 0);

#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			Vector color;

			// TODO (lab 1) : correct ray_direction so that it goes through each pixel (j, i)	
			// LAB 1: Pour chaque pixel de l'image on lancait un seul rayon: il part de la cam, il passe par le centre du pixel on regarde ce quil intersect dans la scene et apres on calcul la couleur du pixel
			// 1 pixel = 1 rayon
			// Probleme 1 pixel cest une surface, une case du coup cest comme si on colorai tout en cases ca fais que les objets qui sont courbé seront representer avec des cases.. du coup l'image n'est pas bonne 
			// creer les bords en escalier, très durs, pas lisses -> **aliasing**.
			double x = j-W/2 + 1/2;
			double y =  H/2-i-1/2;
			double z = -W/(2*tan(scene.fov/2));
			Vector ray_direction(j-W/2 + 1/2, H/2-i-1/2, -W/(2*tan(scene.fov/2)));
			ray_direction.normalize();

			// BUT DU LAB 2 : ANTIALIASING Rendre l'image plus lisse 1 pixel = plusieurs rayons = moyenne des contributions
			// avec plusieurs rayons certains touche la sphere et dautre non, la moyenne donne une couleur intermediaire 
			// dans le cours on choisit les points suivant une Gaussienne (plus de point proche du centre, moinds de points loin)

			Ray ray(scene.camera_center, ray_direction);

			// TODO (lab 2) : add Monte Carlo / averaging of random ray contributions here
			int NB_PATHS = 1000;
			int max_path_length = 5;
			Vector pixelColor(0,0,0);
			for (int k=0; k<NB_PATHS;k++){
				// TODO (lab 2) : add antialiasing by altering the ray_direction here
				// Genere deux gaussienne independante N(0,1) 
				// deux uniforme independantes sur (0,1)
				double r1 = uniform(engine[0]); //better uniform(engine[omp_get_thread_num()])
				double r2 = uniform(engine[0]);
				// 2 rv N(0,1) independantes 
				double new_gauss_x = x + sqrt(-2*log(r1))*cos(2*M_PI*r2); 
				double new_gauss_y = y + sqrt(-2*log(r1))*sin (2*M_PI*r2); // stdev ???
				// TODO (lab 2) : add depth of field effect by altering the ray origin (and direction) here

				Vector new_gauss_direction(new_gauss_x, new_gauss_y ,z);
				new_gauss_direction.normalize();

				Ray new_gauss_ray(scene.camera_center, new_gauss_direction);
				pixelColor = operator+(pixelColor,scene.getColor(new_gauss_ray,0)); 
			}
	

			color = pixelColor / NB_PATHS;

			image[(i * W + j) * 3 + 0] = std::min(255., std::max(0., 255. * std::pow(color[0] / 255., 1. / scene.gamma)));
			image[(i * W + j) * 3 + 1] = std::min(255., std::max(0., 255. * std::pow(color[1] / 255., 1. / scene.gamma)));
			image[(i * W + j) * 3 + 2] = std::min(255., std::max(0., 255. * std::pow(color[2] / 255., 1. / scene.gamma)));
		}
	}
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}