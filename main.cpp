#define _CRT_SECURE_NO_WARNINGS 1
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <map>
#include <string>
#include <fstream>
#include <algorithm>
#include <list>
#include <limits>
#include <omp.h> 

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

	bool intersect(const Ray& ray, Vector& P, double &t, Vector& N) const {
		double delta = dot(ray.u, ray.O-C)*dot(ray.u, ray.O-C) - ((ray.O-C).norm2() - R*R); 
		if (delta < 0){
			return false;
		}
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


class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1) {
		vtx[0] = vtxi; vtx[1] = vtxj; vtx[2] = vtxk;
		uv[0] = uvi; uv[1] = uvj; uv[2] = uvk;
		n[0] = ni; n[1] = nj; n[2] = nk;
		this->group = group;
	};
	int vtx[3];
	int uv[3];
	int n[3];
	int group;
};


class TriangleMesh : public Object {
	struct BVHNode {
		Vector Bmin;
		Vector Bmax;

		int start;   // included 
		int end;     // excluded 

		BVHNode* left;
		BVHNode* right;

		BVHNode() {
			Bmin = Vector(1e9, 1e9, 1e9);
			Bmax = Vector(-1e9, -1e9, -1e9);
			start = 0;
			end = 0;
			left = nullptr;
			right = nullptr;
		}

		bool isLeaf() const {
			return left == nullptr && right == nullptr;
		}
	};

public:
	TriangleMesh(const Vector& albedo, bool mirror = false, bool transparent = false) 
		: ::Object(albedo, mirror, transparent) {
		Bmin = Vector(1e9, 1e9, 1e9);
		Bmax = Vector(-1e9, -1e9, -1e9);
		root = nullptr;
	};

	~TriangleMesh() {
		deleteBVH(root);
	}

	void deleteBVH(BVHNode* node) {
		if (!node) return;
		deleteBVH(node->left);
		deleteBVH(node->right);
		delete node;
	}


	void computeBBox(BVHNode* node, int start, int end) {
		node->Bmin = Vector(1e9, 1e9, 1e9);
		node->Bmax = Vector(-1e9, -1e9, -1e9);
		for (int i = start; i < end; i++) {
			for (int k = 0; k < 3; k++) {
				for (int v = 0; v < 3; v++) {
					double val = vertices[indices[i].vtx[v]][k];
					node->Bmin[k] = std::min(node->Bmin[k], val);
					node->Bmax[k] = std::max(node->Bmax[k], val);
				}
			}
		}
	}

	bool intersectBBox(const BVHNode* node, const Ray& ray, double best_t, double& t_enter) const {
		double tx0 = (node->Bmin[0] - ray.O[0]) / ray.u[0];
		double tx1 = (node->Bmax[0] - ray.O[0]) / ray.u[0];
		if (tx0 > tx1) std::swap(tx0, tx1);

		double ty0 = (node->Bmin[1] - ray.O[1]) / ray.u[1];
		double ty1 = (node->Bmax[1] - ray.O[1]) / ray.u[1];
		if (ty0 > ty1) std::swap(ty0, ty1);

		double tz0 = (node->Bmin[2] - ray.O[2]) / ray.u[2];
		double tz1 = (node->Bmax[2] - ray.O[2]) / ray.u[2];
		if (tz0 > tz1) std::swap(tz0, tz1);

		t_enter = std::max({tx0, ty0, tz0});
		double t_exit  = std::min({tx1, ty1, tz1});

		return (t_exit >= t_enter) && (t_exit >= 0) && (t_enter < best_t); // true if ray hits the box at distance < best_t
	}


	void buildBVH(BVHNode* node, int start, int end) {
		node->start = start;
		node->end   = end;
		computeBBox(node, start, end);

		// Diag of bounding box
		Vector diag = node->Bmax - node->Bmin;

		int longest_axis = 0;
		if (diag[1] > diag[longest_axis]) longest_axis = 1;
		if (diag[2] > diag[longest_axis]) longest_axis = 2;

		double middle = node->Bmin[longest_axis] + diag[longest_axis] * 0.5; // middle value along longest axis

		// partition triangles at middle of the longest axis
		int pivot_index = start;
		for (int i = start; i < end; i++) {
			// here compute barycenter of triangle i along the longest axis
			double centroid = (
				vertices[indices[i].vtx[0]][longest_axis] +
				vertices[indices[i].vtx[1]][longest_axis] +
				vertices[indices[i].vtx[2]][longest_axis]
			) / 3.0;

			if (centroid < middle) {
				std::swap(indices[i], indices[pivot_index]);
				pivot_index++;
			}
		}
		//stopping criterion
		if (pivot_index == start || pivot_index == end) return; // if all triangles are on the same side we stop recusrion
		if (end - start <= 5) return; //leaf threshold = 5 form notes

		//left and right children
		node->left  = new BVHNode();
		node->right = new BVHNode();
		buildBVH(node->left,  start,       pivot_index);
		buildBVH(node->right, pivot_index, end);
	}

	void buildBVH() {
		root = new BVHNode();
		buildBVH(root, 0, (int)indices.size());
	}




	bool intersectTriangle(int i, const Ray& ray, Vector& P, double& t, Vector& N) const {
		const Vector& A = vertices[indices[i].vtx[0]];
		const Vector& B = vertices[indices[i].vtx[1]];
		const Vector& C = vertices[indices[i].vtx[2]];

		Vector e1 = B - A;
		Vector e2 = C - A;

		Vector local_N = cross(e1, e2);
		double denom = dot(ray.u, local_N);
		if (std::abs(denom) < 1e-12) return false;

		double beta  =  dot(e2, cross(A - ray.O, ray.u)) / denom;
		double gamma = -dot(e1, cross(A - ray.O, ray.u)) / denom;
		double alpha =  1.0 - beta - gamma;
		double local_t = dot(A - ray.O, local_N) / denom;

		if (beta >= 0 && gamma >= 0 && alpha >= 0 && local_t > 1e-6 && local_t < t) {
			t = local_t;
			P = ray.O + t * ray.u;
			N = local_N;
			N.normalize();
			return true;
		}
		return false;
	}


	
	bool intersectBVH(const Ray& ray, Vector& P, double& t, Vector& N) const {
		if (!root) return false;

		double dummy_t;
		double best_t = t; // upper bound
		if (!intersectBBox(root, ray, best_t, dummy_t)) return false;

		bool hit = false;
		std::list<BVHNode*> nodes_to_visit;
		nodes_to_visit.push_back(root);

		while (!nodes_to_visit.empty()) {
			BVHNode* cur = nodes_to_visit.back();
			nodes_to_visit.pop_back();

			if (cur->left) {
				double t_left = std::numeric_limits<double>::max();
				double t_right = std::numeric_limits<double>::max();

				bool hit_left  = intersectBBox(cur->left,  ray, t, t_left);
				bool hit_right = intersectBBox(cur->right, ray, t, t_right);

				// depth-first traversal optimisation from the lecture : push the child that s further away first so the closer one is processed first
				if (hit_left && hit_right) {
					if (t_left <= t_right) {
						nodes_to_visit.push_back(cur->right);
						nodes_to_visit.push_back(cur->left);
					} else {
						nodes_to_visit.push_back(cur->left);
						nodes_to_visit.push_back(cur->right);
					}
				} else if (hit_left) {
					nodes_to_visit.push_back(cur->left);
				} else if (hit_right) {
					nodes_to_visit.push_back(cur->right);
				}
			} else {
				for (int i = cur->start; i < cur->end; i++) {
					if (intersectTriangle(i, ray, P, t, N)) {
						hit = true;
					}
				}
			}
		}
		return hit;
	}

	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const {
		// Fast bounding-box rejection first
		double tx0 = (Bmin[0] - ray.O[0]) / ray.u[0];
		double tx1 = (Bmax[0] - ray.O[0]) / ray.u[0];
		if (tx0 > tx1) std::swap(tx0, tx1);
		double ty0 = (Bmin[1] - ray.O[1]) / ray.u[1];
		double ty1 = (Bmax[1] - ray.O[1]) / ray.u[1];
		if (ty0 > ty1) std::swap(ty0, ty1);
		double tz0 = (Bmin[2] - ray.O[2]) / ray.u[2];
		double tz1 = (Bmax[2] - ray.O[2]) / ray.u[2];
		if (tz0 > tz1) std::swap(tz0, tz1);

		double t_enter = std::max({tx0, ty0, tz0});
		double t_exit  = std::min({tx1, ty1, tz1});

		if (t_exit < t_enter || t_exit < 0) return false;

		if (root) {
			return intersectBVH(ray, P, t, N);
		}

		// if no root we brute force it 
		bool res = false;
		for (int i = 0; i < (int)indices.size(); i++) {
			if (intersectTriangle(i, ray, P, t, N)) res = true;
		}
		return res;
	}

	// first scale and then translate the current object
	void scale_translate(double s, const Vector& t) {
		for (int i = 0; i < (int)vertices.size(); i++) {
			vertices[i] = vertices[i] * s + t;
		}

		Bmin = Vector(1e9, 1e9, 1e9);
		Bmax = Vector(-1e9, -1e9, -1e9);
		for (int i = 0; i < (int)vertices.size(); i++) {
			for (int k = 0; k < 3; k++) {
				Bmin[k] = std::min(Bmin[k], vertices[i][k]);
				Bmax[k] = std::max(Bmax[k], vertices[i][k]);
			}
		}
	}

	// read an .obj file
	void readOBJ(const char* obj) {
		std::ifstream f(obj);
		if (!f) return;

		std::map<std::string, int> mtls;
		int curGroup = -1, maxGroup = -1;

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
			line.erase(line.find_last_not_of(" \r\t\n") + 1);
			if (line.empty()) continue;

			const char* s = line.c_str();

			if (line.rfind("usemtl ", 0) == 0) {
				std::string matname = line.substr(7);
				auto result = mtls.emplace(matname, maxGroup + 1);
				if (result.second) curGroup = ++maxGroup;
				else curGroup = result.first->second;
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
			} else if (line[0] == 'f') {
				int i[4], j[4], k[4], offset, nn;
				const char* cur = s + 1;
				TriangleIndices t;
				t.group = curGroup;

				if ((nn = sscanf(cur, "%d/%d/%d %d/%d/%d %d/%d/%d%n", &i[0], &j[0], &k[0], &i[1], &j[1], &k[1], &i[2], &j[2], &k[2], &offset)) == 9) {
					setFaceVerts(t, i[0], i[1], i[2]); setFaceUVs(t, j[0], j[1], j[2]); setFaceNormals(t, k[0], k[1], k[2]);
				} else if ((nn = sscanf(cur, "%d/%d %d/%d %d/%d%n", &i[0], &j[0], &i[1], &j[1], &i[2], &j[2], &offset)) == 6) {
					setFaceVerts(t, i[0], i[1], i[2]); setFaceUVs(t, j[0], j[1], j[2]);
				} else if ((nn = sscanf(cur, "%d//%d %d//%d %d//%d%n", &i[0], &k[0], &i[1], &k[1], &i[2], &k[2], &offset)) == 6) {
					setFaceVerts(t, i[0], i[1], i[2]); setFaceNormals(t, k[0], k[1], k[2]);
				} else if ((nn = sscanf(cur, "%d %d %d%n", &i[0], &i[1], &i[2], &offset)) == 3) {
					setFaceVerts(t, i[0], i[1], i[2]);
				} else continue;

				indices.push_back(t);
				cur += offset;

				while (*cur && *cur != '\n') {
					TriangleIndices t2;
					t2.group = curGroup;
					if ((nn = sscanf(cur, " %d/%d/%d%n", &i[3], &j[3], &k[3], &offset)) == 3) {
						setFaceVerts(t2, i[0], i[2], i[3]); setFaceUVs(t2, j[0], j[2], j[3]); setFaceNormals(t2, k[0], k[2], k[3]);
					} else if ((nn = sscanf(cur, " %d/%d%n", &i[3], &j[3], &offset)) == 2) {
						setFaceVerts(t2, i[0], i[2], i[3]); setFaceUVs(t2, j[0], j[2], j[3]);
					} else if ((nn = sscanf(cur, " %d//%d%n", &i[3], &k[3], &offset)) == 2) {
						setFaceVerts(t2, i[0], i[2], i[3]); setFaceNormals(t2, k[0], k[2], k[3]);
					} else if ((nn = sscanf(cur, " %d%n", &i[3], &offset)) == 1) {
						setFaceVerts(t2, i[0], i[2], i[3]);
					} else { cur++; continue; }

					indices.push_back(t2);
					cur += offset;
					i[2] = i[3]; j[2] = j[3]; k[2] = k[3];
				}
			}
		}
	}


	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;

	Vector Bmin;
	Vector Bmax;
	BVHNode* root;
};


class Scene {
public:
	Scene() {};
	void addObject(const Object* obj) {
		objects.push_back(obj);
	}

	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N, int &object_id) const {
		bool res = false;
		for (size_t i = 0; i < objects.size(); ++i){
			double better_t = 1e9;
			Vector better_P;
			Vector better_N;

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

	Vector getColor(const Ray& ray, int recursion_depth) {
		if (recursion_depth >= max_light_bounce) return Vector(0, 0, 0);
		if (recursion_depth < 0) return Vector(0, 0, 0);

		Vector P, N;
		double t = 1e9;
		int object_id;
		if (intersect(ray, P, t, N, object_id)) {
			if (objects[object_id]->mirror) {
				Ray next_ray(P + 0.001*N, ray.u - 2*dot(ray.u, N)*N);
				return getColor(next_ray, recursion_depth+1);
			}

			if (objects[object_id]->transparent) {
				// refraction (optional)
			}

			Vector Lo(0., 0., 0.);
			Vector LP = light_position - P;
			double d = LP.norm();
			Vector new_ray_u = LP / d;
			Ray newRay(P + 0.0001*N, new_ray_u);

			Vector inter_P, inter_N;
			double inter_t = 1e9;
			int inter_index;
			bool is_intersection = intersect(newRay, inter_P, inter_t, inter_N, inter_index);
			bool is_shadow = is_intersection && (inter_t < d);
			double cos_theta = std::max(0., dot(N, new_ray_u));
			if (!is_shadow && cos_theta > 0) {
				Lo = (light_intensity / (4 * M_PI * d * d)) * (objects[object_id]->albedo / M_PI) * cos_theta;
			}

			// Indirect lighting
			double r1 = uniform(engine[omp_get_thread_num()]);
			double r2 = uniform(engine[omp_get_thread_num()]);
			double x = cos(2*M_PI*r1)*sqrt(1-r2);
			double y = sin(2*M_PI*r1)*sqrt(1-r2);
			double z = sqrt(r2);

			Vector T1;
			if (std::abs(N[0]) >= std::abs(N[2]) && std::abs(N[1]) >= std::abs(N[2])) {
				T1 = Vector(-N[1], N[0], 0);
			} else if (std::abs(N[2]) >= std::abs(N[0]) && std::abs(N[1]) >= std::abs(N[0])) {
				T1 = Vector(0, N[2], -N[1]);
			} else {
				T1 = Vector(-N[2], 0, N[0]);
			}
			T1.normalize();
			Vector T2 = cross(N, T1);
			Vector random_vector = x*T1 + y*T2 + z*N;
			random_vector.normalize();
			Ray random_ray(P + 0.0001*N, random_vector);
			Vector albedo = objects[object_id]->albedo;
			Vector v = getColor(random_ray, recursion_depth+1);
			Vector product(albedo[0]*v[0], albedo[1]*v[1], albedo[2]*v[2]);
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

	for (int i = 0; i < 32; i++) {
		engine[i].seed(i);
	}

	Sphere center_sphere(Vector(0, 0, 0), 10., Vector(0.8, 0.8, 0.8), true);
	Sphere wall_left(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
	Sphere wall_right(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
	Sphere wall_front(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere wall_behind(Vector(0, 0, 1000), 940, Vector(0.8, 0.2, 0.9));
	Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));
	Sphere floor(Vector(0, -1000, 0), 990, Vector(0.6, 0.5, 0.7));

	Scene scene;
	scene.camera_center = Vector(0, 0, 55);
	scene.light_position = Vector(-10, 20, 40);
	scene.light_intensity = 1E7;
	scene.fov = 60 * M_PI / 180.;
	scene.gamma = 2.2;
	scene.max_light_bounce = 8;

	TriangleMesh cat(Vector(0.8, 0.8, 0.8));
	cat.readOBJ("cat.obj");
	cat.scale_translate(0.6, Vector(0, -10, 0));
	cat.buildBVH();   // <-- Build the BVH after scale_translate

	scene.addObject(&cat);
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

			double x = j - W/2 + 0.5;
			double y = H/2 - i - 0.5;
			double z = -W / (2*tan(scene.fov/2));

			int NB_PATHS = 500;
			Vector pixelColor(0, 0, 0);
			for (int k = 0; k < NB_PATHS; k++) {
				double r1 = uniform(engine[omp_get_thread_num()]);
				double r2 = uniform(engine[omp_get_thread_num()]);
				double new_gauss_x = x + sqrt(-2*log(r1)) * cos(2*M_PI*r2);
				double new_gauss_y = y + sqrt(-2*log(r1)) * sin(2*M_PI*r2);

				Vector new_gauss_direction(new_gauss_x, new_gauss_y, z);
				new_gauss_direction.normalize();

				Ray new_gauss_ray(scene.camera_center, new_gauss_direction);
				pixelColor = operator+(pixelColor, scene.getColor(new_gauss_ray, 0));
			}

			color = pixelColor / NB_PATHS;

			image[(i * W + j) * 3 + 0] = std::min(255., std::max(0., 255. * std::pow(color[0] / 255., 1. / scene.gamma)));
			image[(i * W + j) * 3 + 1] = std::min(255., std::max(0., 255. * std::pow(color[1] / 255., 1. / scene.gamma)));
			image[(i * W + j) * 3 + 2] = std::min(255., std::max(0., 255. * std::pow(color[2] / 255., 1. / scene.gamma)));
		}
	}
	stbi_write_png("catimage.png", W, H, 3, &image[0], 0);

	return 0;
}

// to compile on mac : g++ -O3 -o raytracer main.cpp -std=c++17 -Xpreprocessor -fopenmp -I/opt/homebrew/opt/libomp/include -L/opt/homebrew/opt/libomp/lib -lomp