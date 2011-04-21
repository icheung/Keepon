/*
 * mesh.h
 *
 * A simple mesh class
 *
 *  Created on: March 10, 2011
 *      Author: jima
 */

#ifndef MESH_H_
#define MESH_H_

#include "main.h"

#include <string>
#include <sstream>

using namespace std;

// mesh face
struct Face {
    // vertex, texture and normal indices, in order 
    vector<int> vi;

    Face() { vi.reserve(3); }

    void addVertex(vector<int> info, int offset=0) {
        vi.push_back(info[0]+offset);
    }
};

// describes attachment of vertex to a joint
struct BoneWeight {
    int joint; // joint whose frame we attach to
    double weight; // weight with which we attach
    vec3 jointLocalPos; // the position of the vertex in joint-local space

    BoneWeight(int joint, double weight, vec3 jointLocalPos) 
        : joint(joint), weight(weight), jointLocalPos(jointLocalPos) {}
};

// mesh vertex
struct Vert {
	vec3 p;
    vector<BoneWeight> weights;
    int temp;

	Vert(vec3 v) : p(v) {}
};

class Skeleton;

class Mesh {
	vector<Vert> verts;
	vector<Face> faces;

    // skeleton has vertex access to do skinning
    friend class Skeleton; 

public:

	void clear() {
		verts.clear(); faces.clear();
	}

    // centers the mesh and scales it to the given scale.  Also scales the skeleton similarly.
	void centerAndScale(Skeleton &skel, double scale = 1.0);
	
    // face normal
    vec3 getNormal(int f);

    // an inefficient render routine
	void render();
    
	bool loadFile(string file);

    // --- some helpers to access the mesh:
    // get the ii'th vert of the f'th face
	inline Vert &v(int f, int ii) { return verts[faces[f].vi[ii]]; }
    // get the ii'th vert of the f'th face (by index)
	inline int vind(int f, int ii) { return faces[f].vi[ii]; }
    
};



#endif

