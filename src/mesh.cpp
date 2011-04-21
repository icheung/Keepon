#include "mesh.h"

void Mesh::centerAndScale(Skeleton &skel, double scale) {
    if (verts.empty())
	    return;

    vec3 maxp = verts[0].p, minp = verts[0].p;
    for (vector<Vert>::iterator it = verts.begin(); it != verts.end(); ++it) {
	    maxp = max(it->p, maxp); // max and min def'd in algebra3.h take the max or min componentwise from each vector
	    minp = min(it->p, minp);
    }
    vec3 center = (maxp+minp)*.5;
    vec3 size = maxp-minp;
    double maxSizeInv = MAX(size[0],MAX(size[1],size[2]));
    if (maxSizeInv == 0) // mesh is just one point
	    return;
    maxSizeInv = 1.0/maxSizeInv;
    for (vector<Vert>::iterator it = verts.begin(); it != verts.end(); ++it) {
	    it->p = (it->p-center)*maxSizeInv*scale;
    }
    skel.offsetAndScale(-center, maxSizeInv*scale);
}


// just use the first two edges
vec3 Mesh::getNormal(int f) {
	vec3 e1 = v(f,0).p - v(f,1).p;
	vec3 e2 = v(f,2).p - v(f,1).p;
    vec3 n = -e1 ^ e2;
    return n.normalize();
}

void Mesh::render() {
	for (size_t i = 0; i < faces.size(); ++i) {
		glBegin(GL_POLYGON);
        vec3 n = getNormal((int)i);
        glNormal3dv(&n[0]);
        int count = (int)faces[i].vi.size();
		for (int ind = 0; ind < count; ind++) {
			glVertex3dv(&v((int)i,ind).p[0]);
		}
		glEnd();
	}
}

namespace { 
    int getNValues(stringstream &ss, vector<int> &values, char delim = '/') {
	    values.clear();
	    string sblock;
	    if (ss >> sblock) {
		    stringstream block(sblock);
		    string s;
		    int value;
		    while (getline(block, s, delim)) {
			    stringstream valuestream(s);
			    if (valuestream >> value)
				    values.push_back(value);
                else
                    values.push_back(-1);
		    }
	    }
	    return (int)values.size();
    }
}

bool Mesh::loadFile(string file)
{
	clear();
	ifstream f(file.c_str());
	if (!f) {
		cerr << "Couldn't open file: " << file << endl;
		return false;
	}
	string line;
	vector<int> first, second;
	vector<int> orig;
	while (getline(f,line)) {
		if (line.empty())
			continue;
		stringstream ss(line);
		string op;
		ss >> op;
		if (op.empty() || op[0] == '#')
			continue;
		if (op == "v") {
			vec3 v;
			ss >> v;
			verts.push_back(Vert(v));
		}
		if (op == "f")
		{
            faces.push_back(Face());
			Face &f = faces.back();			
			if (!getNValues(ss, first))
				continue;
			orig = first;
			while (getNValues(ss, second)) {
                f.addVertex(first, -1);
				first = second;
			}
            f.addVertex(first, -1);
		}
	}
	return true;
}

