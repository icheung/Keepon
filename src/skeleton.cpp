#include "skeleton.h"

#include <fstream>
#include <string>
#include <sstream>

#define _USE_MATH_DEFINES

void Animation::setJoints(vector<Joint> &joints, double frame) {
    // TODO: Set joints by interpolating frames

	// If the mouse moves offscreen to the right, take quats from last frame.
	// range of x, is from 0 to number of frames -1 (remember: orientations.size() == number of frames)
    if (frame >= orientations.size() - 1) {
        vector<quat> offScreen = orientations.back();
        unsigned int iter;
        for (iter = 0; iter < joints.size(); ++iter)
            joints.at(iter).orient = offScreen.at(iter);
        return;
    }
    
    vector<quat> quatsStart = orientations.at(floor(frame));
    vector<quat> quatsEnd = orientations.at(floor(frame) + 1);
    unsigned int iter;
    for (iter = 0; iter < joints.size(); ++iter) {
        quat baseQuat = quatsStart.at(iter);
        quat nextQuat = baseQuat.nlerp(
            quatsEnd.at(iter), frame - floor(frame)
        );
        joints.at(iter).orient = nextQuat;
    }
}


// note: if you only do one ik implementation, you can ignore the "method" argument here.
void Skeleton::inverseKinematics(int j, vec3 pos, int method) {
    // get chain of joints going back to root
    vector<Joint*> chain = getChain(j);
    // note you can use this function after changing joint orientations in
    // the chain to find the new joint positions & frames:
    //    void updateChainFrames(vector<Joint*> &chain);
    
    if (chain.size() < 2)
        return;
    int rounds;
    for (rounds = 0; rounds < 7; ++rounds) {
        unsigned int iter;
        Joint * tip = chain.front();
        for (iter = 0; iter < chain.size() - 1; ++iter) {
            Joint * child = chain.at(iter);
            Joint * parent = chain.at(iter + 1);
        
            vec3 start = parent->worldToLocal(tip->posn - parent->posn);
            vec3 end = parent->worldToLocal(pos - parent->posn);
            vec3 mid = start + (end - start) / 2;
            quat tempQuat = quat::getRotation(
                //start.normalize(), end.normalize()
                start.normalize(), mid.normalize()
            );
            child->orient = tempQuat * child->orient;
            updateChainFrames(chain);
        }
    }
}



void Skeleton::dance(Animation &a, Mesh &mesh, double t, bool changeMoves){
	if (t == 0.0) {
		a.clear();
		resetPose();
		
		int m = rand() % 4;
		switch (m) {
			case 0:
				a.addAsFrame(getJointArray());
				rotate(true);
				headbang(true);
				a.addAsFrame(getJointArray());
				rotate(false);
				headbang(false);
				a.addAsFrame(getJointArray());
				break;
			case 1:
				a.addAsFrame(getJointArray());
				for (int i=0;i<6;i++)
					rotate(true);
	    		a.addAsFrame(getJointArray());
				for (int i=0;i<6;i++)
					rotate(false);
	    		a.addAsFrame(getJointArray());
				break;
			case 2:
				a.addAsFrame(getJointArray());
				tiltBody(true);
				lean(true);
	    		a.addAsFrame(getJointArray());
				tiltBody(false);
				lean(false);
	    		a.addAsFrame(getJointArray());
				break;
			case 3:
				a.addAsFrame(getJointArray());
				tiltHead(false);
				headbang(false);
		   		a.addAsFrame(getJointArray());
				tiltHead(true);
				headbang(true);
		   		a.addAsFrame(getJointArray());
				break;		
		}
	}

    if (a.numFrames() > 1)
        a.setJoints(getJointArray(), t);
    updateSkin(mesh);
}

// helper
void Animation::playback(vector<Joint> &joints, double interp) {

}

/*** DANCE MOVES! ***/

void Skeleton::headbang(bool down)
{
    vector<Joint*> chain = getChain(2);
    Joint * head_joint = chain.at(0); // head
    
    // 15 degrees
    vec3 hstart = (down) ? vec3(0.0,1.0,0.0) : vec3(0.0,0.966,0.259);
    vec3 hend = (down) ? vec3(0.0,0.966,0.259) : vec3(0.0,1.0,0.0);
    
    quat hQuat = quat::getRotation(hstart.normalize(),hend.normalize());
    head_joint->orient = hQuat * head_joint->orient;
    updateChainFrames(chain);
}

void Skeleton::lean(bool forward)
{
    vector<Joint*> chain = getChain(1);
    Joint * body_joint = chain.at(0); // body
    
    // 15 degrees
    vec3 start = (forward) ? vec3(0.0,1.0,0.0) : vec3(0.0,0.966,0.259);
    vec3 end = (forward) ? vec3(0.0,0.966,0.259) : vec3(0.0,1.0,0.0);
    
    quat q = quat::getRotation(start.normalize(),end.normalize());
    body_joint->orient = q * body_joint->orient;
    updateChainFrames(chain);
}

void Skeleton::rotate(bool clockwise)
{
    vector<Joint*> chain = getChain(0);
    Joint * base_joint = chain.at(0); // base
    
    // 15 degrees
    vec3 start = (clockwise) ? vec3(0.259,0.0,0.966) : vec3(0.0,0.0,1.0);
    vec3 end = (clockwise) ? vec3(0.0,0.0,1.0) : vec3(0.259,0.0,0.966);
    
    quat tQuat = quat::getRotation(start.normalize(),end.normalize());
    base_joint->orient = tQuat * base_joint->orient;
    updateChainFrames(chain);
}


void Skeleton::tiltBody(bool left)
{
    vector<Joint*> chain = getChain(1);
    Joint * body_joint = chain.at(0); // body
    
    // 15 degrees
    vec3 start = (left) ? vec3(0.0,1.0,0.0) : vec3(-0.259,0.966,0.0);
    vec3 end = (left) ? vec3(-0.259,0.966,0.0) : vec3(0.0,1.0,0.0);
    
    quat q = quat::getRotation(start.normalize(),end.normalize());
    body_joint->orient = q * body_joint->orient;
    updateChainFrames(chain);
}

void Skeleton::tiltHead(bool left)
{
    vector<Joint*> chain = getChain(2);
    Joint * head_joint = chain.at(0); // head
    
    // 15 degrees
    vec3 start = (left) ? vec3(0.0,1.0,0.0) : vec3(-0.259,0.966,0.0);
    vec3 end = (left) ? vec3(-0.259,0.966,0.0) : vec3(0.0,1.0,0.0);
    
    quat q = quat::getRotation(start.normalize(),end.normalize());
    head_joint->orient = q * head_joint->orient;
    updateChainFrames(chain);
}

/*** END OF DANCE MOVES ***/

void Skeleton::updateSkin(Mesh &mesh) {
    updateJointPosnAndFrame(root);

    vector<Vert> &meshVerts = mesh.verts;

    int numJoints;
    vec3 tempJointLocalPos;
    vec3 tempJointWorldPos;

    for (vector<Vert>::iterator it = meshVerts.begin(); it != meshVerts.end(); ++it) {
        //IMPLEMENT_ME(__FILE__,__LINE__);
        it->p = vec3(0,0,0); // first test
        
        numJoints = it->weights.size();
        
        for (int i=0; i<numJoints; i++){
            // mesh vertex position local
            tempJointLocalPos = it->weights[i].jointLocalPos;
            //cout << "mesh local pos:" << tempJointLocalPos << endl;
            // mesh vertex position in the world
            tempJointWorldPos = joints.at(it->weights[i].joint).localToWorld(tempJointLocalPos);
            //cout << "mesh world pos: " << tempJointWorldPos << endl;
            // multiply by weight
            tempJointWorldPos *= it->weights[i].weight;
            //cout << "mesh mult weight pos: " << tempJointWorldPos << endl;
            // add to itself
            it->p += tempJointWorldPos;
        }
    }
}


void Skeleton::saveRestPose() {
    restPose.resize(joints.size());
    for (size_t i = 0; i < joints.size(); i++) {
        restPose[i] = joints[i].orient;
    }
}
void Skeleton::resetPose() {
    for (size_t i = 0; i < joints.size(); i++) {
        joints[i].orient = restPose[i];
    }
    updateJoints();
}

// Load a skeleton file
void Skeleton::loadPinocchioFile(const char *skelfile) {
    clear();
    ifstream f(skelfile);

    string line;
	while (getline(f,line)) {
		if (line.empty())
			continue;
		stringstream ss(line);
		int id, parent;
        vec3 posn;
        if (ss >> id >> posn[0] >> posn[1] >> posn[2] >> parent) {
            assert(id == int(joints.size()));
            joints.push_back(Joint(parent, posn));
            if (parent == -1)
                root = id;
        }
    }

    // init lengths and children array
    for (size_t i = 0; i < joints.size(); i++) {
        int parent = joints[i].parent;
        if (parent > -1) {
            joints[i].length = (joints[i].posn - joints[parent].posn).length();
            joints[parent].children.push_back(int(i));
        }
    }

    updateOrientationAndFrameFromPosn(root);

    saveRestPose();
}
// Load a bone weight file (attaching the skeleton to a mesh)
void Skeleton::initBoneWeights(const char *skinfile, Mesh &mesh, double threshWeight) {
    ifstream f(skinfile);

    vector<Vert> &meshVerts = mesh.verts;
    updateJointPosnAndFrame(root);

    // expect one line per vertex
    string line;
    size_t vertind = 0;
	while (getline(f,line)) {
		if (line.empty())
			continue;
		stringstream ss(line);
        
        vector<BoneWeight> &weights = meshVerts[vertind].weights; 

        int id = 0;
        double weight;
        double totalWeight = 0;
        while (ss >> weight) {
            if (id == root) id++;
            if (weight > threshWeight) {
                totalWeight += weight;
                weights.push_back(BoneWeight(id, weight, joints[id].worldToLocal(meshVerts[vertind].p)));
            }
            id++;
        }
        
        assert(totalWeight > 0); // double check you didn't throw away too much ...

        // renormalize weights so they sum to 1 after discarding all small weights.
        for (vector<BoneWeight>::iterator it = weights.begin(); it != weights.end(); ++it) {
            it->weight /= totalWeight;
        }

        vertind++;
    }
}

// offset and scale joint positions; useful when adjusting mesh scale/offset
void Skeleton::offsetAndScale(vec3 offset, double scale) {
    for (vector<Joint>::iterator it = joints.begin(); it != joints.end(); ++it) {
        it->posn += offset;
        it->posn *= scale;
        it->length *= scale;
    }
}

// update joint positions and frames using joint orientations
void Skeleton::updateJointPosnAndFrame(int joint, quat acc) {
    Joint &j = joints[joint];
    acc = acc * j.orient;
    j.l2w = acc;

    if (j.parent != -1) {
        Joint &p = joints[j.parent];
        vec3 bone(0,j.length,0);
        bone = acc.rotate(bone);
        vec3 oldposn = j.posn;
        j.posn = p.posn + bone;
    }
    
    for (vector<int>::iterator it = j.children.begin(); it != j.children.end(); ++it) {
        int ch = *it;
        updateJointPosnAndFrame(ch, acc);
    }
}
// update joint orientations and frames using joint positions
void Skeleton::updateOrientationAndFrameFromPosn(int joint, quat acc) {
    Joint &j = joints[joint];

    if (j.parent != -1) {
        Joint &p = joints[j.parent]; // parent has orientation and frame set
        vec3 boneDir = p.worldToLocal(j.posn);
        j.orient = quat::getRotation(vec3(0,1,0), boneDir);
        acc = acc * j.orient;
    }
    j.l2w = acc;

    for (vector<int>::iterator it = j.children.begin(); it != j.children.end(); ++it) {
        int ch = *it;
        updateOrientationAndFrameFromPosn(ch, acc);
    }
}

void Skeleton::render(int highlightChainToJoint) {
    updateJointPosnAndFrame(root);

    glDisable(GL_LIGHTING); // just draw plain colored lines
    glDisable(GL_DEPTH_TEST); // make it show through the mesh

    if (highlightChainToJoint) {
        glColor3d(1,0,0);
        glLineWidth(10);
        glBegin(GL_LINE_STRIP);
        vector<Joint*> chain = getChain(highlightChainToJoint);
        for (size_t i = 0; i < chain.size(); i++) {
            glVertex3dv(&chain[i]->posn[0]);
        }
        glEnd();
    }
    
    glColor3d(1,1,0);
    glLineWidth(3);
    glBegin(GL_LINES);
    for (size_t i = 0; i < joints.size(); i++) {
        vec3 posn = joints[i].posn;
        int parent = joints[i].parent;
        if (parent > -1) {
            vec3 pposn = joints[parent].posn;
            glVertex3dv(&posn[0]);
            glVertex3dv(&pposn[0]);
        }
    }
    glEnd();

    glPointSize(10);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < joints.size(); i++) {
        glVertex3dv(&joints[i].posn[0]);
    }
    glEnd();

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
}

// choose a joint within the selection radius of the mouse
int Skeleton::pickJoint(double &depth, vec2 mouse, double selectionRadius) {
    double modelview[16], projection[16];
    int viewport[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    mouse[1] = viewport[3] - mouse[1];

    int bestJoint = -1;
    double bestDist = selectionRadius*selectionRadius;
    for (size_t i = 0; i < joints.size(); ++i) {
        vec3 p = joints[i].posn;
        vec2 s;
        double sz;
        gluProject(p[0], p[1], p[2], 
            modelview, projection, viewport, 
            &s[0], &s[1], &sz);
        vec2 diff = mouse - s;
        
        if (diff.length2() <= bestDist) {
            bestDist = diff.length2();
            bestJoint = int(i);
            depth = sz;
        }
    }

    return bestJoint;
}
// project a screen pos into the world at the given depth 
vec3 Skeleton::getPos(vec2 mouse, double depth) {
    double modelview[16], projection[16];
    int viewport[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    mouse[1] = viewport[3] - mouse[1];

    double x,y,z;
    gluUnProject(mouse[0], mouse[1], depth, 
                 modelview, projection, viewport,
                 &x, &y, &z);
    return vec3(x,y,z);
}

// get chain of joints from j to root, useful for ik
vector<Joint*> Skeleton::getChain(int j) {
    vector<Joint*> chain;
    int nextj = j;
    while (nextj != -1) {
        chain.push_back(&joints[nextj]);
        nextj = joints[nextj].parent;
    }
    return chain;
}
// update just the positions and frames in the given chain
void Skeleton::updateChainFrames(vector<Joint*> &chain) {
    quat acc;
    // root is at end of chain; go from root to front
    for (int i = int(chain.size())-1; i >= 0; i--) {
        Joint &j = *chain[i];
        acc = acc * j.orient;
        j.l2w = acc;
        if (j.parent != -1) {
            Joint &p = joints[j.parent];
            vec3 bone(0,j.length,0);
            bone = acc.rotate(bone);
            j.posn = p.posn + bone;
        }
    }
}
