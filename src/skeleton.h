/*
 * mesh.h
 *
 * A skeleton class for ik and mesh skinning
 *
 *  Created on: March 25, 2011
 *      Author: jima
 */

#ifndef SKELETON_H_
#define SKELETON_H_

#include "main.h"
#include "mesh.h"

#include <vector>

using namespace std;

enum {IK_CCD = 0, IK_JACOBIAN, IK_NUMMODES};

// Skeleton joints.  Include info on the bone connecting them to their parent.
// This assumes a tree structure, so one parent but multiple children per joint.
// Note the joints have redundant information -- orientations *and* positions
//  -- this is because it's often convenient to set one and use it to regenerate the other.
//  (the skeleton class has functions for doing the 'regenerate the other' bit)
struct Joint {
    Joint() {}
    Joint(int parent, vec3 posn) : parent(parent), length(0), posn(posn) {}

    int parent; // parent (-1 if root)
    vector<int> children; // any children

    double length; // distance to parent (0 if root)
    quat orient; // change in orientation from parent joint to this joint
    
    // use 'updateJointPosnAndFrame' to set the below after changing orient:
    vec3 posn; // joint position
    quat l2w; // local-to-world transform


    // helpers to convert in and out of 'bone space' (assuming posn and l2w are set)
    inline vec3 worldToLocal(vec3 w) { return l2w.conjugate().rotate(w-posn); }
    inline vec3 localToWorld(vec3 local) { return posn+l2w.rotate(local); }


};

// Store enough data to replay animations you create
struct Animation
{
    vector< vector<quat> > orientations;
    // you may want to add more data here, eg perhaps root position if you animate that
    // save posn's?

    void setJoints(vector<Joint> &joints, double frame);

    // save the orientations from a joint vector as a frame of animation
    void addAsFrame(vector<Joint> &joints) {
        orientations.push_back(vector<quat>());
        for (size_t i = 0; i < joints.size(); i++) {
            orientations.back().push_back(joints[i].orient);
        }
        // you may want to save more data here, eg if you root position if you animate that
    }
    
    int numFrames() { return orientations.size(); }
    
    void playback(vector<Joint> &joints, double interp);

    void clear() { orientations.clear(); }
};

// forward declaration to allow skeleton functions that act on mesh (eg, updateSkin)
class Mesh;

// The main skeleton class.
// Mostly just a vector of joints, and index to the root joint
// plus a lot of helpful functions.
class Skeleton
{
public:
    // load skeleton from a pinocchio .out skeleton file
    void loadPinocchioFile(const char *skelfile);

    // load bone weights from a pinocchio .out attachment file.
    // (use threshWeight to optionally discard very small weights.)
    void initBoneWeights(const char *skinfile, Mesh &mesh, double threshWeight = .01);

    void render(int highlightChainToJoint);
    void updateSkin(Mesh &mesh);

    // call setupView() before calling pickJoint or getPos
    int pickJoint(double &depth, vec2 mouse, double selectionRadius = 10.0);    // gives joint index and (by ref) joint depth
    vec3 getPos(vec2 mouse, double depth);    // get a 3d pos based on mouse and a target depth

    // do ik to put the given joint index at the given position
    void inverseKinematics(int j, vec3 pos, int mode);
	
    void dance(Animation &a, Mesh &mesh, double t, bool changeMoves);
    
	// Dance moves!
    void headbang(bool down);
    void lean(bool forward);
	void rotate(bool clockwise);
    void tiltBody(bool left);
    void tiltHead(bool left);

    // get chain of joints from j to root, useful for ik
    vector<Joint*> getChain(int j);
    // update just the positions and frames in the given chain
    void updateChainFrames(vector<Joint*> &chain);

    // offset and scale joint positions; useful when adjusting mesh scale/offset
    void offsetAndScale(vec3 offset, double scale);

    // updates positions and frames from orientations
    void updateJoints() { updateJointPosnAndFrame(root); }
    
    void clear() { joints.clear(); }
    
    vector<Joint> &getJointArray() { return joints; }

    void saveRestPose();
    void resetPose();

    int getRoot() { return root; }

private:
    vector<quat> restPose; // remember the rest pose just for resetting
    vector<Joint> joints; // all joints
    int root; // index of the root joint

    // helper to get rest orientation of a joint
    quat getRestOrientation(Joint &j);
    // recursive function to set positions and frames from orientations
    void updateJointPosnAndFrame(int joint, quat acc = quat());
    // recursive function to set orientations and frames from positions
    // (eg for particle ik, and also to initialize frames after reading a pinocchio file)
    void updateOrientationAndFrameFromPosn(int joint, quat acc = quat());

};

#endif

