#include "main.h"
#include <queue>

using namespace FMOD;
using namespace std;

// viewport
struct Viewport {
    Viewport(): mousePos(0.0,0.0) { orientation = identity3D(); };
	int w, h; // width and height
	vec2 mousePos;
    mat4 orientation;
};

//****************************************************
// Global Variables
//****************************************************
Viewport viewport;
UCB::ImageSaver * imgSaver;
int frameCount = 0;
Mesh *mesh;
Skeleton *skel;
Animation *anim;

// music
bool startedMusic=false;
System  *fmodsys;
Sound   *fmodsnd;
Channel *fmodchn;
float *specL, *specR, *spec;
float *oldspecL;
float oldvalue;
bool firstNote=true;
bool music_paused=false;
queue<float> localSamples;
float localSoundEnergy = 0.0;
#define SPECLEN 1024

// these variables track which joint is under IK, and where
int ikJoint;
double ikDepth;

// ui modes
bool playanim = false;
int ik_mode = IK_CCD;

// saving animated gif
bool savingImages = false;
int endFrameCount = 0;

// A simple helper function to load a mat4 into opengl
void applyMat4(mat4 &m) {
	double glmat[16];
	int idx = 0;
	for (int j = 0; j < 4; j++) 
		for (int i = 0; i < 4; i++)
			glmat[idx++] = m[i][j];
	glMultMatrixd(glmat);
}

// setup the model view matrix for mesh rendering
void setupView() {
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    glTranslatef(0,0,-3);
    applyMat4(viewport.orientation);
}

//----------------------------------------------------------------------------
void updateLocalSamples()
{
    for (int iter = 0; iter < SPECLEN; iter++) {
        if (localSamples.size() >= 43 * SPECLEN) {
            localSoundEnergy -= abs(localSamples.front());
            localSamples.pop();
        }
        localSoundEnergy += abs(specL[iter] + specR[iter]);
        localSamples.push(abs(specL[iter] + specR[iter]));
    }
}

void parseMusic()
{
     bool paused;
     fmodchn->getPaused( &paused );
     if ( !paused ) {
        // frequency domain data
        //fmodchn->getSpectrum(specL, SPECLEN, 0, FMOD_DSP_FFT_WINDOW_BLACKMAN);
        //fmodchn->getSpectrum(specR, SPECLEN, 1, FMOD_DSP_FFT_WINDOW_BLACKMAN);
        
        // time domain data
        fmodchn->getWaveData(specL, SPECLEN, 0);
        fmodchn->getWaveData(specR, SPECLEN, 1);
        
        // treat the first note differently
        if (firstNote) {
            oldspecL = &oldvalue;
            *oldspecL = *specL;
            firstNote=false;
        }
        
        // Beat can be measured by checking a sample for
        // (instant sound energy - local average sound energy)
        // and checking if the deviation is significant.
        // http://www.flipcode.com/misc/BeatDetectionAlgorithms.pdf
        updateLocalSamples();
        if (localSamples.size() >= 43 * SPECLEN) {
            float instantSoundEnergy = 0.0;
            for (int iter = 0; iter < SPECLEN; iter++)
                instantSoundEnergy += abs(specL[iter] + specR[iter]);
            cout << "----------------" << endl;

            // Beat detection!
            if (instantSoundEnergy > localSoundEnergy * 2 * 1024 / 44100) {
                //cout << "*************************************" << endl;
                cout << "\nBEAT\n" << endl;
                //cout << "*************************************" << endl;
                vec3 test = (frameCount % 2) ? vec3(0, 0.42, 0.2) : vec3(0, 0.42, -0.2);
                skel->inverseKinematics(2, test, ik_mode);
                skel->updateSkin(*mesh);
            }
        }
    } else {
        memset( specL, 0, SPECLEN*sizeof(float) );
        memset( specR, 0, SPECLEN*sizeof(float) );  
    };
}

//----------------------------------------------------------------------------
/// You will be calling all of your drawing-related code from this function.
/// Nowhere else in your code should you use glBegin(...) and glEnd() except
/// code called from this method.
///
/// To force a redraw of the screen (eg. after mouse events or the like)
/// simply call glutPostRedisplay();
void display() {
    //Clear Buffers
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    setupView();
    parseMusic();
    mesh->render();

    if (!playanim) { // if not playing an animation draw the skeleton
        skel->render(ikJoint);
    }

    //Now that we've drawn on the buffer, swap the drawing buffer and the displaying buffer.
    glutSwapBuffers();
    
    if (savingImages) {
        if (frameCount < endFrameCount)
            imgSaver->saveFrame();
        else
            savingImages = false;
    }
}


//----------------------------------------------------------------------------
/// \brief	Called when the screen gets resized.
/// This gives you the opportunity to set up all the relevant transforms.
///
void reshape(int w, int h) {
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, ((double)w / MAX(h, 1)), 1.0, 100.0);
	//glOrtho(-10,10,-10,10,1,100);

    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

//----------------------------------------------------------------------------
/// Called to handle keyboard events.
void myKeyboardFunc (unsigned char key, int x, int y) {
	switch (key) {
		case 27:			// Escape key
			exit(0);
			break;
        case 'S':
        case 's':
            imgSaver->saveFrame();
            break;
        case 'a': // add a frame to your animation
        case 'A':
            anim->addAsFrame(skel->getJointArray());
            break;
        case 'r': // reset skeleton pose
        case 'R':
            skel->resetPose();
            skel->updateSkin(*mesh);
            break;
        case 'p': // toggle animation playback mode
        case 'P':
            playanim = !playanim;
            break;
        case 'm': // switch through ik methods
        case 'M':
            ik_mode = (ik_mode+1) % IK_NUMMODES;
            break;
        case 'g':
        case 'G':
            savingImages = true;
            endFrameCount = frameCount + 100;
            break;
        case ' ':
            music_paused = !music_paused;
            fmodchn->setPaused(music_paused);
            break;
	}
}

void myMouseFunc(int button, int state, int x, int y) {
    setupView();
    ikJoint = skel->pickJoint(ikDepth, vec2(x,y));
}

// helper to set joints from the animation file, using the mouse x to select the animation frame
void setJointsByAnimation(int x) {
    if (playanim && anim->orientations.size() > 1) {
        double t = double(anim->orientations.size()-1) * double(x)/double(glutGet(GLUT_WINDOW_WIDTH));
        anim->setJoints(skel->getJointArray(), t);
        skel->updateSkin(*mesh);
    }
}

//----------------------------------------------------------------------------
/// Called whenever the mouse moves while a button is pressed
void myActiveMotionFunc(int x, int y) {
    if (ikJoint != -1 && !playanim) { // if a joint is selected for ik and we're not in animation playback mode, do ik
        vec3 target = skel->getPos(vec2(x,y), ikDepth);
        //cout << target << endl;
        skel->inverseKinematics(ikJoint, target, ik_mode);
        skel->updateSkin(*mesh);
    } else { // else mouse movements update the view
        // Rotate viewport orientation proportional to mouse motion
        vec2 newMouse = vec2((double)x / glutGet(GLUT_WINDOW_WIDTH),(double)y / glutGet(GLUT_WINDOW_HEIGHT));
        vec2 diff = (newMouse - viewport.mousePos);
        double len = diff.length();
        if (len > .001) {
            vec3 axis = vec3(diff[1]/len, diff[0]/len, 0);
            viewport.orientation = rotation3D(axis, 180 * len) * viewport.orientation;
        }

        //Record the mouse location for drawing crosshairs
        viewport.mousePos = newMouse;
    }
    
    //Force a redraw of the window.
    glutPostRedisplay();
}

//----------------------------------------------------------------------------
/// Called whenever the mouse moves without any buttons pressed.
void myPassiveMotionFunc(int x, int y) {
    ikJoint = -1;

    setJointsByAnimation(x);

    //Record the mouse location for drawing crosshairs
    viewport.mousePos = vec2((double)x / glutGet(GLUT_WINDOW_WIDTH),(double)y / glutGet(GLUT_WINDOW_HEIGHT));

    //Force a redraw of the window.
    glutPostRedisplay();
}

//----------------------------------------------------------------------------
/// Called to update the screen at 30 fps.
void frameTimer(int value) {
    frameCount++;
    glutPostRedisplay();
    glutTimerFunc(1000/30, frameTimer, 1);
}

//----------------------------------------------------------------------------
void ERRCHECK(FMOD_RESULT result)
{
    if (result != FMOD_OK)
    {
        printf("FMOD error! (%d) %s\n", result, FMOD_ErrorString(result));
        exit(-1);
    }
}

//----------------------------------------------------------------------------
void initFMOD(char * path)
{
    System_Create(&fmodsys);
    fmodsys->init(1,FMOD_INIT_NORMAL,0);
    //cout << "\nfile: " << path << endl;
    fmodsys->createSound(path,
        FMOD_SOFTWARE | FMOD_2D | FMOD_CREATESTREAM,
        0, &fmodsnd);
    fmodsys->playSound(FMOD_CHANNEL_FREE,fmodsnd,true,&fmodchn);
    specL = (float*)malloc(SPECLEN*sizeof(float));
    specR = (float*)malloc(SPECLEN*sizeof(float));
    //tmp
    unsigned int len;
    fmodchn->setVolume(1);
    fmodsnd->getLength( &len, FMOD_TIMEUNIT_MS );
    fmodchn->setPosition( (int)(len*0.0), FMOD_TIMEUNIT_MS );
};

//----------------------------------------------------------------------------

/// Initialize the environment
int main(int argc,char** argv) {
    if (argc < 2) {
        cout << "Usage: keepon [audio file]" << endl;
        exit(1);
    }
    
	//Initialize OpenGL
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH);

	//Set up global variables
	viewport.w = 600;
	viewport.h = 600;

	//Initialize the screen capture class to save BMP captures
	//in the current directory, with the prefix "keepon"
	imgSaver = new UCB::ImageSaver("./", "keepon");

	//Create OpenGL Window
	glutInitWindowSize(viewport.w,viewport.h);
	glutInitWindowPosition(0,0);
	glutCreateWindow("CS184 Framework");

	//Register event handlers with OpenGL.
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(myKeyboardFunc);
	glutMotionFunc(myActiveMotionFunc);
	glutPassiveMotionFunc(myPassiveMotionFunc);
    glutMouseFunc(myMouseFunc);
    frameTimer(0);

    glClearColor(.4,.2,1,0);

    // set some lights
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .2f, .5f, .5f };
       float pos[4] = { 0, 5, -5, 0 };
       glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT1, GL_POSITION, pos);
       glEnable(GL_LIGHT1);
    }
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .5f, .2f, .5f };
       float pos[4] = { 0, 0, 0, 1 };
       glLightfv(GL_LIGHT2, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT2, GL_POSITION, pos);
       glEnable(GL_LIGHT2);
    }
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    // load a mesh
    mesh = new Mesh();
    mesh->loadFile("keepon.obj");
    // load a matching skeleton
    skel = new Skeleton();
    skel->loadPinocchioFile("skeleton.out");
    mesh->centerAndScale(*skel);
    // load the correspondence between skeleton and mesh
    skel->initBoneWeights("attachment.out", *mesh);
    skel->updateSkin(*mesh);
    // start a new animation
    anim = new Animation();

    // note the .out files loaded above were made using pinocchio
    //  -- a neat free tool for auto-skinning a mesh
    // get it here: http://www.mit.edu/~ibaran/autorig/pinocchio.html
    // you can use it to easily replace the default mesh with a mesh
    // of your own making

	/*************************************************
	 GET YOUR GROOVE ON -- MUSACK INITIALIZED HERE
	*************************************************/
    cout << "Now loading: " << argv[1] << endl;
	initFMOD( argv[1] );
	fmodchn->setPaused(music_paused);
	
	glutMainLoop();
	
	/*************************************************
	 CLEAN UP HERE
	*************************************************/
	fmodsnd->release();
	fmodsys->close();
	fmodsys->release();	
	free(specR);
	free(specL);
}
