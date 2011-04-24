#include "main.h"



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
bool startedMusic=false;

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

//-------------------------------------------------------------------------------
/// You will be calling all of your drawing-related code from this function.
/// Nowhere else in your code should you use glBegin(...) and glEnd() except code
/// called from this method.
///
/// To force a redraw of the screen (eg. after mouse events or the like) simply call
/// glutPostRedisplay();
void display() {

	//Clear Buffers
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    setupView();
    
    mesh->render();

    if (!playanim) { // if not playing an animation draw the skeleton
        skel->render(ikJoint);
    }

	//Now that we've drawn on the buffer, swap the drawing buffer and the displaying buffer.
	glutSwapBuffers();
	
	if (savingImages) {
        //cout << "endFrameCount: " << endFrameCount << endl;
	    if (frameCount < endFrameCount) {
            //cout << "frameCount: " << frameCount << endl;
            imgSaver->saveFrame();
        }
        else
            savingImages = false;
	}
}


//-------------------------------------------------------------------------------
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


//-------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------
/// Called whenever the mouse moves while a button is pressed
void myActiveMotionFunc(int x, int y) {
    if (ikJoint != -1 && !playanim) { // if a joint is selected for ik and we're not in animation playback mode, do ik
        vec3 target = skel->getPos(vec2(x,y), ikDepth);
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


//-------------------------------------------------------------------------------
/// Called whenever the mouse moves without any buttons pressed.
void myPassiveMotionFunc(int x, int y) {
    ikJoint = -1;

    setJointsByAnimation(x);

    //Record the mouse location for drawing crosshairs
    viewport.mousePos = vec2((double)x / glutGet(GLUT_WINDOW_WIDTH),(double)y / glutGet(GLUT_WINDOW_HEIGHT));

    //Force a redraw of the window.
    glutPostRedisplay();
}

//-------------------------------------------------------------------------------
/// Called to update the screen at 30 fps.
void frameTimer(int value) {
    frameCount++;
    glutPostRedisplay();
    glutTimerFunc(1000/30, frameTimer, 1);
}



//-------------------------------------------------------------------------------
void ERRCHECK(FMOD_RESULT result)
{
    if (result != FMOD_OK)
    {
        printf("FMOD error! (%d) %s\n", result, FMOD_ErrorString(result));
        exit(-1);
    }
}

//-----------------------------------------------------------------------------------
void playMusic()
{		
		// For now this is where I'm going to put the music stuff:
			FMOD::System     *system;
		    FMOD::Sound      *sound;
		    FMOD::Channel    *channel = 0;
		    FMOD_RESULT       result;
		    int               key;
		    unsigned int      version;

		/*
	        Global Settings
	    */
	    result = FMOD::System_Create(&system);
	    ERRCHECK(result);

	    result = system->getVersion(&version);
	    ERRCHECK(result);

	    if (version < FMOD_VERSION)
	    {
	        printf("Error!  You are using an old version of FMOD %08x.  This program requires %08x\n", version, FMOD_VERSION);
	        //getch();
	        //return 0;
	    }

	    result = system->init(1, FMOD_INIT_NORMAL, 0);
	    ERRCHECK(result);

	    result = system->createSound("camera.mp3", (FMOD_MODE)(FMOD_SOFTWARE | FMOD_2D | FMOD_CREATESTREAM), 0, &sound);
	    ERRCHECK(result);

	    printf("====================================================================\n");
	    printf("PlayStream Example.  Copyright (c) Firelight Technologies 2004-2011.\n");
	    printf("====================================================================\n");
	    printf("\n");
	    printf("Press space to pause, Esc to quit\n");
	    printf("\n");

	/*
	        Play the sound.
	    */

	    result = system->playSound(FMOD_CHANNEL_FREE, sound, false, &channel);
	    ERRCHECK(result);

	    /*
	        Main loop.
	    */
	    do
	    {
			/*
	        if (kbhit())
	        {
	            key = getch();

	            switch (key)
	            {
	                case ' ' :
	                {
	                    bool paused;
	                    channel->getPaused(&paused);
	                    channel->setPaused(!paused);
	                    break;
	                }
	            }
	        }
			*/
	        system->update();

	        if (channel)
	        {
	            unsigned int ms;
	            unsigned int lenms;
	            bool         playing;
	            bool         paused;

	            channel->isPlaying(&playing);
	            if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE))
	            {
	                ERRCHECK(result);
	            }

	            result = channel->getPaused(&paused);
	            if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE))
	            {
	                ERRCHECK(result);
	            }

	            result = channel->getPosition(&ms, FMOD_TIMEUNIT_MS);
	            if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE))
	            {
	                ERRCHECK(result);
	            }

	            result = sound->getLength(&lenms, FMOD_TIMEUNIT_MS);
	            if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE))
	            {
	                ERRCHECK(result);
	            }

	            printf("\rTime %02d:%02d:%02d/%02d:%02d:%02d : %s", ms / 1000 / 60, ms / 1000 % 60, ms / 10 % 100, lenms / 1000 / 60, lenms / 1000 % 60, lenms / 10 % 100, paused ? "Paused " : playing ? "Playing" : "Stopped");
	            fflush(stdout);
	        }

	        //Sleep(10);

	    } while (key != 27);

	    printf("\n");

	    /*
	        Shut down
	    */
	    result = sound->release();
	    ERRCHECK(result);
	    result = system->close();
	    ERRCHECK(result);
	    result = system->release();
	    ERRCHECK(result);	
}


void IdleFunction(){
	glutPostRedisplay();
	//And Go!
	if (startedMusic==false) {
		startedMusic=true;
		playMusic();
	}
}

/// Initialize the environment
int main(int argc,char** argv) {
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
	glutIdleFunc(IdleFunction);
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
    //skel->initBoneWeights("attachment.out", *mesh);
    //skel->updateSkin(*mesh);
    // start a new animation
    //anim = new Animation();

    // note the .out files loaded above were made using pinocchio
    //  -- a neat free tool for auto-skinning a mesh
    // get it here: http://www.mit.edu/~ibaran/autorig/pinocchio.html
    // you can use it to easily replace the default mesh with a mesh of your own making

	glutMainLoop();
}
