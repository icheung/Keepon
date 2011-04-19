#include "main.h"

using namespace std;

//****************************************************
// Global Variables
//****************************************************
Scene * scene;
World * world;
Film * film;
int frameCount = 0;
mat4 identity = mat4(vec4(1, 0, 0, 0),
                     vec4(0, 1, 0, 0),
                     vec4(0, 0, 1, 0),
                     vec4(0, 0, 0, 1));

//------------------------------------------------------------------------------
//Helper Functions


void loadSceneInstanceIntoWorld(SceneInstance *inst, mat4 transform) {
    mat4 m = identity;
    if (inst->computeTransform(m, frameCount))
        ;

    SceneGroup *g = inst->getChild();
    double radius;
    CameraInfo ci;
    MaterialInfo mi;
    LightInfo li;

    m = transform * m;
    if (g->computeSphere(radius, mi, frameCount))
        world->insertSphere(Sphere(m * vec4(0, 0, 0, 1), radius, mi, m));
    if (g->computeLight(li, frameCount)) {
        if (li.type == LIGHT_AMBIENT) {
            world->setAmbientLight(li.color);
        } else if (li.type == LIGHT_DIRECTIONAL) {
            world->insertLight(Light(vec3(0,0,0),
                                     vec3(m * vec4(0,0,-1,0), VW),
                                     li));
        } else {
            world->insertLight(Light(vec3(m * vec4(0,0,0,1), VW),
                                     m * vec4(0,0,-1,0),
                                     li));
        }
    }
    if (g->computeCamera(ci, frameCount)) {
        double l = ci.sides[FRUS_LEFT];
        double r = ci.sides[FRUS_RIGHT];
        double b = ci.sides[FRUS_BOTTOM];
        double t = ci.sides[FRUS_TOP];
        double n = ci.sides[FRUS_NEAR];
        world->insertViewport(Viewport(vec4(0, 0, 0, 1),
                                       vec4(l, b, -1 * n, 1), // LL
                                       vec4(l, t, -1 * n, 1), // UL
                                       vec4(r, b, -1 * n, 1), // LR
                                       vec4(r, t, -1 * n, 1), // UR
                                       IMAGE_WIDTH, IMAGE_HEIGHT));
    }
    
    for (int i = 0; i < g->getChildCount(); i++)
        loadSceneInstanceIntoWorld(g->getChild(i), m);
}


//------------------------------------------------------------------------------

// use this to multiply colors:
inline vec3 pairwiseMult(const vec3 &a, const vec3 &b) {
    return vec3(a[0]*b[0], a[1]*b[1], a[2]*b[2]);
}

//-------------------------------------------------------------------------------
// Here you raycast a single ray, calculating the color of this ray.
vec3 raycast(Ray & ray) {
    vec3 retColor(0,0,0);
    vec3 d = vec3(ray.direction().normalize(),VW);    
    double t; vec3 n; MaterialInfo m;

    if (world->intersect(ray, t, n, m)) {
        retColor += pairwiseMult(m.color, world->getAmbientLight()) *
            m.k[MAT_KA];
        vec3 hit = ray.getPos(t);
        vec3 S = m.k[MAT_KSM]*m.color+(1-m.k[MAT_KSM])*vec3(1,1,1);

        for (int lighttype = 0; lighttype < 3; lighttype++) {
            vector<Light>::iterator it;
            for (it = world->getLightsBeginIterator(lighttype); 
                 it != world->getLightsEndIterator(lighttype); ++it)
            {
                const LightInfo &info = it->getLightInfo();
                vec3 pos = it->getPosition();
                vec3 incident = -it->getDirection();
                if (lighttype == LIGHT_POINT)
                    incident = pos-hit;
                vec4 incidentAgain = vec4(incident, 0);
                incident.normalize();

                // shadows (HOMOGENEOUS COORDINATES MATTER!)
                vec4 start = vec4(hit, 1) + 0.1 * vec4(incident, 1);
                start[VW] = 1;
                vec4 end = (lighttype == LIGHT_DIRECTIONAL) ? 
                    (start + vec4(incident, 1)) : vec4(pos, 1);
                end[VW] = 1;
                Ray shadow = Ray(start, end, 0.0);
                double tt; vec3 nn; MaterialInfo mm;
                
                if (world->intersect(shadow, tt, nn, mm)) {
                    if (lighttype == LIGHT_DIRECTIONAL)
                        break;
                    if (tt > 0.00001 && tt < 0.5)
                        break;
                }
                
                // falloff
                double distToLight = (pos - hit).length();
                vec3 color_falloff = info.color * 
                    pow(1 / (distToLight + info.deadDistance), info.falloff);
                
                // diffuse
                retColor += MAX(0,incident*n) * m.k[MAT_KD] *
                    pairwiseMult(m.color, color_falloff);
               
                // specular
                double rvdot = MAX(0,-d*(-incident+2*(incident*n)*n));
                retColor += m.k[MAT_KS] * pow(rvdot,m.k[MAT_KSP]) *
                    pairwiseMult(S, color_falloff);

            }
        }
    }
    
    return retColor;
}

//-------------------------------------------------------------------------------
/// The display function
void display() {
    
    //Clear Buffers
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);                 // indicate we are specifying camera transformations
    glLoadIdentity();                           // make sure transformation is "zero'd"
    
    film->show();
    
    //Now that we've drawn on the buffer, swap the drawing buffer and
    //the displaying buffer.
    glutSwapBuffers();
}


//-------------------------------------------------------------------------------
/// \brief  Called when the screen gets resized.
/// This gives you the opportunity to set up all the relevant transforms.
///
void reshape(int w, int h) {
    //Set up the viewport to ignore any size change.
    Viewport &view = world->getViewport();
    glViewport(0,0,view.getW(),view.getH());
    
    //Set up the PROJECTION transformationstack to be a simple
    //orthographic [-1, +1] projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, view.getW(), 0, view.getH(), 1, -1);
  
    //Set the MODELVIEW transformation stack to the identity.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}


//-------------------------------------------------------------------------------
/// Called to handle keyboard events.
void myKeyboardFunc(unsigned char key, int x, int y) {
    switch (key) {
    case 27:            // Escape key
        exit(0);
        break;
    }
}

void myMouseFunc(int button, int state, int x, int y) {

}


//-------------------------------------------------------------------------------
/// Initialize the environment
int main(int argc,char** argv) {
    
    //Initialize OpenGL
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);
    
    // Load the world
    scene = new Scene(argv[1]);
    world = new World();
    loadSceneInstanceIntoWorld(scene->getRoot(), identity);
    // Allocate film for rendering
    film = new Film(IMAGE_WIDTH, IMAGE_HEIGHT);
    
    //Create OpenGL Window
    glutInitWindowSize(IMAGE_WIDTH,IMAGE_HEIGHT);
    glutInitWindowPosition(0,0);
    glutCreateWindow("CS184 Raycaster");
    
    //Register event handlers with OpenGL.
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(myKeyboardFunc);
    //glutMouseFunc(myMouseFunc);

    // Calls the raycast method on each pixel sampled using the
    // Viewport and Ray classes and stores the result using the
    // Film class
    film->clear();
    Viewport &view = world->getViewport();
    view.resetSampler();
    view.setRaysPerPixel(4);
    vec2 point; Ray ray;
    while(view.getSample(point, ray)) {
        ray.transform(view.getViewToWorld());
        vec3 c = raycast(ray);
        film->expose(c, point);
    }
    film->saveAsBMP(argv[2]);
    
    //And Go!
    glutMainLoop();
}
