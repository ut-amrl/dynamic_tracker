#include <GL/glew.h>
#include <GL/glut.h>
#include <iostream>
#include <vector>
using namespace std;

// Data for single skeleton
typedef struct Body {
    uint32_t bodyId;
    vector<vector<float>> joints; // vector of x,y,z x,y,z,w?
} Body;

// Single frame - can contain multiple bodies
typedef struct BodyFrame {
    uint32_t timestamp;
    uint32_t frameId;
    vector<Body> bodies;
} BodyFrame;

// Draw skeleton for a single body
void drawBody(Body& b)
{
    glPointSize(20.0f);
    glScalef(0.01f, 0.01f, 0.01f);

    glPointSize(5.0f);
    // Draw points for joints
    glBegin(GL_POINTS);
    cout << "num joints: " << b.joints.size() << endl;
    for (int i = 0; i < b.joints.size(); i++) {
        cout << "point[" << i << "]: " << b.joints[i][0] << " " << b.joints[i][1] << " " << b.joints[i][2] << endl;
        glVertex3f(b.joints[i][0], b.joints[i][1], b.joints[i][2]);
    }
    glEnd();

    if (b.joints.size() < 24)
        return;
    // Add lines between joints
    glBegin(GL_LINES);
    glColor3f(0, 1, 0);
    // pelvis - naval
    glVertex3f(b.joints[0][0], b.joints[0][1], b.joints[0][2]);
    glVertex3f(b.joints[1][0], b.joints[1][1], b.joints[1][2]);
    // pevlis - hip left
    glVertex3f(b.joints[0][0], b.joints[0][1], b.joints[0][2]);
    glVertex3f(b.joints[18][0], b.joints[18][1], b.joints[18][2]);
    // pelvis - hip right
    glVertex3f(b.joints[0][0], b.joints[0][1], b.joints[0][2]);
    glVertex3f(b.joints[22][0], b.joints[22][1], b.joints[22][2]);
    // naval - chest
    glVertex3f(b.joints[1][0], b.joints[1][1], b.joints[1][2]);
    glVertex3f(b.joints[2][0], b.joints[2][1], b.joints[2][2]);
    // hip left - knee left
    glVertex3f(b.joints[18][0], b.joints[18][1], b.joints[18][2]);
    glVertex3f(b.joints[19][0], b.joints[19][1], b.joints[19][2]);
    // hip right - knee right
    glVertex3f(b.joints[22][0], b.joints[22][1], b.joints[22][2]);
    glVertex3f(b.joints[23][0], b.joints[23][1], b.joints[23][2]);
    // TODO ...

    glEnd();
}

// To be called by GLUT event loop, handles window resizing
void reshape(int width, int height)
{
    glViewport(0, 0, (GLsizei)width, (GLsizei)height); // Set our viewport to the size of our window
    glMatrixMode(GL_PROJECTION); // Switch to the projection matrix so that we can manipulate how our scene is viewed
    glLoadIdentity(); // Reset the projection matrix to the identity matrix so that we don't get any artifacts (cleaning up)
    gluPerspective(60, (GLfloat)width / (GLfloat)height, 1.0, 100.0); // Set the Field of view angle (in degrees), the aspect ratio of our window, and the new and far planes
    glMatrixMode(GL_MODELVIEW); // Switch back to the model view matrix, so that we can start drawing shapes correctly
    // gluOrtho2D(0.0, (GLdouble)width, 0.0, (GLdouble)height); // Book - puts the origin, (0, 0), all the way in the lowest, leftmost square, and makes each square represent one unit
    // glMatrixMode(GL_MODELVIEW);
}

// Draw 3D axis at origin
void drawRGBAxis(void)
{
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(1, 0, 0);
    glVertex3f(0, 0, 0);

    glColor3f(0, 1, 0);
    glVertex3f(0, 1, 0);
    glVertex3f(0, 0, 0);

    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 1);
    glVertex3f(0, 0, 0);

    glEnd();
}

// Sample draw of 4 points in a square
void renderPrimitive(void)
{
    glPointSize(20.0f);
    glBegin(GL_POINTS); // Start drawing a point primitive
    glVertex3f(-1.0f, -1.0f, 0.0f); // The bottom left corner
    glVertex3f(-1.0f, 1.0f, 0.0f); // The top left corner
    glVertex3f(1.0f, 1.0f, 0.0f); // The top right corner
    glVertex3f(1.0f, -1.0f, 0.0f); // The bottom right corner
    glEnd();
}

vector<BodyFrame> frameData;
BodyFrame activeFrame;
float dist = -5.0f;

// Main display function
void display(void)
{
    glClearColor(0.5f, 0.5f, 0.5f, 1.0f); 
    glClear(GL_COLOR_BUFFER_BIT); //Clear the colour buffer
    glLoadIdentity(); // Load the Identity Matrix to reset our drawing locations

    // glTranslatef(0.0f, 0.0f, -5.0f);      // Push eveything 5 units back into the scene
    gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0); // Move camera 5 units back

    // renderPrimitive();
    // Draw all bodies for the current frame
    for (int i = 0; i < activeFrame.bodies.size(); i++) {
        cout << "drawing body " << i << endl;
        drawBody(activeFrame.bodies[i]);
    }

    glFlush(); // Flush the OpenGL buffers to the window
}

// Updates using glutTimerFunc, will run every x ms period
void timerUpdate(int time)
{
    // TODO use to step through frames
    dist -= 1.0f;
    if (dist < -50) {
        dist = -5.0f;
    }
    glutPostRedisplay();
    printf("time: %d \tdist: %f\n", time, dist);
    glutTimerFunc(500, timerUpdate, 0);
}

int main(int argc, char** argv)
{
    // TODO add parsing body track from file
    // frameData = parseFile();
    Body testBody;
    // Add some random joints to the test body
    testBody.joints.emplace_back(initializer_list<float> { 200, 200, 200 });
    testBody.joints.emplace_back(initializer_list<float> { 150, 150, 192 });
    testBody.joints.emplace_back(initializer_list<float> { 175, 175, 190 });
    activeFrame.bodies.push_back(testBody);
    
    // Lots of openGL boilerplate from: http://www.swiftless.com/opengltuts.html
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE); // Set up a basic display buffer (only single buffered for now)
    glutInitWindowSize(900, 700);
    glutInitWindowPosition(540, 100);
    glutCreateWindow("Body model");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    // glutTimerFunc(500, timerUpdate, 0);

    glutMainLoop();
}