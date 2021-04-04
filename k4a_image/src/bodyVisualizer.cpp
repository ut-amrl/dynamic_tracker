#include <GL/glew.h>
#include <GL/glut.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using namespace std;

// Data for single skeleton
typedef struct Body {
    uint32_t bodyId;
    vector<vector<float>> joints; // vector of x,y,z x,y,z,w?
} Body;

// Single frame - can contain multiple bodies
typedef struct BodyFrame {
    uint64_t timestamp;
    vector<Body> bodies;
} BodyFrame;

vector<BodyFrame> parse(string path)
{
    vector<BodyFrame> res;
    ifstream input(path);
    if (!input.is_open()) {
        cout << "Failed to open file: " << path << endl;
        return res;
    }
    // cout << "3" << endl;
    int jointCount;
    input >> jointCount;
    int frameCount = 0;
    // Read frames
    while (!input.eof()) {
        cout << frameCount++ << endl;
        BodyFrame frame;
        input >> frame.timestamp;
        // Read bodies of frame
        uint32_t numBodies;
        input >> numBodies;
        for (int i = 0; i < numBodies; i++) {
            Body b;
            input >> b.bodyId;
            for (int j = 0; j < jointCount; j++) {
                float x, y, z, qw, qx, qy, qz;
                input >> x >> y >> z >> qw >> qx >> qy >> qz;
                b.joints.emplace_back(initializer_list<float> { x, y, -1 * z, qw, qx, qy, qz });
            }
            frame.bodies.push_back(b);
        }
        res.push_back(frame);
        // done = input.peek() == EOF;
    }
    return res;
}

// Draw skeleton for a single body
void drawBody(Body& b)
{
    glPointSize(20.0f);
    // glScalef(0.01f, 0.01f, 0.01f);

    glPointSize(5.0f);
    // Draw points for joints
    glColor3f(0, 1, 0);
    glBegin(GL_POINTS);
    // cout << "num joints: " << b.joints.size() << endl;
    for (int i = 0; i < b.joints.size(); i++) {
        // cout << "point[" << i << "]: " << b.joints[i][0] << " " << b.joints[i][1] << " " << b.joints[i][2] << endl;
        glVertex3f(b.joints[i][0], b.joints[i][1], b.joints[i][2]);
    }
    glEnd();

    if (b.joints.size() < 24)
        return;
    cout << "drawing lines" << endl;
    glLineWidth(1.0);
    // Add lines between joints
    glBegin(GL_LINES);
    glColor3f(0, 0, 1);
    // Upper body
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
    // chest - neck
    glVertex3f(b.joints[2][0], b.joints[2][1], b.joints[2][2]);
    glVertex3f(b.joints[3][0], b.joints[3][1], b.joints[3][2]);
    // chest - clavicle left
    glVertex3f(b.joints[2][0], b.joints[2][1], b.joints[2][2]);
    glVertex3f(b.joints[4][0], b.joints[4][1], b.joints[4][2]);
    // chest - clavicle right
    glVertex3f(b.joints[2][0], b.joints[2][1], b.joints[2][2]);
    glVertex3f(b.joints[11][0], b.joints[11][1], b.joints[11][2]);
    // clavicle right - shoulder right
    glVertex3f(b.joints[11][0], b.joints[11][1], b.joints[11][2]);
    glVertex3f(b.joints[12][0], b.joints[12][1], b.joints[12][2]);
    // clavicle left - shoulder left
    glVertex3f(b.joints[4][0], b.joints[4][1], b.joints[4][2]);
    glVertex3f(b.joints[5][0], b.joints[5][1], b.joints[5][2]);
    // neck - head
    glVertex3f(b.joints[3][0], b.joints[3][1], b.joints[3][2]);
    glVertex3f(b.joints[26][0], b.joints[26][1], b.joints[26][2]);

    // Arms
    // shoulder left - elbow left
    glVertex3f(b.joints[5][0], b.joints[5][1], b.joints[5][2]);
    glVertex3f(b.joints[6][0], b.joints[6][1], b.joints[6][2]);
    // shoulder right - elbow right
    glVertex3f(b.joints[12][0], b.joints[12][1], b.joints[12][2]);
    glVertex3f(b.joints[13][0], b.joints[13][1], b.joints[13][2]);
    // elbow left - wrist left
    glVertex3f(b.joints[6][0], b.joints[6][1], b.joints[6][2]);
    glVertex3f(b.joints[7][0], b.joints[7][1], b.joints[7][2]);
    // elbow right - wrist right
    glVertex3f(b.joints[13][0], b.joints[13][1], b.joints[13][2]);
    glVertex3f(b.joints[14][0], b.joints[14][1], b.joints[14][2]);
    // wrist left - hand left
    glVertex3f(b.joints[7][0], b.joints[7][1], b.joints[7][2]);
    glVertex3f(b.joints[8][0], b.joints[8][1], b.joints[8][2]);
    // wrist right - hand right
    glVertex3f(b.joints[14][0], b.joints[14][1], b.joints[14][2]);
    glVertex3f(b.joints[15][0], b.joints[15][1], b.joints[15][2]);
    // hand left - handtip left
    glVertex3f(b.joints[8][0], b.joints[8][1], b.joints[8][2]);
    glVertex3f(b.joints[9][0], b.joints[9][1], b.joints[9][2]);
    // hand right - hantip right
    glVertex3f(b.joints[15][0], b.joints[15][1], b.joints[15][2]);
    glVertex3f(b.joints[16][0], b.joints[16][1], b.joints[16][2]);
    // hand left - thumb left
    glVertex3f(b.joints[8][0], b.joints[8][1], b.joints[8][2]);
    glVertex3f(b.joints[10][0], b.joints[10][1], b.joints[10][2]);
    // hand right - thumb right
    glVertex3f(b.joints[15][0], b.joints[15][1], b.joints[15][2]);
    glVertex3f(b.joints[17][0], b.joints[17][1], b.joints[17][2]);

    // Lower body
    // hip left - knee left
    glVertex3f(b.joints[18][0], b.joints[18][1], b.joints[18][2]);
    glVertex3f(b.joints[19][0], b.joints[19][1], b.joints[19][2]);
    // hip right - knee right
    glVertex3f(b.joints[22][0], b.joints[22][1], b.joints[22][2]);
    glVertex3f(b.joints[23][0], b.joints[23][1], b.joints[23][2]);
    // knee left - ankle left
    glVertex3f(b.joints[19][0], b.joints[19][1], b.joints[19][2]);
    glVertex3f(b.joints[20][0], b.joints[20][1], b.joints[20][2]);
    // knee right - ankle right
    glVertex3f(b.joints[23][0], b.joints[23][1], b.joints[23][2]);
    glVertex3f(b.joints[24][0], b.joints[24][1], b.joints[24][2]);
    // ankle left - foot left
    glVertex3f(b.joints[20][0], b.joints[20][1], b.joints[20][2]);
    glVertex3f(b.joints[21][0], b.joints[21][1], b.joints[21][2]);
    // ankle right - foot right
    glVertex3f(b.joints[24][0], b.joints[24][1], b.joints[24][2]);
    glVertex3f(b.joints[25][0], b.joints[25][1], b.joints[25][2]);

    glEnd();
}

// To be called by GLUT event loop, handles window resizing
void reshape(int width, int height)
{
    glViewport(0, 0, (GLsizei)width, (GLsizei)height); // Set our viewport to the size of our window
    glMatrixMode(GL_PROJECTION); // Switch to the projection matrix so that we can manipulate how our scene is viewed
    glLoadIdentity(); // Reset the projection matrix to the identity matrix so that we don't get any artifacts (cleaning up)

    // const GLdouble pi = 3.1415926535897932384626433832795;
    // GLdouble fW, fH;

    // //fH = tan( (fovY / 2) / 180 * pi ) * zNear;
    // fH = 0.57;
    // fW = fH * 1;

    // glFrustum(-fW, fW, -fH, fH, 1, 10000);
    // gluPerspective(60, (GLfloat)width / (GLfloat)height, 1.0, 100.0); // Set the Field of view angle (in degrees), the aspect ratio of our window, and the new and far planes
    gluPerspective(65, (GLfloat)width / (GLfloat)height, 100.0, 10000.0); // Set the Field of view angle (in degrees), the aspect ratio of our window, and the new and far planes
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
    glColor3f(0, 1, 0);
    glBegin(GL_POINTS); // Start drawing a point primitive
    glVertex3f(-1.0f, -1.0f, 0.0f); // The bottom left corner
    glVertex3f(-1.0f, 1.0f, 0.0f); // The top left corner
    glVertex3f(1.0f, 1.0f, 0.0f); // The top right corner
    glVertex3f(1.0f, -1.0f, 0.0f); // The bottom right corner
    glVertex3f(0.0f, 0.0f, -95.0f); // The bottom right corner
    glColor3f(0, 1, 1);
    glVertex3f(0.0f, 0.0f, -1244.0f); // The bottom right corner
    glVertex3f(192.0f, -216.0f, 1244.0f); // The bottom right corner
    glEnd();
}

vector<BodyFrame> frameData;
BodyFrame activeFrame;
int activeFrameIndex;
float dist = -5.0f;

// Main display function
void display(void)
{
    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT); //Clear the colour buffer
    glLoadIdentity(); // Load the Identity Matrix to reset our drawing locations

    // glTranslatef(0.0f, 0.0f, -5.0f);      // Push eveything 5 units back into the scene
    gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0); // Move camera 5 units back

    // renderPrimitive();
    // Draw all bodies for the current frame
    // for (int i = 0; i < activeFrame.bodies.size(); i++) {
    //     cout << "drawing body " << i << endl;
    //     drawBody(activeFrame.bodies[i]);
    // }
    for (int i = 0; i < frameData[activeFrameIndex].bodies.size(); i++) {
        cout << "drawing body " << i << endl;
        drawBody(frameData[activeFrameIndex].bodies[i]);
    }

    glFlush(); // Flush the OpenGL buffers to the window
}

// Updates using glutTimerFunc, will run every x ms period
void timerUpdate(int time)
{
    activeFrameIndex++;
    if(activeFrameIndex > 140){
        activeFrameIndex = 0;
    }
    glutPostRedisplay();
    printf("time: %d \tactiveFrameIndex: %d\n", time, activeFrameIndex);
    glutTimerFunc(33, timerUpdate, 0);
}

int main(int argc, char** argv)
{
    frameData = parse("/home/fri/Documents/henry/dynamic_tracker/k4a_image/pranav2.bt");
    activeFrame = frameData[0];
    Body end = activeFrame.bodies[0];
    cout << end.joints[0][0] << " " << end.joints[0][1] << " " << end.joints[0][2] << " " << end.joints[0][3] << " " << end.joints[0][5] << " " << end.joints[0][6] << " " << endl;
    // return 0;
    // TODO add parsing body track from file
    // frameData = parseFile();
    // Body testBody;
    // // Add some random joints to the test body
    // testBody.joints.emplace_back(initializer_list<float> { 200, 200, 200 });
    // testBody.joints.emplace_back(initializer_list<float> { 150, 150, 192 });
    // testBody.joints.emplace_back(initializer_list<float> { 175, 175, 190 });
    // testBody = end;
    // activeFrame.bodies.push_back(testBody);

    // Lots of openGL boilerplate from: http://www.swiftless.com/opengltuts.html
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE); // Set up a basic display buffer (only single buffered for now)
    glutInitWindowSize(900, 700);
    glutInitWindowPosition(540, 100);
    glutCreateWindow("Body model");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutTimerFunc(33, timerUpdate, 0);

    glutMainLoop();
}