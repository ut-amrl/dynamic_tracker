#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glut.h>

#include <iostream>

void display () {

    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);

    glPushMatrix();
    glPushMatrix();

    glTranslatef(0,0,-3);
    glutWireTeapot(1);                // middle teapot
    glTranslatef(0,2,0);
    glutSolidTeapot(1);               // top teapot
    glPopMatrix();

    glTranslatef(0,-2,-1);
    glutSolidTeapot(1);               // bottom teapot

    glPopMatrix();

    glFlush();

}

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}

int main(int argc, char** argv) {
        if (!glfwInit())
        return -1;
        glutInit(&argc, argv);

    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        exit(EXIT_FAILURE);

    GLFWwindow *window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    // glViewport(0, 0,  640, 480);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //gluPerspective(60,1,1,10);

    const GLdouble pi = 3.1415926535897932384626433832795;
    GLdouble fW, fH;

    //fH = tan( (fovY / 2) / 180 * pi ) * zNear;
    fH = 0.57;
    fW = fH * 1;

    glFrustum( -fW, fW, -fH, fH, 1, 100);


    double angle = 0;
    while (!glfwWindowShouldClose(window)) {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0, 0, -7.5);
        glRotatef(angle, 0, 1, 0);
        angle += 1;

        glViewport(0,0,640,480);
        display();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}



// int main(int argc, char **argv) {
//     if (!glfwInit())
//         return -1;
//     glutInit(&argc, argv);

//     GLFWwindow *window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
//     // glViewport(0, 0,  640, 480);
//     if (!window) {
//         glfwTerminate();
//         return -1;
//     }

//     glfwMakeContextCurrent(window);

//     glMatrixMode(GL_PROJECTION);
//     glLoadIdentity();

//     while (!glfwWindowShouldClose(window)) {
//         glViewport(0,0,640,480);
//         display();
//         glfwSwapBuffers(window);
//         glfwPollEvents();
//     }

//     glfwTerminate();
//     return 0;
// }




//http://math.hws.edu/bridgeman/courses/324/s06/doc/opengl.html


/*void reshape ( int width, int height ) {
    glViewport(0,0,width,height);

}

int main ( int argc, char * argv[] ) {

    glutInit(&argc,argv);

    glutInitWindowSize(500,500);
    glutInitWindowPosition(0,0);
    glutInitDisplayMode(GLUT_RGB);

    glutCreateWindow("hello, teapot!");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60,1,1,10);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0,0.0,5.0,0.0,0.0,0.0,0.0,1.0,0.0);

    glutMainLoop();
}*/
