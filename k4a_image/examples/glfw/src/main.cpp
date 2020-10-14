#include <GLFW/glfw3.h>

int main(void) {
    if (!glfwInit())
        return -1;

    GLFWwindow *window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

/*

http://math.hws.edu/bridgeman/courses/324/s06/doc/opengl.html

#include <GL/gl.h>
#include <GL/glut.h>

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

void reshape ( int width, int height ) {
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
}
*/