#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <k4a/k4a.h>
#include <iostream>

#include "KinectWrapper.h"

class KFRViewer: public K4ACaptureRecipient {
private:
    k4a_calibration_t calib;
    k4a_image_t xyz_image = NULL;
    k4a_image_t mapped_color_image = NULL;
public:
    ~KFRViewer() {
        if (xyz_image) k4a_image_release(xyz_image);
        if (mapped_color_image) k4a_image_release(mapped_color_image);
    }

    void setCalibration(k4a_calibration_t calib) {
        this->calib = calib;
    }

    void receiveFrame(k4a_capture_t capture) {
        k4a_capture_reference(capture);

        k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
        k4a_image_t color_image = k4a_capture_get_color_image(capture);
        // TODO: does the depth image need to be converted to the color transformation?

        if (!xyz_image) {
            k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                k4a_image_get_width_pixels(depth_image),
                k4a_image_get_height_pixels(depth_image),
                k4a_image_get_width_pixels(depth_image) * 3 * sizeof(int16_t),
                &xyz_image);
        }
        if (!mapped_color_image) {
            k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                k4a_image_get_width_pixels(depth_image),
                k4a_image_get_height_pixels(depth_image),
                k4a_image_get_width_pixels(depth_image) * 4 * sizeof(int8_t),
                &mapped_color_image);
        }
        k4a_transformation_t transform = k4a_transformation_create(&calib);
        k4a_transformation_depth_image_to_point_cloud(transform, depth_image, K4A_CALIBRATION_TYPE_DEPTH, xyz_image);
        k4a_transformation_color_image_to_depth_camera(transform, depth_image, color_image, mapped_color_image);
        //std::cout << k4a_image_get_buffer(xyz_image) << std::endl;

        k4a_image_release(depth_image);
        k4a_image_release(color_image);
        k4a_capture_release(capture);
    }

    void draw() {
        size_t size = k4a_image_get_size(xyz_image);
        void *buffer = k4a_image_get_buffer(xyz_image);
        void *colorBuffer = k4a_image_get_buffer(mapped_color_image);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glVertexPointer(3, GL_SHORT, 0, buffer);
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, colorBuffer); // TODO this is BGRA, GL reads it as RGBA
        glDrawArrays(GL_POINTS, 0, size / 6);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);
    }
};

KFRViewer* frameRecipient = new KFRViewer[2];

void displayTeapots () {

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

}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    //displayTeapots();
    for(int i = 0; i < 2; i++) {
        frameRecipient[i].draw();
    }

    glFlush();
}

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}

// https://www.khronos.org/opengl/wiki/OpenGL_Error#Catching_errors_.28the_easy_way.29
void GLAPIENTRY
onGLMessage( GLenum source,
                 GLenum type,
                 GLuint id,
                 GLenum severity,
                 GLsizei length,
                 const GLchar* message,
                 const void* userParam )
{
    switch (severity) {
        case GL_DEBUG_SEVERITY_HIGH:
            fprintf(stderr, "GL message (severity HIGH): %s\n", message);
            break;
        case GL_DEBUG_SEVERITY_MEDIUM:
            fprintf(stderr, "GL message (severity MEDIUM): %s\n", message);
            break;
        case GL_DEBUG_SEVERITY_LOW:
            fprintf(stderr, "GL message (severity LOW): %s\n", message);
            break;
        default:
            fprintf(stderr, "GL message (severity 0x%x): %s\n", severity, message);
    }
}

int main(int argc, char** argv) {
    if (!glfwInit())
        return -1;
    glutInit(&argc, argv);

    for(int i = 0; i < 2; i++) {
        KinectWrapper wrapper(i, frameRecipient[i]);
        k4a_calibration_t calib = wrapper.getCalibration();
        frameRecipient[i].setCalibration(calib);
        if (!wrapper.capture()) {
            return -1;
        }
    }

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
    
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(onGLMessage, NULL);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //gluPerspective(60,1,1,10);

    const GLdouble pi = 3.1415926535897932384626433832795;
    GLdouble fW, fH;

    //fH = tan( (fovY / 2) / 180 * pi ) * zNear;
    fH = 0.57;
    fW = fH * 1;

    glFrustum( -fW, fW, -fH, fH, 1, 10000);

    double lastMouseX, lastMouseY;

    double cameraX = 0, cameraY = 0, cameraZ = 0;

    while (!glfwWindowShouldClose(window)) {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glRotatef(180, 0, 1, 0);
        glRotatef(180, 0, 0, 1);

        glTranslatef(cameraX, cameraY, cameraZ);

        double currentMouseX, currentMouseY;
        glfwGetCursorPos(window, &currentMouseX, &currentMouseY);

        double mouseChangeX = currentMouseX - lastMouseX, mouseChangeY = currentMouseY - lastMouseY;
        lastMouseX = currentMouseX;
        lastMouseY = currentMouseY;

        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1)) {
            // Rotation
        }
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2)) {
            // Translation

        }
        // TODO zoom with mouse wheel

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
}




CloudRenderer::CloudRenderer(size_t maxPts, int ptSize, GLubyte alpha
	, int imgChannelSrc, int imgChannel) :
	_maxPts(maxPts)
	, _nPts(0)
	, _ptSize(ptSize)

	, _imgChannelSrc(imgChannelSrc)
	, _imgChannel(imgChannel)

	, _alpha(alpha)
	, _cameraSpacePoints(new CameraSpacePoint[_maxPts])
	//, _colorSpacePoints(new ColorSpacePoint[_maxPts])
	//, _colorIsGood(new bool[_maxPts])
	, _rgbx(cv::Mat(1, maxPts, CV_8UC3))
	//, _bgr(new unsigned char[_maxPts * 3])
{

	#ifdef _WIN32
		_mutex = CreateMutex(NULL, FALSE, NULL);
	#elif __linux__
		pthread_mutex_init(&_mutex, NULL);
	#endif


	glGenBuffers(1, &_pointBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, _pointBuffer);
	glBufferData(GL_ARRAY_BUFFER, _maxPts*sizeof(CameraSpacePoint), NULL
		, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &_colorBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, _colorBuffer);
	glBufferData(GL_ARRAY_BUFFER, _maxPts * 3, NULL, GL_DYNAMIC_DRAW);

	//glBufferSubData(GL_ARRAY_BUFFER, 0, _nPts*3*sizeof(float)
	//	, _cameraSpacePoints);
}

CloudRenderer::~CloudRenderer() {
	delete[] _cameraSpacePoints;
	//delete[] _colorSpacePoints;
	//delete[] _colorIsGood;
	
	glDeleteBuffers(1, &_pointBuffer);
	glDeleteBuffers(1, &_colorBuffer);

	//delete[] _bgr;
	#ifdef _WIN32
		//ReleaseMutex(_mutex);
		CloseHandle(_mutex);
	#elif __linux__
		//pthread_mutex_unlock(&_mutex);
		pthread_mutex_destroy(&_mutex);
	#endif
}

void CloudRenderer::initializeRenderer() {}

void CloudRenderer::render(CameraViewportManager *cvm) {
	pthread_mutex_lock(&_mutex);


	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, _pointBuffer);
	glBufferSubData(GL_ARRAY_BUFFER, 0
		, _nPts*sizeof(CameraSpacePoint)
		, _cameraSpacePoints);
	glVertexPointer(3, GL_FLOAT, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, _colorBuffer);
	glBufferSubData(GL_ARRAY_BUFFER, 0
		, _maxPts * 3
		, _rgbx.data);
	glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);

	glDrawArrays(GL_POINTS, 0, _nPts);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	pthread_mutex_unlock(&_mutex);
}

void CloudRenderer::processType(shared_ptr<Pt3DCollection> t) {
	processType(t.get());
}

void CloudRenderer::processType(Pt3DCollection *t) {
	_nPts = t->getNPts();
	if (_nPts > _maxPts) _nPts = _maxPts;

	memcpy(_cameraSpacePoints, t->getCameraPts()
		, _nPts * sizeof(CameraSpacePoint));
	memcpy(_rgbx.data, t->getRGB(), _nPts * sizeof(unsigned int));
}

void CloudRenderer::processType(SimplePool<KinectFrame>::PooledPtr t) {
	_nPts = t->_pts.getNPts();
	if (_nPts > _maxPts) _nPts = _maxPts;

	cv::Mat &rgb = t->getImageChannel(_imgChannelSrc, _imgChannel);

	memcpy(_cameraSpacePoints, t->_pts.getCameraPts()
		, _nPts * sizeof(CameraSpacePoint));
	memcpy(_rgbx.data, rgb.data, _nPts * sizeof(unsigned int));
}



*/
