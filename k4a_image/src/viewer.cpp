#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <k4a/k4a.h>
#include <iostream>

#include <Eigen/Eigen>

#include "KinectWrapper.h"
#include "CalibrationManager.h"

Eigen::MatrixXd RIGID_TRANSFORMATION(4, 4);

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
        glVertexPointer(3, GL_SHORT, 0, buffer);
        glEnableClientState(GL_COLOR_ARRAY);
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, colorBuffer); // TODO this is BGRA, GL reads it as RGBA
        glDrawArrays(GL_POINTS, 0, size / 6);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);
    }
};

std::vector<KFRViewer*> frameRecipients;
std::vector<Eigen::MatrixXd> rigidTransformations;

void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    //displayTeapots();
    for(int i = 0; i < frameRecipients.size(); i++) {
        glPushMatrix();
        if (i == 1) {
            //glColor3d(0, 1, 0);
            glMultMatrixd(rigidTransformations[i].data());
            //glRotated(180, 0, 0, 1);
        } else {
            //glColor3d(1, 0, 0);
        }
        frameRecipients[i]->draw();
        glPopMatrix();
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

// Rotation helpers
/*Eigen::Vector3d mapToArcball(double x, double y, double displayWidth, double displayHeight) {
    Eigen::Vector2d scaled((x * 2 / displayWidth) - 1, (y * 2 / displayHeight) - 1);

    double distanceFromCenterSquared = scaled(0) * scaled(0) + scaled(1) * scaled(1);

    if (distanceFromCenterSquared >= 1) {
        double distanceFromCenter = std::sqrt(distanceFromCenterSquared);
        return Eigen::Vector3d(scaled(0) / distanceFromCenter, scaled(1) / distanceFromCenter, 0);
    }

    return Eigen::Vector3d(scaled(0), scaled(1), std::sqrt(1 - distanceFromCenterSquared));
}

Eigen::Quaterniond getArcballRotation(double startX, double startY, double endX, double endY, double displayWidth, double displayHeight) {
    Eigen::Vector3d startPos = mapToArcball(startX, startY, displayWidth, displayHeight);
    Eigen::Vector3d endPos = mapToArcball(endX, endY, displayWidth, displayHeight);

    Eigen::Vector3d crossProduct = startPos.cross(endPos);

    return Eigen::Quaterniond(startPos.dot(endPos), crossProduct(0), crossProduct(1), crossProduct(2));
}*/

int main(int argc, char** argv) {
    Calibration poses = Calibration::readFile("calibration.txt");
    std::vector<KinectWrapper*> wrappers;

    if (!glfwInit())
        return -1;

    for(int i = 0; i < KinectWrapper::getNumCameras(); i++) {
        if (!poses.has(Anchor(CAMERA, i))) {
            continue;
        }
        KFRViewer *frameRecipient = new KFRViewer;
        KinectWrapper* wrapper = new KinectWrapper(i, *frameRecipient);
        wrappers.push_back(wrapper);
        k4a_calibration_t calib = wrapper->getCalibration();
        frameRecipient->setCalibration(calib);
        wrapper->capture();
        frameRecipients.push_back(frameRecipient);
        rigidTransformations.push_back(poses.translation(Anchor(CAMERA, i), poses.firstCamera()));
    }

    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        exit(EXIT_FAILURE);

    int windowWidth = 640;
    int windowHeight = 480;
    GLFWwindow *window = glfwCreateWindow(windowWidth, windowHeight, "Camera View", NULL, NULL);
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

    //Eigen::MatrixXd cameraTransform = Eigen::MatrixXd::Identity(4, 4);

    double cameraX = 0, cameraY = 0, cameraZ = 0;
    double cameraYaw = 0, cameraPitch = 0;

    while (!glfwWindowShouldClose(window)) {
        for(int i = 0; i < wrappers.size(); i++) {
            //wrappers[i]->capture();
        }
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        double currentMouseX, currentMouseY;
        glfwGetCursorPos(window, &currentMouseX, &currentMouseY);

        double mouseChangeX = currentMouseX - lastMouseX, mouseChangeY = currentMouseY - lastMouseY;
        lastMouseX = currentMouseX;
        lastMouseY = currentMouseY;

        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1)) {
            // Rotation
            cameraYaw += mouseChangeX * -0.1;
            cameraPitch += mouseChangeY * -0.1;

            while (cameraYaw < -180) cameraYaw += 360;
            while (cameraYaw > 180) cameraYaw -= 360;
            
            if (cameraPitch > 90) cameraPitch = 90;
            if (cameraPitch < -90) cameraPitch = -90;
        }
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2)) {
            // Translation
            cameraX += mouseChangeX;
            cameraY -= mouseChangeY;
        }
        // TODO zoom with mouse wheel

        /*
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1)) {
            // Rotation
            //Eigen::MatrixXd rigidTransform = Eigen::MatrixXd::Identity(4, 4);
            //rigidTransform.block(0, 0, 3, 3) = getArcballRotation(lastMouseX, lastMouseY, currentMouseX, currentMouseY,
            //    windowWidth, windowHeight).toRotationMatrix();
            //cameraTransform *= rigidTransform;
        }
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2)) {
            // Translation
            Eigen::MatrixXd translateRelativeToCamera = Eigen::MatrixXd::Identity(4, 4);
            translateRelativeToCamera(0, 3) = mouseChangeX; // Movement on camera's X axis
            translateRelativeToCamera(1, 3) = -mouseChangeY; // Movement on camera's Y axis
            
            cameraTransform *= translateRelativeToCamera;
        }
        // TODO zoom with mouse wheel

        glMultMatrixd(cameraTransform.data());
        */

        glRotated(cameraPitch, 1, 0, 0);
        glRotated(cameraYaw, 0, 1, 0);
        glTranslatef(cameraX, cameraY, cameraZ);

        glRotatef(180, 0, 1, 0);
        glRotatef(180, 0, 0, 1);

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
