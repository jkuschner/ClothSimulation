//
//  main.cpp
//  hw1
//
//  Created by Baichuan Wu on 12/16/21.
//

#include <Cube.hpp>
#include "Scene.hpp"
#include "Shader.hpp"
#include "core.hpp"
#include "utils.hpp"
#include "Cloth.hpp"

const int width = 800;
const int height = 600;

bool actPan = false, actPitchYaw = false, actZoom = false;
int mouseX = 0, mouseY = 0;

bool wireframe_mode = false;

Scene* scene;
Cloth* cloth;
std::map<std::string, Shader*> shaders;

glm::vec3 windSpeed = DEFAULT_WIND_SPEED;


void initialize() {
    glClearColor(0.0f, 0.0f, 0.0f, 1);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    shaders.insert(std::make_pair("Workbench", new Shader("Workbench", "shaders/shader.vs", "shaders/shader.fs")));
    shaders.insert(std::make_pair("Grid", new Shader("Grid", "shaders/grid.vs", "shaders/grid.fs")));

    // MARK: Example code to load from obj file
    /*
        const char* objFile = "assets/cube.obj";
        scene = loadFromObj("Scene", objFile);
    */

    scene = new Scene("Scene");
    cloth = new Cloth("Cloth", 15, MASS);
    scene->objects.insert(std::make_pair("Cloth", cloth));
}


/// This is the display call back.
void display_callback() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (wireframe_mode) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    /*
     MARK: Draw Call
     For each frame, the draw() method of current scene is called once.
     */
    scene->draw(shaders["Workbench"]);

    // Draw the grid
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    scene->drawGrid(shaders["Grid"]);

    glutSwapBuffers();
}


/// The idle call back.
void idle_callback() {
    // MARK: Perform any background tasks here
    // CALL glutPostRedisplay() if the work changes what's displayed on screen
    /*
    for (const auto& object: scene->objects) {
        object.second->update();
    }
    */
    cloth->update(windSpeed);
    glutPostRedisplay();
}


/// Keystroke callback.
void handle_keypress(unsigned char key, int x, int y) {
    switch (key) {
        case 27: // ESC
            exit(0);
            break;
        case 'r':
            scene->camera->reset();
            break;
        case 'w':
            wireframe_mode = !wireframe_mode;
            break;
        case 'q':
            windSpeed.z += 1.0f;
            break;
        case 'a':
            windSpeed.z -= 1.0f;
            break;

        //
        // MARK: Add your custom keystroks here.
        //
        default:
            return;
    }
    glutPostRedisplay();
}


/// Mouse button callback
void handle_mouse_click(int button, int state, int x, int y) {
    GLuint mod = glutGetModifiers();
    switch (button) {
        case GLUT_LEFT_BUTTON:
            actPan = (mod & GLUT_ACTIVE_SHIFT) && (state == GLUT_DOWN);
            actPitchYaw = !actPan && (state == GLUT_DOWN);
            break;
        case GLUT_RIGHT_BUTTON:
            actZoom = (state == GLUT_DOWN);
            break;
        default:
            break;
    }
    mouseX = x; mouseY = y;
}


/// Called when the mouse is moving with at least one button pressed
void handle_mouse_drag(int x, int y) {
    const int dmax = 100;
    const int dx = glm::clamp(x - mouseX, -dmax, dmax);
    const int dy = glm::clamp(y - mouseY, -dmax, dmax);
    
    mouseX = x; mouseY = y;
    
    if (actPan) {
        const float panRate = .0075f;
        scene->camera->offsetPivot(glm::vec2(-dx, dy) * panRate);
    }
    if (actPitchYaw) {
        const float pitchYawRate = .25f;
        scene->camera->setAzimuth(scene->camera->getAzimuth() + dx * pitchYawRate);
        scene->camera->setIncline(glm::clamp(scene->camera->getIncline() + dy * pitchYawRate, -90.f, 90.f));
    }
    if (actZoom) {
        const float zoomRate = .005f;
        float d = glm::clamp(scene->camera->getDistance() * (1.f - dx * zoomRate), .01f, 1000.f);
        scene->camera->setDistance(d);
    }
    
    scene->camera->update();
    glutPostRedisplay();
}


void cleanup() {
    if (scene) { delete scene; }
    shaders.clear();
}


int main(int argc, char * argv[]) {
    // Initialize glue window
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_3_2_CORE_PROFILE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(width, height);
    glutCreateWindow("Viewer");
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;

    initialize();

    /// see link: https://www.opengl.org/resources/libraries/glut/spec3/node46.html
    glutDisplayFunc(display_callback);
    glutIdleFunc(idle_callback);
    glutKeyboardFunc(handle_keypress);
    glutMouseFunc(handle_mouse_click);
    glutMotionFunc(handle_mouse_drag);
    glutMainLoop();

    cleanup();
    return 0;
}
