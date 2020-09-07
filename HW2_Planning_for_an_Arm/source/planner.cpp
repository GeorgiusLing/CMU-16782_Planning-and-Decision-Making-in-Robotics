#define _USE_MATH_DEFINES

#include <iostream>
#include <chrono>
#include <fstream>
#include <sstream>

#include <GL/freeglut.h>
#include <glm/vec3.hpp>

#include "planner.hpp"

const int LINK_LENGTH = 10;
glm::dvec3 eye;
size_t numDOFs;
std::vector<std::vector<GLdouble>> grids;
std::deque<GLdouble*> path;

RRTPlanner<GLdouble*> *rrtPlanner;
PRMPlanner<GLdouble*> *prmPlanner;

/**
 * Read the world representation.
 *
 * The world is represented as an occupancy grid map.
 * 1 is for occupied, 0 is for free.
 *
 * @param filename the text file representing the world
 */
static void readWorld(const std::string filenanme) {
    grids.clear();
    std::ifstream ifs(filenanme);
    if (!ifs) {
        exit (EXIT_FAILURE);
    }
    std::string line;
    while (std::getline(ifs, line)) {
        grids.push_back(std::vector<GLdouble>());
        std::istringstream tokens(line);
        GLdouble grid;
        while (tokens >> grid) {
            grids.back().push_back(grid);
        }
    }
}

/**
 * Check if a linkage collides with an obstacle.
 *
 * The linkage is represented as a line segment. This method implements 
 * Bresenham's line algorithm to check if a grid on the line segment is
 * occupied to determine if a line segment can represent a valid linkage.
 *
 * @param x0 the x-coord of first point
 * @param y0 the y-coord of first point
 * @param x1 the x-coord of second point
 * @param y1 the y-coord of second point
 * @return false if the line segment crosses an occupied grid
 */
static bool isValidLinkage(GLdouble x0, GLdouble y0, GLdouble x1, GLdouble y1) {
    bool steep = std::fabs(y1 - y0) > std::fabs(x1 - x0);
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int dx = x1 - x0;
    int dy = std::fabs(y1 - y0);
    int error = dx / 2;
    int step = y0 < y1 ? 1 : -1;
    int y = y0;
    for (int x = x0; x <= x1; ++x) {
        if (x >= grids[0].size() || x < 0 || y >= grids.size() || y < 0) {
            continue;
        }
        if ((steep && grids[x][y] > 0) || (!steep && grids[y][x] > 0)) {
            return false;
        }
        error -= dy;
        if (error < 0) {
            y += step;
            error += dx;
        }
    }

    return true;
}

/**
 * Check if a configuration is valid.
 *
 * The linkage is represented as a line segment. This method implements
 * Bresenham's line algorithm to check if a grid on the line segment is
 * occupied to determine if a line segment can represent a valid linkage.
 *
 * @param angles the angular displacements in radians
 * @return true if the configuration is valid
 */
static bool isValidConfiguration(GLdouble* angles) {
    GLdouble x0 = 0;
    GLdouble y0 = grids.size() / 2.0;
    for (auto index = 0; index < numDOFs; ++index) {
        GLdouble x1 = x0 + LINK_LENGTH * std::cos(angles[index]);
        GLdouble y1 = y0 + LINK_LENGTH * std::sin(angles[index]);
        if (!isValidLinkage(x0, y0, x1, y1)) {
            return false;
        }
        x0 = x1;
        y0 = y1;
    }
    return true;
}

/**
 * Compute the Euclidean distance between 2 configurations
 *
 * @param angles1 first configuration
 * @param angles2 second configuration
 * @return euclidean distance
 */
static GLdouble euclideanDistance(GLdouble* angles1, GLdouble* angles2) {
    GLdouble norm = 0.0;
    for (auto index = 0; index < numDOFs; ++index) {
        GLdouble diff = fmod(angles1[index] - angles2[index] + M_PI, 2 * M_PI);
        diff = (diff > 0 ? diff - M_PI : diff + M_PI);
        norm += diff * diff;
    }
    norm = std::sqrt(norm);
    return norm;
}

/**
 * Draw a valid sample configuration from the configuration space
 *
 * @return sample drawn
 */
static GLdouble* sample(void) {
    GLdouble* angles = new GLdouble[numDOFs];
    do {
        for (auto i = 0; i < numDOFs; ++i) {
            angles[i] = static_cast<GLdouble>(rand())
                    / (static_cast<GLdouble>(RAND_MAX / (2 * M_PI)));
        }
    } while (!isValidConfiguration(angles));

    return angles;
}

/**
 * Interpolate between 2 configurations.
 *
 * @param angles1 first configuration
 * @param angles2 second configuration
 * @param offset from first to second configuration
 */
static GLdouble* interpolate(GLdouble* angles1, GLdouble* angles2,
        double offset) {
    GLdouble* angles_interp = new GLdouble[numDOFs];
    for (auto i = 0; i < numDOFs; ++i) {
        angles_interp[i] = angles1[i]
                + (angles2[i] - angles1[i]) * offset;
    }
    return angles_interp;
}

/**
 * The main routine of the planner
 *
 * @param start initial configuration
 * @param goal target configuration
 * @param plannerId 0-RRT, 1-RRT connect, 2-RRT*, 3-PRM
 */
static void plan(GLdouble* start, GLdouble* goal, int plannerId) {
    size_t numNodes = 70000;
    GLdouble goalSamplingRate = 0.25;
    GLdouble tolerance = 0.15;
    GLdouble resolution = std::atan(1.0 / (numDOFs * LINK_LENGTH));
    GLdouble extension_distance = resolution * 50;
    GLdouble radius = 1.0;
    auto begin = std::chrono::high_resolution_clock::now();

    switch (plannerId) {
    case 0:
        rrtPlanner =
                new RRTPlanner<GLdouble*>(numNodes, tolerance,
                        goalSamplingRate, isValidConfiguration,
                        euclideanDistance, sample, interpolate,
                        [](GLdouble* angles) {if (angles != nullptr)delete[] angles; angles = nullptr;});
        path = rrtPlanner->buildPath(start, goal, extension_distance,
                resolution);
        break;
    case 1:
        rrtPlanner =
                new RRTConnectPlanner<GLdouble*>(numNodes,
                        tolerance, goalSamplingRate, isValidConfiguration,
                        euclideanDistance, sample, interpolate,
                        [](GLdouble* angles) {if (angles != nullptr)delete[] angles; angles = nullptr;});
        path = rrtPlanner->buildPath(start, goal, extension_distance,
                resolution);
        break;
    case 2:
        rrtPlanner =
                new RRTStarPlanner<GLdouble*>(numNodes,
                        tolerance, goalSamplingRate, isValidConfiguration,
                        euclideanDistance, sample, interpolate,
                        [](GLdouble* angles) {if (angles != nullptr)delete[] angles; angles = nullptr;});
        path = rrtPlanner->buildPath(start, goal, extension_distance,
                resolution);
        break;
    case 3:
        prmPlanner =
                new PRMPlanner<GLdouble*>(isValidConfiguration,
                        euclideanDistance, sample, interpolate,
                        [](GLdouble* angles) {if (angles != nullptr)delete[] angles; angles = nullptr;});
        prmPlanner->buildRoadmap(numNodes / 10, radius, resolution);
        path = prmPlanner->buildPath(start, goal, radius, resolution);
        break;
    default:
        std::cout << "Invalid planner id." << std::endl;
    }
    if (path.empty()) {
        path.push_back(start);
    }
}

/**
 * Set up the demo resource.
 */
void setup(void) {
    system("color 6");

    rrtPlanner = nullptr;
    prmPlanner = nullptr;

    glEnable (GL_DEPTH_TEST);
    glDepthFunc (GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    srand(time(nullptr));

    std::string filename;
    std::cout << "Enter the filename of the map: ";
    std::cin >> filename;

    std::cout << "Enter number of DOFs: ";
    std::cin >> numDOFs;
    readWorld(filename);
    eye.x = grids[0].size() / 2.0;
    eye.y = grids.size() / 2.0;
    eye.z = 150.0;
}

/**
 * The OpenGL display callback function.
 */
void display(void) {
    glClearColor(0, 0, 0, 0);
    glClear (GL_COLOR_BUFFER_BIT);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30, 1.0, 1.0, 1000);

    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye.x, eye.y, eye.z, eye.x, eye.y, 0.0, 0.0, 1.0, 0.0);

    glColor3f(1.0f, 0.0f, 0.0f);
    for (auto rowInd = 0; rowInd < grids.size(); ++rowInd) {
        for (auto colInd = 0; colInd < grids[rowInd].size(); ++colInd) {
            if (grids[rowInd][colInd] > 0) {
                glPushMatrix();
                glTranslatef(colInd + 0.5, rowInd + 0.5, 0.0);
                glutSolidCube(1);
                glPopMatrix();
            }
        }
    }

    auto angles = path[0];

    GLdouble x = 0.0;
    GLdouble y = grids.size() / 2.0;
    for (auto index = 0; index < numDOFs; ++index) {
        glColor3f(0.0f, 1.0f, 0.0f);
        glPushMatrix();
        glTranslated(x, y, 0.0);
        glRotated(angles[index] * 180 / M_PI, 0.0, 0.0, 1.0);
        glScaled(LINK_LENGTH, 0.5f, 1.0);
        glTranslated(0.5f, 0.0f, 0.0f);
        glutSolidCube(1);
        glPopMatrix();

        glColor3f(1.0f, 1.0f, 0.0f);
        glPushMatrix();
        glTranslated(x, y, 0.0);
        glutSolidSphere(0.5, 10, 10);
        glPopMatrix();

        x += LINK_LENGTH * std::cos(angles[index]);
        y += LINK_LENGTH * std::sin(angles[index]);
    }
    glutSwapBuffers();
}

/**
 * The OpenGL reshape callback function.
 */
void reshape(int width, int height) {
    glViewport(0, 0, (GLsizei) width, (GLsizei) height);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLdouble) width / (GLdouble) height, 4, 10.0);
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye.x, eye.y, eye.z, eye.x, eye.y, 0.0, 0.0, 1.0, 0.0);
}

/**
 * Tear down the demo resource.
 */
void dispose() {
    if (rrtPlanner != nullptr) {
        delete rrtPlanner;
        rrtPlanner = nullptr;
    }
    if (prmPlanner != nullptr) {
        delete prmPlanner;
        prmPlanner = nullptr;
    }
    system("cls");
}

/**
 * The OpenGL mouse wheel callback function.
 */
void mouseWheel(int button, int direction, int x, int y) {
    if (direction > 0) {
        eye.z += 10;
    }
    if (direction < 0) {
        eye.z -= 10;
    }
}

/**
 * The OpenGL timer callback function.
 */
void timer(GLint value) {
    glutTimerFunc(1000, timer, 1);
    if (path.size() > 1) {
        path.pop_front();
    }
    glutPostRedisplay();
}

/**
 * The OpenGL keyboard callback function.
 */
void keyPressed(int key, int x, int y) {
    switch (key) {
    case GLUT_KEY_END:
        glutLeaveMainLoop();
        exit (EXIT_SUCCESS);
    case GLUT_KEY_LEFT:
        --eye.x;
        break;
    case GLUT_KEY_RIGHT:
        ++eye.x;
        break;
    case GLUT_KEY_UP:
        ++eye.y;
        break;
    case GLUT_KEY_DOWN:
        --eye.y;
        break;
    default:
        return;
    }
}

int main(int argc, char** argv) {
    LABEL: setup();

    std::vector < GLdouble > start(numDOFs, DBL_MAX);
    std::vector < GLdouble > goal(numDOFs, DBL_MAX);
    std::cout << "Enter start configuration: ";
    for (int index = 0; index < numDOFs; ++index) {
        std::cin >> start[index];
    }
    std::cout << "Enter goal configuration: ";
    for (int index = 0; index < numDOFs; ++index) {
        std::cin >> goal[index];
    }
    int plannerId;
    std::cout << "Enter planner ID (0-RRT, 1-RRT Connect, 2-RRT*, 3-PRM): ";
    std::cin >> plannerId;
    plan(start.data(), goal.data(), plannerId);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(800, 800);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("N-DOF robot arm planning");
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    glutDisplayFunc(display);
    glutTimerFunc(1000, timer, 1);
    glutReshapeFunc(reshape);
    glutMouseWheelFunc(mouseWheel);
    glutSpecialFunc(keyPressed);
    glutCloseFunc(dispose);
    glutMainLoop();
    goto LABEL;
    return EXIT_SUCCESS;
}
