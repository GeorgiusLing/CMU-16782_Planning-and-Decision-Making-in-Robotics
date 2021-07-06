#include <iostream>
#include <chrono>
#include <execution>
#include <fstream>
#include <sstream>

#include <GL/freeglut.h>
#include <glm/glm.hpp>

#include "planner.hpp"

glm::ivec3 robotStart;
GLuint numCols, numRows;
std::vector<std::vector<GLfloat>> grids;
GLfloat collisionThrehold;
std::deque<glm::ivec3> targetTraj;

static glm::dvec3 eye;
static GLuint texId;

/**
 * Read the world representation.
 *
 * The world is represented as a grid map.
 * The value associated with a grid is the cost to cross it.
 *
 * @param filename the text file representing the world
 */
static void readWorld(const std::string& filename) {
    std::ifstream ifs(filename);
    char flag = 0;
    if (!ifs) {
        exit(EXIT_FAILURE);
    }

    
    std::string line;
    
    while (std::getline(ifs, line)) {

        if (line.length() == 1) {
            flag = line[0];
        }
        else if (flag == 'N') {
            // Dimension of the map
            std::stringstream tokens(line);
            std::string token;
            std::getline(tokens, token, ',');
            numRows = std::stoul(token, nullptr);
            std::getline(tokens, token, ',');
            numCols = std::stoul(token, nullptr);
        }
        else if (flag == 'C') {
            // Threshold above which a grid is considered not crossable
            collisionThrehold = std::stof(line, nullptr);
        }
        else if (flag == 'M') {
            // Map
            grids.push_back(std::vector<GLfloat>());
            std::stringstream tokens(line);
            std::string token;
            while (tokens.good()) {
                std::getline(tokens, token, ',');
                GLfloat grid = std::stof(token, nullptr);
                grids.back().push_back(grid);
            }
        }
        else if (flag == 'R') {
            // Intial position of the robot
            std::stringstream tokens(line);
            std::string token;
            std::getline(tokens, token, ',');
            robotStart.z = std::stoul(token, nullptr);
            std::getline(tokens, token, ',');
            robotStart.x = std::stoul(token, nullptr);
        }
        else if (flag == 'T') {
            // Trajectory of the target
            glm::ivec3 coord;
            std::stringstream tokens(line);
            std::string token;
            std::getline(tokens, token, ',');
            coord.z = std::stoul(token, nullptr);
            std::getline(tokens, token, ',');
            coord.x = std::stoul(token, nullptr);
            // t component represents timestamp
            coord.t = targetTraj.size();
            targetTraj.push_back(coord);
        }
        else {
            std::cerr << "Unknow state." << std::endl;
        }
    }
}

std::vector<GLint> dcol = { 1, 1, 1, 0, -1, -1, -1, 0, 0 };
std::vector<GLint> drow = { -1, 0, 1, 1, 1, 0, -1, -1, 0 };
static std::unordered_map<GLint, double> getEdges(GLint vertex) {
    std::unordered_map<GLint, double> edges;
    
    GLint row = (GLint)(vertex / numCols);
    GLint col = (GLint)(vertex % numCols);
    
    for (auto i = 0; i < drow.size(); ++i) {
        GLint neighborRow = row + drow[i];
        GLint neighborCol = col + dcol[i];

        if (neighborRow >= 0 && neighborCol >= 0 
            && neighborRow < grids.size() 
            && neighborCol < grids[row].size() 
            && grids[neighborRow][neighborCol] < collisionThrehold) {
            
            GLint neighbor = (GLint)(neighborRow * numCols + neighborCol);
            edges[neighbor] = grids[neighborRow][neighborCol];
        }
    }
    return edges;
}

GLint timestamp = 0;

std::deque<glm::ivec3> robotTraj;
static void plan(void) {
    DijkstraPlanner<GLint> planner(getEdges);
    GLint source = robotStart.z * numCols + robotStart.x;

    std::deque<GLint> selectedPath;
    std::vector<std::pair<std::deque<GLint>, double>> candidatePaths;
    auto begin = std::chrono::high_resolution_clock::now();
    for (auto& target : targetTraj) {
        GLint goal = target.z * numCols + target.x;
        std::deque<GLint> candidatePath = planner.buildPath(source, goal);
        std::cout << candidatePath.size() << std::endl;
        candidatePaths.push_back(std::make_pair(candidatePath, planner.getG(goal)));
    }
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - begin).count();
    std::cout << "Elapsed: " << elapsed << std::endl;
    timestamp += elapsed;
    double minCost = std::numeric_limits<double>::max();
    auto compare = [](GLint& lhs, GLint& rhs) {
                GLint row1 = (GLint)(lhs / numCols);
                GLint col1 = (GLint)(lhs % numCols);
                GLint row2 = (GLint)(rhs / numCols);
                GLint col2 = (GLint)(rhs % numCols);
                return grids[row1][col1] < grids[row2][col2];
    };

    GLint stop = source;
    GLint wait = 0;
    for (int index = 0; index < candidatePaths.size(); ++index) {
        auto candidatePath = candidatePaths[index].first;
        if (candidatePath.empty()) {
            continue;
        }
        auto diff = index - (candidatePath.size() + elapsed);
        if (diff > 0) {
            auto minPos = std::min_element(candidatePath.begin(), candidatePath.end(), compare);
            
            GLint row = (*minPos) / numCols;
            GLint col = (*minPos) % numCols;

            auto cost = candidatePaths[index].second + grids[row][col] * diff;
            
            if (cost < minCost) {
                minCost = cost;
                selectedPath = candidatePath;
                stop = *minPos;
                wait = diff;
            }
        }
    }

    while (elapsed > 0) {
        robotTraj.push_back(robotStart);
        --elapsed;
    }
    for (auto& vertex : selectedPath) {
        glm::ivec3 robot;
        robot.z = vertex / numCols;
        robot.x = vertex % numCols;
        robotTraj.push_back(robot);
        if (vertex == stop) {
            while (wait > 0) {
                robotTraj.push_back(robot);
                --wait;
            }
        }
    }
}

/**
 * Initialize the texture representing the environment, camera, etc.
 *
 * @param filename the text file representing the world
 */
void init(void) {

    std::string filename;
    std::cout << "Enter the filename of the map: ";
    std::cin >> filename;
    readWorld(filename);

    std::vector<GLfloat> texture;
    for (auto i = 0; i < grids.size(); ++i) {
        for (auto j = 0; j < grids[i].size(); ++j) {
            texture.push_back(1.0 - grids[i][j] / collisionThrehold);
        }
    }
    
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glGenTextures(1, &texId);
    glBindTexture(GL_TEXTURE_2D, texId);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
        GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
        GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, grids[0].size(),
        grids.size(), 0, GL_BLUE, GL_FLOAT,
        texture.data());

    texture.clear();

    eye.x = grids[0].size() / 2.0;
    eye.y = 4000.0;
    eye.z = grids.size() / 2.0;
}

/**
 * The OpenGL display callback function.
 */
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30, 1.0, 1.0, 10000);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye.x, eye.y, eye.z, eye.x, 0, eye.z, 0.0, 0.0, 1.0);

    glEnable(GL_TEXTURE_2D);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    glBindTexture(GL_TEXTURE_2D, texId);
    glBegin(GL_QUADS);
    glTexCoord2i(0, 0); glVertex3i(0, 0, 0);
    glTexCoord2i(0, 1); glVertex3i(0, 0, grids.size());
    glTexCoord2i(1, 1); glVertex3i(grids[0].size(), 0, grids.size());
    glTexCoord2i(1, 0); glVertex3i(grids[0].size(), 0, 0);
    glEnd();
    glDisable(GL_TEXTURE_2D);

    for (int index = 0; index < timestamp; ++index) {
        auto target = targetTraj[index];
        glColor3f(1.0f, 0.0f, 0.0f);
        glPushMatrix();
        glTranslated(target.x, 0.0, target.z);
        glutSolidSphere(10, 180, 180);
        glPopMatrix();
        
        auto robot = robotTraj[index];
        glColor3f(0.0f, 1.0f, 0.0f);
        glPushMatrix();
        glTranslated(robot.x, 0.0, robot.z);
        glutSolidSphere(10, 180, 180);
        glPopMatrix();
    }
    
    timestamp = std::min(++timestamp, (int)targetTraj.size());
    glFlush();
}

/**
 * The OpenGL reshape callback function.
 */
void reshape(int width, int height)
{
    glViewport(0, 0, (GLsizei)width, (GLsizei)height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)width / (GLfloat)height, 1.0, 30.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye.x, eye.y, eye.z, eye.x, 0.0, eye.z, 0.0, 0.0, 1.0);
}

void keyboard(unsigned char key, int x, int y)
{
    if (key == 27) {
        exit(EXIT_SUCCESS);
    }
}

/**
 * The OpenGL mouse wheel callback function.
 */
void mouseWheel(int button, int direction, int x, int y) {
    if (direction > 0) {
        eye.y += 10;
    }
    if (direction < 0) {
        eye.y -= 10;
    }
}

/**
 * The OpenGL timer callback function.
 */
void timer(GLint value) {
    glutTimerFunc(1, timer, 1);
    if (targetTraj.size() > 1) {
        targetTraj.pop_front();
    }
    glutPostRedisplay();
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    
    glutInitWindowSize(800, 800);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Catching mobile target");
    init();
    plan();
    glutDisplayFunc(display);
    glutTimerFunc(1, timer, 1);
    glutReshapeFunc(reshape);
    glutMouseWheelFunc(mouseWheel);
    glutKeyboardFunc(keyboard);
    glutMainLoop();
    system("pause");
    return 0;
}