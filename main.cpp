#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <cmath>
#include <iostream>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Constants
const int WIDTH = 800;
const int HEIGHT = 600;
const int NUM_BOIDS = 20;
const int NUM_POINTS = 7;

// Catmull-Rom Spline Matrix
const float a = 0.5f;
const float CRSplineM[4][4] = {
    {-a, 2.0f - a, a - 2.0f, a},
    {2.0f * a, a - 3.0f, 3.0f - 2.0f * a, -a},
    {-a, 0.0f, a, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f}
};

// B-Spline Matrix
const float BSplineM[4][4] = {
    {-1.0f / 6, 3.0f / 6, -3.0f / 6, 1.0f / 6},
    {3.0f / 6, -6.0f / 6, 3.0f / 6, 0.0f},
    {-3.0f / 6, 0.0f, 3.0f / 6, 0.0f},
    {1.0f / 6, 4.0f / 6, 1.0f / 6, 0.0f}
};

// Initial boid positions
const float initialPositions[NUM_BOIDS][3] = {
    {-3.0f, 7.0f, -5.6f}, {9.0f, 8.5f, -4.0f}, {4.0f, 7.2f, -5.7f},
    {-4.5f, 6.8f, -5.8f}, {3.0f, 8.6f, -5.0f}, {5.0f, 9.8f, -4.9f},
    {-4.0f, 9.0f, -4.9f}, {4.0f, 12.0f, -4.5f}, {0.0f, 8.2f, -5.0f},
    {1.0f, 7.6f, -4.5f}, {-2.0f, 8.0f, -4.3f}, {10.0f, 9.5f, -3.9f},
    {5.0f, 8.2f, -4.2f}, {-3.5f, 7.8f, -4.1f}, {4.0f, 9.6f, -4.9f},
    {6.0f, 10.8f, -4.8f}, {-3.0f, 10.0f, -4.8f}, {5.0f, 13.0f, -4.4f},
    {1.0f, 9.2f, -4.9f}, {2.0f, 8.6f, -4.4f}
};

// Control points in Euler angles (x_angle, y_angle, z_angle, x, y, z)
const float controlPoints[NUM_POINTS][6] = {
    {90.0f, 0.0f, 45.0f, -3.0f, 7.0f, -8.0f},
    {70.0f, 20.0f, 65.0f, -8.0f, -2.0f, -9.0f},
    {50.0f, 40.0f, 85.0f, -5.0f, 6.0f, -10.0f},
    {30.0f, 60.0f, 105.0f, 5.0f, -8.0f, -10.0f},
    {50.0f, 40.0f, 85.0f, 3.0f, -10.0f, -5.0f},
    {70.0f, 20.0f, 65.0f, -3.0f, -14.0f, -5.0f},
    {90.0f, 0.0f, 45.0f, 1.0f, -18.0f, -3.0f}
};

// Global animation variables
float t = 0.0f;
int pointIndex = 0;

// Quaternion structure
struct Quaternion {
    float w, x, y, z;
    float px, py, pz;

    Quaternion() : w(1), x(0), y(0), z(0), px(0), py(0), pz(0) {}

    Quaternion(float w_, float x_, float y_, float z_, float px_, float py_, float pz_)
        : w(w_), x(x_), y(y_), z(z_), px(px_), py(py_), pz(pz_) {
        normalize();
    }

    void normalize() {
        float mag = sqrt(w * w + x * x + y * y + z * z);
        if (mag != 0) {
            w /= mag; x /= mag; y /= mag; z /= mag;
        }
    }

    void toMatrix(float mat[16]) {
        mat[0] = 1 - 2 * y * y - 2 * z * z; mat[4] = 2 * x * y - 2 * w * z; mat[8] = 2 * x * z + 2 * w * y;   mat[12] = px;
        mat[1] = 2 * x * y + 2 * w * z;   mat[5] = 1 - 2 * x * x - 2 * z * z; mat[9] = 2 * y * z - 2 * w * x;   mat[13] = py;
        mat[2] = 2 * x * z - 2 * w * y;   mat[6] = 2 * y * z + 2 * w * x;   mat[10] = 1 - 2 * x * x - 2 * y * y; mat[14] = pz;
        mat[3] = 0;             mat[7] = 0;             mat[11] = 0;             mat[15] = 1;
    }
};

// Boid structure
struct Boid {
    float position[3];
    float velocity[3];
    float matrix[16];
    float r1[3], r2[3], r3[3], r4[3];

    Boid() {
        memset(position, 0, sizeof(position));
        memset(velocity, 0, sizeof(velocity));
        memset(r1, 0, sizeof(r1));
        memset(r2, 0, sizeof(r2));
        memset(r3, 0, sizeof(r3));
        memset(r4, 0, sizeof(r4));

        for (int i = 0; i < 16; i++) matrix[i] = 0;
        matrix[0] = matrix[5] = matrix[10] = matrix[15] = 1.0f;
    }

    void setPosition(float x, float y, float z) {
        position[0] = x; position[1] = y; position[2] = z;
        matrix[12] = x; matrix[13] = y; matrix[14] = z;
    }

    void setVelocity(float vx, float vy, float vz) {
        velocity[0] = vx; velocity[1] = vy; velocity[2] = vz;
    }

    float distance(const Boid& other) const {
        float dx = position[0] - other.position[0];
        float dy = position[1] - other.position[1];
        float dz = position[2] - other.position[2];
        return sqrt(dx * dx + dy * dy + dz * dz);
    }
};

// Global boids array
Boid boids[NUM_BOIDS];
Boid leaderBoid;

// Matrix multiplication
void multiplyMatrix(const float A[4][4], const float B[4][4], float result[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Convert Euler angles to Quaternion
Quaternion eulerToQuaternion(float ax, float ay, float az, float px, float py, float pz) {
    float a = ax / 2.0f;
    float b = ay / 2.0f;
    float c = az / 2.0f;

    float ca = cos(a), sa = sin(a);
    float cb = cos(b), sb = sin(b);
    float cc = cos(c), sc = sin(c);

    float w = cc * cb * ca + sc * sb * sa;
    float x = sc * cb * ca - cc * sb * sa;
    float y = cc * sb * ca + sc * cb * sa;
    float z = cc * cb * sa - sc * sb * ca;

    return Quaternion(w, x, y, z, px, py, pz);
}

// Quaternion interpolation using spline
Quaternion quaternionInterpolation(float t, int idx) {
    // Build time matrix
    float T[1][4] = { {t * t * t, t * t, t, 1} };

    // Build geometry matrix with 4 control points
    float G[4][7];
    for (int i = 0; i < 4; i++) {
        int pointIdx = (idx + i) % NUM_POINTS;
        Quaternion q = eulerToQuaternion(
            controlPoints[pointIdx][0] * M_PI / 180.0f,
            controlPoints[pointIdx][1] * M_PI / 180.0f,
            controlPoints[pointIdx][2] * M_PI / 180.0f,
            controlPoints[pointIdx][3],
            controlPoints[pointIdx][4],
            controlPoints[pointIdx][5]
        );
        G[i][0] = q.w; G[i][1] = q.x; G[i][2] = q.y; G[i][3] = q.z;
        G[i][4] = q.px; G[i][5] = q.py; G[i][6] = q.pz;
    }

    // Compute T * M
    float TM[1][4];
    for (int j = 0; j < 4; j++) {
        TM[0][j] = 0;
        for (int k = 0; k < 4; k++) {
            TM[0][j] += T[0][k] * BSplineM[k][j];
        }
    }

    // Compute (T*M) * G
    float result[7];
    for (int j = 0; j < 7; j++) {
        result[j] = 0;
        for (int k = 0; k < 4; k++) {
            result[j] += TM[0][k] * G[k][j];
        }
    }

    return Quaternion(result[0], result[1], result[2], result[3],
        result[4], result[5], result[6]);
}

// Boid behaviors
void followLeader(int idx) {
    for (int i = 0; i < 3; i++) {
        boids[idx].r1[i] = (leaderBoid.position[i] - boids[idx].position[i]) / 1000.0f;
    }
}

void collisionAvoidance(int idx) {
    int count = 0;
    boids[idx].r2[0] = boids[idx].r2[1] = boids[idx].r2[2] = 0;

    for (int i = 0; i < NUM_BOIDS; i++) {
        if (i != idx) {
            float dist = boids[idx].distance(boids[i]);
            if (dist < 4.0f) {
                count++;
                for (int j = 0; j < 3; j++) {
                    boids[idx].r2[j] += -dist / 1500.0f;
                }
            }
        }
    }

    if (count > 0) {
        for (int j = 0; j < 3; j++) {
            boids[idx].r2[j] /= count;
        }
    }
}

void velocityMatching(int idx) {
    float totalVel[3] = { 0, 0, 0 };

    for (int i = 0; i < NUM_BOIDS; i++) {
        for (int j = 0; j < 3; j++) {
            totalVel[j] += boids[i].velocity[j];
        }
    }

    for (int i = 0; i < 3; i++) {
        float avgVel = (totalVel[i] - boids[idx].velocity[i]) / (NUM_BOIDS - 1);
        boids[idx].r3[i] = (avgVel - boids[idx].velocity[i]) / 2000.0f;
    }
}

void flockCentering(int idx) {
    float totalPos[3] = { 0, 0, 0 };

    for (int i = 0; i < NUM_BOIDS; i++) {
        for (int j = 0; j < 3; j++) {
            totalPos[j] += boids[i].position[j];
        }
    }

    for (int i = 0; i < 3; i++) {
        float centerPos = (totalPos[i] - boids[idx].position[i]) / (NUM_BOIDS - 1);
        boids[idx].r4[i] = (centerPos - boids[idx].position[i]) / 2500.0f;
    }
}

void updateBoid(int idx) {
    followLeader(idx);
    collisionAvoidance(idx);
    velocityMatching(idx);
    flockCentering(idx);

    float newVel[3], newPos[3];
    for (int j = 0; j < 3; j++) {
        newVel[j] = boids[idx].velocity[j] + boids[idx].r1[j] +
            boids[idx].r2[j] + boids[idx].r3[j] + boids[idx].r4[j];
        newPos[j] = boids[idx].position[j] + newVel[j] * 0.15f;
    }

    boids[idx].setVelocity(newVel[0], newVel[1], newVel[2]);
    boids[idx].setPosition(newPos[0], newPos[1], newPos[2]);
}

// OpenGL setup and rendering
void setupLighting() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glShadeModel(GL_SMOOTH);

    float lightAmbient[] = { 0.5f, 0.0f, 0.0f, 1.0f };
    float lightDiffuse[] = { 0.4f, 0.4f, 0.4f, 1.0f };
    float lightSpecular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    float lightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

    float materialKa[] = { 1.0f, 1.0f, 0.0f, 1.0f };
    float materialKd[] = { 0.43f, 0.47f, 0.54f, 1.0f };
    float materialKs[] = { 0.33f, 0.33f, 0.52f, 1.0f };
    float materialKe[] = { 0.1f, 0.0f, 0.1f, 1.0f };

    glMaterialfv(GL_FRONT, GL_AMBIENT, materialKa);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, materialKd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, materialKs);
    glMaterialfv(GL_FRONT, GL_EMISSION, materialKe);
    glMaterialf(GL_FRONT, GL_SHININESS, 10.0f);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearDepth(1.0);

    setupLighting();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(15.0, 15.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // Set dark blue color for all objects
    float darkBlue[] = { 0.0f, 0.2f, 0.6f, 1.0f };
    glMaterialfv(GL_FRONT, GL_AMBIENT, darkBlue);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, darkBlue);

    // Render leader boid as SPHERE (changed from cube)
    Quaternion q = quaternionInterpolation(t, pointIndex);
    leaderBoid.setPosition(q.px, q.py, q.pz);
    q.toMatrix(leaderBoid.matrix);

    glPushMatrix();
    glMultMatrixf(leaderBoid.matrix);
    glutSolidSphere(1.5, 30, 30);  // Larger sphere with more detail
    glPopMatrix();

    // Render follower boids as larger spheres
    for (int i = 0; i < NUM_BOIDS; i++) {
        updateBoid(i);
        glPushMatrix();
        glMultMatrixf(boids[i].matrix);
        glutSolidSphere(0.8, 25, 25);  // Much larger spheres
        glPopMatrix();
    }

    glutSwapBuffers();

    // Update animation time
    t += 0.01f;
    if (t >= 1.0f) {
        t = 0.0f;
        pointIndex = (pointIndex + 1) % NUM_POINTS;
    }
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(100.0, (float)width / (float)height, 1.0, 2000.0);
}

void timer(int value) {
    glutPostRedisplay();
    glutTimerFunc(16, timer, 0); // ~60 FPS
}

void init() {
    // Initialize boids
    for (int i = 0; i < NUM_BOIDS; i++) {
        boids[i].setPosition(initialPositions[i][0],
            initialPositions[i][1],
            initialPositions[i][2]);
        boids[i].setVelocity(0, 0, 0);
    }
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Behavioral Motion Control System");

    init();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutTimerFunc(0, timer, 0);

    glutMainLoop();
    return 0;
}
