// ============================================================
//  CarWorkStation - main.cpp
//
//  CAMERA (6-DOF fly-simulator):
//    W/S          - Move forward / backward
//    A/D          - Strafe left / right
//    E / R        - Move up / down
//    Arrow Keys   - Pitch / Yaw look
//    X            - Pitch up   (hold)
//    Y            - Yaw right  (hold)
//    Z            - Roll right (hold)
//    F            - Orbit around building centre (hold)
//    0            - Bird's Eye View toggle (top-down camera)
//    Scroll       - Zoom FOV
//
//  LIGHTING:
//    1            - Toggle directional (sun) light
//    2            - Toggle all point lights
//    3            - Toggle spot light (single cut-off cone)
//    5            - Toggle ambient  component
//    6            - Toggle diffuse  component
//    7            - Toggle specular component
//    8            - Toggle emissive glow (neons, headlights, indicator LEDs, globes)
//    L            - Master light on/off
//
//  VIEWPORT:
//    4            - Toggle Full screen / 4-quadrant viewport
//                   Top-Left    : Combined lighting  (interactive camera)
//                   Top-Right   : Ambient only       (top-down view)
//                   Bottom-Left : Diffuse only       (front view)
//                   Bottom-Right: Directional only   (isometric view)
//
//  DOORS & WINDOWS:
//    T            - Open / Close main shutter door (rolls up/down)
//    I            - Open / Close paint shop booth door (slides up)
//
//  HYDRAULIC LIFTS (with cars):
//    J / K        - 4-post lift 1 / 2 (Assembly Bay front)
//    M            - 4-post lift 3 NEW (Assembly Bay right)
//    V / N        - Scissor lift 1 / 2 (Assembly Bay rear)
//
//  DYNAMIC EQUIPMENT:
//    C            - Toggle conveyor belt
//    O            - Toggle robotic arms
//    B            - Toggle paint spray / turntable rotation
//    H            - Toggle hydraulic press (Body Shop)
//    G            - Toggle ceiling fans
//
//  CAR DRIVE (road test):
//    P            - Enter / Exit drive mode
//    (in drive)   Arrow Keys - Steer / Accelerate / Brake
//    (in drive)   SPACE      - Handbrake
//
//  ESC            - Quit
// ============================================================

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "stb_image.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader.h"
#include "basic_camera.h"
#include "pointLight.h"
#include "cube.h"
#include "cylinder.h"
#include "sphere.h"
#include "sound.h"

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <cmath>
using namespace std;

// ---- Window ----
const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;

// ---- Camera ----
BasicCamera camera(100.0f, 60.0f, 185.0f, 100.0f, 5.0f, 24.0f);

// ---- Timing ----
float deltaTime = 0.0f, lastFrame = 0.0f;

// ---- Viewport mode ----
// 0 = full screen (single view, combined lighting)
// 1 = 4 quadrants
int viewportMode = 0;

// ---- Shutter door ----
// The shutter is composed of NUM_PANELS horizontal slabs that
// slide upward (translate +Y) when opening.
float shutterOpenProgress = 0.0f;   // 0.0 = fully closed, 1.0 = fully open
bool  shutterOpening = false;
bool  shutterClosing = false;
const float SHUTTER_SPD = 1.2f;   // units per second (fraction of full travel)
const float SHUTTER_H = 8.8f;   // total travel height (matches door opening)
const int   NUM_PANELS = 10;

// ---- Hydraulic lifts ----
// [0]=J 4-post#1  [1]=K 4-post#2  [2]=V scissor#1  [3]=N scissor#2  [4]=M 4-post#3 (new)
float liftH[5] = { 0.0f, 2.6f, 1.8f, 0.0f, 1.5f };
bool  liftGoingUp[5] = { false,false,false,false,false };
bool  liftGoingDn[5] = { false,false,false,false,false };
const float LIFT_MAX = 3.5f;
const float LIFT_SPD = 1.4f;

// ---- Fan / paint turntable ----
bool  fanRotating          = true;    // G key toggles ceiling fan rotation
float paintTurntableAngle  = 0.0f;   // turntable car rotation (degrees)

// ---- Bird's Eye View ----
bool  birdEyeView          = false;  // key 0 toggles top-down camera

// ---- Paint shop side door ----
bool  paintDoorOpen        = false;
float paintDoorAmt         = 0.0f;   // 0=closed, 1=fully open (slides upward)

// ---- Drive-mode car ----
bool  driveMode = false;
float drivePosX = 100.0f;  // centre of new 200-wide building
float drivePosZ = 80.0f;   // on road, south of building front
float driveYaw  = 90.0f;   // facing north (-Z), toward building entrance
float driveSpeed = 0.0f;    // current speed (units/s)
float driveWheelRot = 0.0f;
float driveSteerAngle = 0.0f;
const float DRIVE_ACCEL = 8.0f;
const float DRIVE_BRAKE = 14.0f;
const float DRIVE_MAX_SPD = 18.0f;
const float DRIVE_FRICTION = 3.5f;
const float DRIVE_STEER_SPD = 60.0f; // deg/s
bool  driveKeyUp = false, driveKeyDn = false, driveKeyLt = false, driveKeyRt = false;
bool  driveHandbrake = false;
float fanAngle = 0.0f;  // ceiling fan blade rotation (degrees)

// ---- Player car paint colour (permanently changed by driving into paint zone) ----
glm::vec3 driveCarColor    = { 0.80f, 0.08f, 0.08f };  // starts red
bool      driveInPaintZone = false;

// Paint-zone world bounds — matches the red rectangle on the floor (one car)
const float PZ_X1 = 71.f, PZ_X2 = 79.f;
const float PZ_Z1 = 18.f, PZ_Z2 = 30.f;

// ---- Fuel & Damage system ------------------------------------------------
float fuelLevel     = 1.0f;   // 1.0 = full, 0.0 = empty
float damageLevel   = 0.0f;   // 0.0 = undamaged, 1.0 = destroyed
float fuelZoneTimer = 0.0f;   // seconds spent in fuel zone (need 3 to refuel)
bool  inFuelZone    = false;
bool  inRepairZone  = false;

const float FUEL_BURN_RATE = 1.0f / 180.0f; // full tank lasts 3 minutes
const float FUEL_FILL_SECS = 3.0f;           // seconds to fully refuel

// Fuel zone bounds (on gas station forecourt, centre world (-38,0,22))
const float FZ_X1 = -44.f, FZ_X2 = -32.f;
const float FZ_Z1 =  17.f, FZ_Z2 =  27.f;
// Repair zone bounds (assembly bay back area)
const float RPZX1 = 18.f, RPZX2 = 30.f;
const float RPZZ1 = 36.f, RPZZ2 = 46.f;

// ---- Section animation globals ----
bool  conveyorOn   = true;
float conveyorOff  = 0.0f;   // belt animation offset (loops 0..beltLen)
bool  robotOn      = true;
float robotAngle   = 0.0f;   // robotic arm sweep angle (degrees)
bool  sprayOn      = true;
float sprayAngle   = 0.0f;   // paint nozzle swing angle
bool  pressActive  = false;
float pressHeight  = 3.0f;   // hydraulic press Y height (3=up, 0=down)
float pressDir     = 0.0f;   // 1=going down, -1=going up

// ---- Sound system ----
SoundSystem g_sound;
SectionID   currentSection = SEC_NONE;

// ---- Lighting toggles ----
bool dirLightOn = true;
bool pointLightsOn = true;
bool spotLightOn = true;
bool ambientOn = true;
bool diffuseOn = true;
bool specularOn = true;
bool masterLightOn = true;
bool emissiveOn = true;        // key 8 – toggle all emissive glow objects

// ---- Skybox ----
unsigned int skyboxVAO = 0, skyboxVBO = 0, skyboxTex = 0;

// ---- Texture IDs ----
unsigned int texRoad, texGrass, texBrick, texConcrete;
unsigned int texMetal, texTire, texBark, texLeaf, texPot;
unsigned int texWhite, texOrange, texRed, texWindow, texRoof, texLaneMark;
unsigned int texWood, texDarkFloor, texStripe;
unsigned int texAsphalt, texSidewalk, texGlassBlue;
unsigned int texColor;

// Collision AABBs {x1,z1,x2,z2} – populated once in main(), tested each frame in updateDriveCar()
std::vector<std::array<float,4>> g_collBoxes;

// ---- Prototypes ----
void framebuffer_size_callback(GLFWwindow*, int, int);
void scroll_callback(GLFWwindow*, double, double);
void key_callback(GLFWwindow*, int, int, int, int);
void processInput(GLFWwindow*);
unsigned int loadTexture(const char*);

// ============================================================
//  myRotate – Rodrigues' rotation (matches glm::rotate semantics)
// ============================================================
glm::mat4 myRotate(glm::mat4 m, float angle, glm::vec3 axis)
{
    axis = glm::normalize(axis);
    float c = cosf(angle), s = sinf(angle), t = 1.0f - c;
    float x = axis.x, y = axis.y, z = axis.z;
    glm::mat4 R(1.0f);
    R[0][0] = t * x * x + c;    R[1][0] = t * x * y - s * z; R[2][0] = t * x * z + s * y;
    R[0][1] = t * x * y + s * z;  R[1][1] = t * y * y + c;   R[2][1] = t * y * z - s * x;
    R[0][2] = t * x * z - s * y;  R[1][2] = t * y * z + s * x; R[2][2] = t * z * z + c;
    return m * R;
}

// ============================================================
//  Utility helpers
// ============================================================
void drawCube(Cube& cube, Shader& sh, glm::mat4 model,
    glm::vec3 amb, glm::vec3 diff, glm::vec3 spec,
    float shine, unsigned int tex)
{
    cube.ambient = amb;
    cube.diffuse = diff;
    cube.specular = spec;
    cube.shininess = shine;
    cube.textureMap = tex;
    cube.draw(sh, model);
}

void solidCube(Cube& cube, Shader& sh, glm::mat4 model,
    glm::vec3 color, float shine = 32.0f)
{
    drawCube(cube, sh, model, color * 0.35f, color, color * 0.5f, shine, texWhite);
    sh.setBool("useTexture", false);
}

inline glm::mat4 TRS(glm::vec3 t, glm::vec3 r_deg, glm::vec3 s)
{
    glm::mat4 m = glm::translate(glm::mat4(1.0f), t);
    if (r_deg.y != 0) m = myRotate(m, glm::radians(r_deg.y), glm::vec3(0, 1, 0));
    if (r_deg.x != 0) m = myRotate(m, glm::radians(r_deg.x), glm::vec3(1, 0, 0));
    if (r_deg.z != 0) m = myRotate(m, glm::radians(r_deg.z), glm::vec3(0, 0, 1));
    return glm::scale(m, s);
}

// ============================================================
//  drawAttractiveCar
//  A more realistic multi-piece car:
//    - Sculpted lower body + raised cabin + sloped hood & trunk
//    - Bumpers, side skirts, spoiler
//    - Detailed wheels: tire + rim + 5 spokes
//    - Separate headlight lenses, tail-light bars
//    - Emissive lights (headlights / tail-lights / indicators)
//  pos    : world position of car centre (ground level)
//  yaw    : heading in degrees
//  bodyCol: paint colour
//  emitHL : emit headlights
//  emitTL : emit tail-lights
//  wRot   : wheel rotation (radians)
// ============================================================
void drawAttractiveCar(Cube& cube, Shader& sh,
    glm::vec3 pos, float yaw,
    glm::vec3 bodyCol,
    bool emitHL = false, bool emitTL = false,
    float wRot = 0.0f)
{
    const glm::vec3 chromC(0.85f, 0.85f, 0.88f);
    const glm::vec3 glassC(0.35f, 0.55f, 0.75f);
    const glm::vec3 darkC(0.08f, 0.08f, 0.09f);
    const glm::vec3 tireC(0.08f, 0.08f, 0.08f);
    const glm::vec3 rimC(0.80f, 0.80f, 0.84f);
    const glm::vec3 hlC(1.00f, 0.98f, 0.90f);
    const glm::vec3 tlC(1.00f, 0.10f, 0.05f);
    const glm::vec3 skirC(0.06f, 0.06f, 0.07f);

    // Base transform (car centre at ground)
    auto T = [&](glm::vec3 t, glm::vec3 s) -> glm::mat4 {
        glm::mat4 m = glm::translate(glm::mat4(1.0f), pos);
        m = myRotate(m, glm::radians(yaw), glm::vec3(0, 1, 0));
        m = glm::translate(m, t);
        return glm::scale(m, s);
        };
    auto TR = [&](glm::vec3 t, float rx, float ry, float rz, glm::vec3 s) -> glm::mat4 {
        glm::mat4 m = glm::translate(glm::mat4(1.0f), pos);
        m = myRotate(m, glm::radians(yaw), glm::vec3(0, 1, 0));
        m = glm::translate(m, t);
        if (ry != 0) m = myRotate(m, glm::radians(ry), glm::vec3(0, 1, 0));
        if (rx != 0) m = myRotate(m, glm::radians(rx), glm::vec3(1, 0, 0));
        if (rz != 0) m = myRotate(m, glm::radians(rz), glm::vec3(0, 0, 1));
        return glm::scale(m, s);
        };

    // ── Body ──────────────────────────────────────────────────
    // Lower chassis slab
    drawCube(cube, sh, T({ 0,0.22f,0 }, { 4.20f,0.44f,1.90f }),
        bodyCol * 0.3f, bodyCol, bodyCol * 0.6f, 80, texMetal);
    // Mid body belt
    drawCube(cube, sh, T({ 0,0.55f,0 }, { 4.10f,0.22f,1.92f }),
        bodyCol * 0.35f, bodyCol, bodyCol * 0.65f, 80, texMetal);
    // Front fender section
    drawCube(cube, sh, T({ 1.45f,0.62f,0 }, { 1.30f,0.30f,1.88f }),
        bodyCol * 0.35f, bodyCol, bodyCol * 0.65f, 80, texMetal);
    // Rear fender section
    drawCube(cube, sh, T({ -1.45f,0.62f,0 }, { 1.30f,0.30f,1.88f }),
        bodyCol * 0.35f, bodyCol, bodyCol * 0.65f, 80, texMetal);

    // ── Hood (angled) ─────────────────────────────────────────
    // Two overlapping boxes to fake the slope
    drawCube(cube, sh, TR({ 1.55f,0.74f,0 }, -7, 0, 0, { 1.50f,0.10f,1.80f }),
        bodyCol * 0.35f, bodyCol, bodyCol * 0.7f, 96, texMetal);
    drawCube(cube, sh, TR({ 1.00f,0.82f,0 }, -3, 0, 0, { 0.65f,0.09f,1.80f }),
        bodyCol * 0.35f, bodyCol, bodyCol * 0.7f, 96, texMetal);
    // Hood leading edge
    solidCube(cube, sh, T({ 2.10f,0.62f,0 }, { 0.05f,0.20f,1.86f }), chromC, 96);

    // ── Trunk lid ─────────────────────────────────────────────
    drawCube(cube, sh, TR({ -1.60f,0.74f,0 }, 6, 0, 0, { 1.30f,0.10f,1.80f }),
        bodyCol * 0.35f, bodyCol, bodyCol * 0.7f, 96, texMetal);
    // Rear spoiler
    drawCube(cube, sh, T({ -2.05f,0.96f,0 }, { 0.12f,0.16f,1.70f }),
        darkC * 0.5f, darkC, glm::vec3(0.05f), 16, texWhite);
    sh.setBool("useTexture", false);
    solidCube(cube, sh, T({ -2.05f,1.05f,0 }, { 0.60f,0.06f,1.74f }), darkC, 32);

    // ── Cabin ─────────────────────────────────────────────────
    drawCube(cube, sh, T({ -0.15f,1.02f,0 }, { 2.30f,0.62f,1.72f }),
        bodyCol * 0.35f, bodyCol, bodyCol * 0.6f, 80, texMetal);
    // Roof
    drawCube(cube, sh, T({ -0.15f,1.38f,0 }, { 2.10f,0.15f,1.65f }),
        bodyCol * 0.3f, bodyCol, bodyCol * 0.55f, 64, texMetal);

    // ── Glass ─────────────────────────────────────────────────
    // Windshield (front)
    drawCube(cube, sh, TR({ 0.95f,1.05f,0 }, -12, 0, 0, { 0.06f,0.56f,1.58f }),
        glassC * 0.4f, glassC, glm::vec3(1, 1, 1), 128, texWindow);
    // Rear window
    drawCube(cube, sh, TR({ -1.22f,1.05f,0 }, 10, 0, 0, { 0.06f,0.55f,1.58f }),
        glassC * 0.4f, glassC, glm::vec3(1, 1, 1), 128, texWindow);
    // Side windows
    drawCube(cube, sh, T({ -0.15f,1.06f, 0.875f }, { 2.06f,0.53f,0.05f }),
        glassC * 0.4f, glassC, glm::vec3(1, 1, 1), 128, texWindow);
    drawCube(cube, sh, T({ -0.15f,1.06f,-0.875f }, { 2.06f,0.53f,0.05f }),
        glassC * 0.4f, glassC, glm::vec3(1, 1, 1), 128, texWindow);

    // ── A-pillars & B-pillars ─────────────────────────────────
    for (float sz : {0.87f, -0.87f}) {
        solidCube(cube, sh, T({ 0.98f,1.06f,sz }, { 0.09f,0.56f,0.07f }), darkC, 32);
        solidCube(cube, sh, T({ -0.28f,1.06f,sz }, { 0.09f,0.56f,0.07f }), darkC, 32);
        solidCube(cube, sh, T({ -1.12f,1.06f,sz }, { 0.09f,0.56f,0.07f }), darkC, 32);
    }

    // ── Doors (body-coloured panels below windows) ────────────
    for (float sz : {0.94f, -0.94f}) {
        drawCube(cube, sh, T({ 0.32f,0.70f,sz }, { 1.85f,0.42f,0.06f }),
            bodyCol * 0.35f, bodyCol, bodyCol * 0.6f, 80, texMetal);
        drawCube(cube, sh, T({ -0.90f,0.70f,sz }, { 1.45f,0.42f,0.06f }),
            bodyCol * 0.35f, bodyCol, bodyCol * 0.6f, 80, texMetal);
        // Door handles
        solidCube(cube, sh, T({ 0.28f,0.75f,sz + 0.04f * glm::sign(sz) }, { 0.25f,0.06f,0.06f }), chromC, 128);
        solidCube(cube, sh, T({ -0.90f,0.75f,sz + 0.04f * glm::sign(sz) }, { 0.22f,0.06f,0.06f }), chromC, 128);
    }

    // ── Side skirts ──────────────────────────────────────────
    for (float sz : {1.00f, -1.00f}) {
        solidCube(cube, sh, T({ 0,0.15f,sz }, { 3.90f,0.14f,0.08f }), skirC, 16);
    }

    // ── Front bumper system ───────────────────────────────────
    solidCube(cube, sh, T({ 2.16f,0.30f,0 }, { 0.14f,0.36f,1.88f }), chromC, 96);
    solidCube(cube, sh, T({ 2.12f,0.20f,0 }, { 0.18f,0.16f,1.90f }), darkC, 32);
    // Grille
    drawCube(cube, sh, T({ 2.15f,0.50f,0 }, { 0.06f,0.20f,1.60f }),
        darkC * 0.3f, darkC, glm::vec3(0.1f), 16, texWhite);
    sh.setBool("useTexture", false);
    // Grille mesh bars
    for (int gi = -3; gi <= 3; gi++) {
        solidCube(cube, sh, T({ 2.16f,0.50f,gi * 0.22f }, { 0.07f,0.18f,0.04f }), darkC, 8);
    }
    // Front lower splitter
    solidCube(cube, sh, T({ 2.10f,0.06f,0 }, { 0.20f,0.06f,1.85f }), darkC, 16);

    // ── Rear bumper ───────────────────────────────────────────
    solidCube(cube, sh, T({ -2.16f,0.30f,0 }, { 0.14f,0.36f,1.88f }), chromC, 96);
    solidCube(cube, sh, T({ -2.12f,0.20f,0 }, { 0.18f,0.16f,1.90f }), darkC, 32);
    // Exhaust pipes
    for (float ez : {0.55f, -0.55f}) {
        solidCube(cube, sh, T({ -2.20f,0.18f,ez }, { 0.10f,0.10f,0.12f }), chromC, 128);
        solidCube(cube, sh, T({ -2.22f,0.18f,ez }, { 0.04f,0.08f,0.10f }), darkC, 8);
    }

    // ── Headlights (sleek LED-style) ─────────────────────────
    for (float hz : {0.68f, -0.68f}) {
        // Outer lens housing
        drawCube(cube, sh, T({ 2.12f,0.60f,hz }, { 0.10f,0.18f,0.45f }),
            chromC * 0.3f, chromC, glm::vec3(1, 1, 1), 128, texWhite);
        sh.setBool("useTexture", false);
        // Inner LED bar
        if (emitHL)
            cube.drawEmissive(sh, T({ 2.14f,0.60f,hz }, { 0.05f,0.08f,0.40f }), hlC);
        else
            solidCube(cube, sh, T({ 2.14f,0.60f,hz }, { 0.05f,0.08f,0.40f }),
                glm::vec3(0.82f, 0.82f, 0.70f), 128);
        // DRL strip
        if (emitHL)
            cube.drawEmissive(sh, T({ 2.13f,0.50f,hz }, { 0.05f,0.04f,0.44f }), hlC * 0.8f);
        else
            solidCube(cube, sh, T({ 2.13f,0.50f,hz }, { 0.05f,0.04f,0.44f }),
                glm::vec3(0.75f, 0.75f, 0.60f), 64);
    }

    // ── Tail-lights (full-width bar style) ───────────────────
    if (emitTL) {
        cube.drawEmissive(sh, T({ -2.12f,0.55f,0 }, { 0.06f,0.24f,1.70f }), tlC);
    }
    else {
        drawCube(cube, sh, T({ -2.12f,0.55f,0 }, { 0.06f,0.24f,1.70f }),
            glm::vec3(0.30f, 0.02f, 0.02f), glm::vec3(0.45f, 0.03f, 0.03f),
            glm::vec3(0.20f), 32, texWhite);
        sh.setBool("useTexture", false);
    }
    // Central light bar connector
    if (emitTL)
        cube.drawEmissive(sh, T({ -2.11f,0.55f,0 }, { 0.04f,0.06f,1.68f }), tlC * 0.5f);

    // ── Wheels (detailed 5-spoke) ─────────────────────────────
    float wxArr[4] = { 1.25f, 1.25f,-1.25f,-1.25f };
    float wzArr[4] = { 1.00f,-1.00f, 1.00f,-1.00f };

    for (int i = 0; i < 4; ++i)
    {
        // Wheel base transform (with spin)
        glm::mat4 wBase = glm::translate(glm::mat4(1.0f), pos);
        wBase = myRotate(wBase, glm::radians(yaw), glm::vec3(0, 1, 0));
        wBase = glm::translate(wBase, glm::vec3(wxArr[i], 0.34f, wzArr[i]));
        wBase = myRotate(wBase, wRot, glm::vec3(0, 0, 1));

        // Tyre
        drawCube(cube, sh,
            glm::scale(wBase, glm::vec3(0.58f, 0.58f, 0.26f)),
            tireC * 0.3f, tireC, glm::vec3(0.05f), 8, texTire);

        // Sidewall detail ring
        solidCube(cube, sh,
            glm::scale(wBase, glm::vec3(0.56f, 0.56f, 0.22f)),
            tireC * 1.2f, 16);

        // Rim face plate
        solidCube(cube, sh,
            glm::scale(wBase, glm::vec3(0.40f, 0.40f, 0.27f)),
            rimC, 128);

        // Centre hub
        solidCube(cube, sh,
            glm::scale(wBase, glm::vec3(0.14f, 0.14f, 0.29f)),
            glm::vec3(0.5f, 0.5f, 0.52f), 128);

        // 5 spokes
        for (int sp = 0; sp < 5; ++sp) {
            glm::mat4 sm = glm::translate(glm::mat4(1.0f), pos);
            sm = myRotate(sm, glm::radians(yaw), glm::vec3(0, 1, 0));
            sm = glm::translate(sm, glm::vec3(wxArr[i], 0.34f, wzArr[i]));
            sm = myRotate(sm, wRot + glm::radians(72.0f * sp), glm::vec3(0, 0, 1));
            sm = glm::scale(sm, glm::vec3(0.04f, 0.36f, 0.07f));
            solidCube(cube, sh, sm, rimC, 96);
        }

        // Lug nut ring (small dark dots around hub)
        for (int ln = 0; ln < 5; ++ln) {
            float la = wRot + glm::radians(72.0f * ln + 36.0f);
            float lx2 = 0.16f * cosf(la);
            float ly2 = 0.16f * sinf(la);
            glm::mat4 lm = glm::translate(glm::mat4(1.0f), pos);
            lm = myRotate(lm, glm::radians(yaw), glm::vec3(0, 1, 0));
            lm = glm::translate(lm, glm::vec3(wxArr[i] + 0.00f, 0.34f + ly2, wzArr[i] + lx2));
            lm = glm::scale(lm, glm::vec3(0.04f, 0.04f, 0.29f));
            solidCube(cube, sh, lm, darkC, 64);
        }

        // Brake disc visible through spokes
        glm::mat4 bm = glm::translate(glm::mat4(1.0f), pos);
        bm = myRotate(bm, glm::radians(yaw), glm::vec3(0, 1, 0));
        bm = glm::translate(bm, glm::vec3(wxArr[i], 0.34f, wzArr[i]));
        bm = glm::scale(bm, glm::vec3(0.30f, 0.30f, 0.05f));
        solidCube(cube, sh, bm, glm::vec3(0.38f, 0.36f, 0.36f), 64);
    }

    // ── Undercarriage & skid plate ────────────────────────────
    solidCube(cube, sh, T({ 0,0.04f,0 }, { 3.80f,0.08f,1.70f }), darkC, 8);
    // Front diffuser fins
    for (int df = -2; df <= 2; df++) {
        solidCube(cube, sh, T({ 2.0f,0.10f,df * 0.30f }, { 0.40f,0.06f,0.05f }), darkC, 8);
    }
}

// ============================================================
//  drawWhiteCar  (variant – re-uses drawAttractiveCar)
// ============================================================
void drawWhiteCar(Cube& cube, Shader& sh, glm::vec3 pos, float yaw)
{
    drawAttractiveCar(cube, sh, pos, yaw,
        glm::vec3(0.90f, 0.90f, 0.92f), false, false, 0.0f);
}

// ============================================================
//  drawShutterDoor
//  Draws NUM_PANELS horizontal slats.  Each slat at rest sits
//  at its closed Y position; as shutterOpenProgress goes 0→1
//  the entire group shifts upward by SHUTTER_H.
//
//  pos    : bottom-left corner of the door opening (world space)
//  width  : total door width
//  height : total door height (closed)
//  progress: 0=closed, 1=fully open (panels retracted above opening)
// ============================================================
void drawShutterDoor(Cube& cube, Shader& sh,
    glm::vec3 pos, float width, float height,
    float progress)
{
    const glm::vec3 panelDark(0.18f, 0.22f, 0.30f);
    const glm::vec3 panelLight(0.25f, 0.30f, 0.42f);
    const glm::vec3 railC(0.30f, 0.30f, 0.32f);

    float panelH = height / NUM_PANELS;
    float liftY = progress * SHUTTER_H;   // how far panels have risen

    // Side rails
    for (float rx : {0.0f, width}) {
        solidCube(cube, sh,
            TRS(pos + glm::vec3(rx, height / 2.0f, 0),
                glm::vec3(0), glm::vec3(0.12f, height + 0.2f, 0.12f)),
            railC, 32);
    }

    // Bottom guide bar (visible when door is raised)
    solidCube(cube, sh,
        TRS(pos + glm::vec3(width / 2.0f, liftY + panelH * 0.5f, 0),
            glm::vec3(0), glm::vec3(width, 0.10f, 0.16f)),
        railC, 32);

    // Panels
    for (int i = 0; i < NUM_PANELS; ++i) {
        float panelCentreY = pos.y + panelH * (i + 0.5f) + liftY;
        glm::vec3 col = (i % 2 == 0) ? panelDark : panelLight;

        // Main panel body
        solidCube(cube, sh,
            TRS(glm::vec3(pos.x + width / 2.0f, panelCentreY, pos.z),
                glm::vec3(0), glm::vec3(width - 0.28f, panelH * 0.90f, 0.14f)),
            col, 32);

        // Horizontal rib across the panel
        solidCube(cube, sh,
            TRS(glm::vec3(pos.x + width / 2.0f, panelCentreY, pos.z + 0.08f),
                glm::vec3(0), glm::vec3(width - 0.30f, panelH * 0.12f, 0.06f)),
            col * 0.75f, 16);

        // Two vertical stiffener ribs per panel
        for (float rx2 : {-width * 0.28f, width * 0.28f}) {
            solidCube(cube, sh,
                TRS(glm::vec3(pos.x + width / 2.0f + rx2, panelCentreY, pos.z + 0.07f),
                    glm::vec3(0), glm::vec3(0.06f, panelH * 0.86f, 0.05f)),
                col * 0.80f, 16);
        }
    }

    // Handle bar (on the bottom panel, moves with door)
    solidCube(cube, sh,
        TRS(pos + glm::vec3(width / 2.0f, liftY + panelH * 0.5f, 0.12f),
            glm::vec3(0), glm::vec3(1.20f, 0.12f, 0.10f)),
        glm::vec3(0.60f, 0.62f, 0.65f), 64);
}

// ============================================================
//  draw4PostLift
// ============================================================
void draw4PostLift(Cube& cube, Shader& sh, glm::vec3 pos, float h)
{
    const glm::vec3 postC(0.10f, 0.25f, 0.72f);
    const glm::vec3 beamC(0.08f, 0.20f, 0.60f);
    const glm::vec3 platC(0.55f, 0.55f, 0.58f);
    const glm::vec3 safeC(0.95f, 0.80f, 0.05f);
    float PW = 2.6f, PL = 4.8f, PH = 4.8f, PD = 0.18f;

    float pxs[4] = { -PL / 2,-PL / 2, PL / 2, PL / 2 };
    float pzs[4] = { -PW / 2, PW / 2,-PW / 2, PW / 2 };
    for (int i = 0; i < 4; i++)
        solidCube(cube, sh, TRS(pos + glm::vec3(pxs[i], PH / 2, pzs[i]), { 0,0,0 }, { PD,PH,PD }), postC, 64);

    solidCube(cube, sh, TRS(pos + glm::vec3(-PL / 2, PH, 0), { 0,0,0 }, { PD,PD,PW + PD }), beamC, 64);
    solidCube(cube, sh, TRS(pos + glm::vec3(PL / 2, PH, 0), { 0,0,0 }, { PD,PD,PW + PD }), beamC, 64);

    float rY = h + 0.1f;
    solidCube(cube, sh, TRS(pos + glm::vec3(0, rY, -PW / 2), { 0,0,0 }, { PL + PD,PD * 0.8f,PD }), beamC, 32);
    solidCube(cube, sh, TRS(pos + glm::vec3(0, rY, PW / 2), { 0,0,0 }, { PL + PD,PD * 0.8f,PD }), beamC, 32);

    float pPlY = h + PD * 0.8f;
    drawCube(cube, sh, TRS(pos + glm::vec3(0, pPlY, 0), { 0,0,0 }, { PL + PD * 2,0.12f,PW + PD * 2 }),
        platC * 0.3f, platC, platC * 0.5f, 32, texConcrete);

    int sides[2] = { -1,1 };
    for (int si = 0; si < 2; si++)
        solidCube(cube, sh, TRS(pos + glm::vec3(0, pPlY + 0.08f, (float)sides[si] * (PW / 2 + 0.05f)),
            { 0,0,0 }, { PL * 0.9f,0.14f,0.06f }), safeC, 16);

    solidCube(cube, sh, TRS(pos + glm::vec3(0, h / 2 + 0.05f, 0), { 0,0,0 }, { 0.1f,h + 0.1f,0.1f }),
        glm::vec3(0.6f, 0.6f, 0.65f), 64);
}

// ============================================================
//  drawScissorLift
// ============================================================
void drawScissorLift(Cube& cube, Shader& sh, glm::vec3 pos, float h)
{
    const glm::vec3 redC(0.80f, 0.15f, 0.05f);
    const glm::vec3 platC(0.55f, 0.55f, 0.58f);
    const glm::vec3 safeC(0.95f, 0.80f, 0.05f);
    float BL = 4.5f, BW = 2.3f;

    drawCube(cube, sh, TRS(pos + glm::vec3(0, 0.06f, 0), { 0,0,0 }, { BL,0.12f,BW }),
        redC * 0.3f, redC, redC * 0.4f, 16, 0);

    if (h > 0.05f) {
        float hw = BL * 0.45f;
        float tilt = atan2f(h, hw) * (180.0f / 3.14159f);
        float armLen = sqrtf(hw * hw + h * h) + 0.05f;

        float zOffs[2] = { -BW / 2 + 0.15f, BW / 2 - 0.15f };
        for (int zi = 0; zi < 2; zi++) {
            glm::mat4 mA = glm::translate(glm::mat4(1.0f), pos + glm::vec3(0, h / 2, zOffs[zi]));
            mA = myRotate(mA, glm::radians(tilt), glm::vec3(0, 0, 1));
            mA = glm::scale(mA, glm::vec3(armLen, 0.12f, 0.10f));
            solidCube(cube, sh, mA, redC, 16);
            glm::mat4 mB = glm::translate(glm::mat4(1.0f), pos + glm::vec3(0, h / 2, zOffs[zi]));
            mB = myRotate(mB, glm::radians(-tilt), glm::vec3(0, 0, 1));
            mB = glm::scale(mB, glm::vec3(armLen, 0.12f, 0.10f));
            solidCube(cube, sh, mB, redC, 16);
        }
    }

    float topY = h + 0.18f;
    drawCube(cube, sh, TRS(pos + glm::vec3(0, topY, 0), { 0,0,0 }, { BL + 0.2f,0.12f,BW + 0.2f }),
        platC * 0.3f, platC, platC * 0.5f, 32, texConcrete);

    int sides[2] = { -1,1 };
    for (int si = 0; si < 2; si++) {
        solidCube(cube, sh, TRS(pos + glm::vec3(0, topY + 0.1f, (float)sides[si] * (BW / 2 + 0.08f)),
            { 0,0,0 }, { BL * 0.92f,0.16f,0.06f }), safeC, 16);
        solidCube(cube, sh, TRS(pos + glm::vec3((float)sides[si] * (BL / 2 + 0.08f), topY + 0.1f, 0),
            { 0,0,0 }, { 0.06f,0.16f,BW * 0.92f }), safeC, 16);
    }
}

// ============================================================
//  drawToolTrolley
// ============================================================
void drawToolTrolley(Cube& cube, Shader& sh, glm::vec3 pos, float yaw = 0)
{
    const glm::vec3 redC(0.78f, 0.12f, 0.05f);
    const glm::vec3 chrC(0.80f, 0.80f, 0.82f);
    glm::mat4 base = myRotate(glm::translate(glm::mat4(1.0f), pos), glm::radians(yaw), glm::vec3(0, 1, 0));
    auto B = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float shine = 32) {
        drawCube(cube, sh, glm::scale(glm::translate(base, t), s), c * 0.3f, c, c * 0.5f, shine, 0);
        sh.setBool("useTexture", false);
        };
    B({ 0,0.6f,0 }, { 0.70f,1.10f,0.45f }, redC, 32);
    for (int i = 0; i < 5; i++) {
        float dy = 0.16f + i * 0.18f;
        B({ 0,dy,0.23f }, { 0.62f,0.14f,0.02f }, chrC * 0.8f, 64);
        B({ 0,dy + 0.04f,0.24f }, { 0.30f,0.04f,0.02f }, chrC, 128);
    }
    B({ 0,1.17f,0 }, { 0.72f,0.06f,0.47f }, glm::vec3(0.15f), 128);
    float wxs[2] = { -0.28f,0.28f }, wzs[2] = { -0.18f,0.18f };
    for (int ix = 0; ix < 2; ix++) for (int iz = 0; iz < 2; iz++)
        B({ wxs[ix],-0.02f,wzs[iz] }, { 0.08f,0.08f,0.08f }, { 0.1f,0.1f,0.1f }, 8);
}

// ============================================================
//  drawAirCompressor
// ============================================================
void drawAirCompressor(Cube& cube, Shader& sh, glm::vec3 pos)
{
    const glm::vec3 blkC(0.10f, 0.12f, 0.18f);
    const glm::vec3 slvC(0.65f, 0.65f, 0.68f);
    auto B = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float shine = 32) {
        drawCube(cube, sh, glm::scale(glm::translate(glm::mat4(1.0f), pos + t), s), c * 0.3f, c, c * 0.5f, shine, 0);
        sh.setBool("useTexture", false);
        };
    B({ 0,0.55f,0 }, { 1.6f,0.5f,0.48f }, blkC, 32);
    float lxs[2] = { -0.6f,0.6f }, lzs[2] = { -0.18f,0.18f };
    for (int ix = 0; ix < 2; ix++) for (int iz = 0; iz < 2; iz++)
        B({ lxs[ix],0.28f,lzs[iz] }, { 0.08f,0.56f,0.08f }, slvC, 32);
    B({ 0.4f,0.85f,0 }, { 0.35f,0.22f,0.32f }, { 0.3f,0.3f,0.35f }, 32);
    B({ -0.7f,0.62f,0 }, { 0.06f,0.08f,0.06f }, slvC, 64);
}

// ============================================================
//  drawDiagnosticArm
// ============================================================
void drawDiagnosticArm(Cube& cube, Shader& sh, glm::vec3 pos)
{
    auto B = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float shine = 32) {
        drawCube(cube, sh, glm::scale(glm::translate(glm::mat4(1.0f), pos + t), s), c * 0.3f, c, c * 0.5f, shine, 0);
        sh.setBool("useTexture", false);
        };
    B({ 0,0.9f,0 }, { 0.08f,1.8f,0.08f }, { 0.25f,0.25f,0.28f }, 32);
    B({ 0.4f,1.7f,0 }, { 0.80f,0.06f,0.06f }, { 0.25f,0.25f,0.28f }, 32);
    B({ 0.8f,1.65f,0 }, { 0.38f,0.28f,0.05f }, { 0.05f,0.05f,0.05f }, 128);
    cube.drawEmissive(sh,
        glm::scale(glm::translate(glm::mat4(1.0f), pos + glm::vec3(0.8f, 1.65f, 0.03f)),
            glm::vec3(0.34f, 0.24f, 0.01f)), { 0.3f,0.7f,1.0f });
}

// ============================================================
//  drawPartsShelf
// ============================================================
void drawPartsShelf(Cube& cube, Shader& sh, glm::vec3 pos, float yaw = 0)
{
    const glm::vec3 metC(0.55f, 0.55f, 0.58f);
    glm::mat4 base = myRotate(glm::translate(glm::mat4(1.0f), pos), glm::radians(yaw), glm::vec3(0, 1, 0));
    auto B = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2 = 16) {
        drawCube(cube, sh, glm::scale(glm::translate(base, t), s), c * 0.3f, c, c * 0.4f, sh2, 0);
        sh.setBool("useTexture", false);
        };
    float uxs[2] = { -1.2f,1.2f };
    for (int ix = 0; ix < 2; ix++) {
        B({ uxs[ix],1.0f,0 }, { 0.06f,2.0f,0.06f }, metC, 32);
        B({ uxs[ix],1.0f,0.45f }, { 0.06f,2.0f,0.06f }, metC, 32);
    }
    for (int i = 0; i < 4; i++) {
        float y = 0.4f + i * 0.5f;
        B({ 0,y,0.24f }, { 2.4f,0.05f,0.50f }, { 0.4f,0.4f,0.42f }, 16);
        float bxs[3] = { -0.8f,0.0f,0.8f };
        for (int bi = 0; bi < 3; bi++)
            B({ bxs[bi],y + 0.12f,0.2f }, { 0.22f,0.18f,0.3f }, { 0.2f + bxs[bi] * 0.1f,0.3f,0.4f }, 8);
    }
}

// ============================================================
//  drawPaintBooth
// ============================================================
void drawPaintBooth(Cube& cube, Shader& sh, glm::vec3 pos)
{
    const glm::vec3 glassC(0.5f, 0.75f, 0.9f);
    const glm::vec3 frameC(0.25f, 0.25f, 0.28f);
    float BW = 6.5f, BD = 5.5f, BH = 4.5f;
    auto F = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2, unsigned int tx) {
        drawCube(cube, sh, glm::scale(glm::translate(glm::mat4(1.0f), pos + t), s), c * 0.3f, c, c * 0.5f, sh2, tx);
        };
    float fxA[] = { 0,BW }; float fzA[] = { 0,BD };
    for (int ix = 0; ix < 2; ix++) for (int iz = 0; iz < 2; iz++) {
        F({ fxA[ix],BH / 2,fzA[iz] }, { 0.12f,BH,0.12f }, frameC, 32, 0);
        sh.setBool("useTexture", false);
    }
    F({ BW / 2,BH / 2,0 }, { BW - 0.24f,BH - 0.3f,0.06f }, glassC, 128, texWindow);
    F({ BW / 2,BH / 2,BD }, { BW - 0.24f,BH - 0.3f,0.06f }, glassC, 128, texWindow);
    F({ 0,BH / 2,BD / 2 }, { 0.06f,BH - 0.3f,BD - 0.24f }, glassC, 128, texWindow);
    F({ BW / 2,BH,BD / 2 }, { BW,0.12f,BD }, frameC, 32, 0);
    sh.setBool("useTexture", false);
    F({ BW / 2,0.06f,BD / 2 }, { BW - 0.3f,0.12f,BD - 0.3f }, { 0.35f,0.35f,0.38f }, 16, texConcrete);
    float fxB[] = { BW * 0.3f, BW * 0.7f };
    for (int ix = 0; ix < 2; ix++) {
        F({ fxB[ix],BH - 0.05f,BD / 2 }, { 0.6f,0.15f,0.6f }, { 0.15f,0.15f,0.18f }, 32, 0);
        sh.setBool("useTexture", false);
    }
}

// ============================================================
//  drawCeilingFan
// ============================================================
void drawCeilingFan(Cube& cube, Shader& sh, glm::vec3 pos, float angle)
{
    const glm::vec3 hubC(0.22f, 0.22f, 0.25f);
    const glm::vec3 bladeC(0.55f, 0.38f, 0.20f);
    solidCube(cube, sh, TRS(pos + glm::vec3(0, 0.35f, 0), { 0,0,0 }, { 0.07f,0.7f,0.07f }), hubC, 32);
    solidCube(cube, sh, TRS(pos, { 0,0,0 }, { 0.28f,0.14f,0.28f }), hubC, 64);
    for (int i = 0; i < 4; i++) {
        float bAngle = angle + i * 90.0f;
        glm::mat4 bm = glm::translate(glm::mat4(1.0f), pos);
        bm = myRotate(bm, glm::radians(bAngle), glm::vec3(0, 1, 0));
        bm = glm::translate(bm, glm::vec3(0.55f, -0.03f, 0));
        bm = glm::scale(bm, glm::vec3(0.90f, 0.04f, 0.26f));
        drawCube(cube, sh, bm, bladeC * 0.3f, bladeC, glm::vec3(0.1f), 16, texWood);
    }
}

// ============================================================
//  drawWorker  – simple cube humanoid worker
//  pos  : foot position (ground)
//  yaw  : facing direction (degrees)
//  shirtCol : torso/shirt colour
//  armSwing : arm swing angle offset (radians, for walking animation)
//  hasHat   : true = yellow hard hat
// ============================================================
void drawWorker(Cube& cube, Shader& sh,
    glm::vec3 pos, float yaw,
    glm::vec3 shirtCol, float armSwing = 0.f, bool hasHat = true)
{
    glm::vec3 skinC(0.88f,0.70f,0.55f);
    glm::vec3 pantC(0.15f,0.15f,0.20f);
    glm::vec3 shoeC(0.10f,0.08f,0.08f);
    glm::vec3 hatC (0.90f,0.75f,0.05f);

    glm::mat4 base = glm::translate(glm::mat4(1.f), pos);
    base = myRotate(base, glm::radians(yaw), glm::vec3(0,1,0));

    auto P = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2=32) {
        solidCube(cube,sh,glm::scale(glm::translate(base,t),s),c,sh2); };
    auto PR = [&](glm::vec3 t, float rx, float ry, float rz, glm::vec3 s, glm::vec3 c) {
        glm::mat4 m=glm::translate(base,t);
        if(rx!=0) m=myRotate(m,rx,glm::vec3(1,0,0));
        if(ry!=0) m=myRotate(m,ry,glm::vec3(0,1,0));
        if(rz!=0) m=myRotate(m,rz,glm::vec3(0,0,1));
        solidCube(cube,sh,glm::scale(m,s),c,32); };

    // Legs
    PR({-0.12f,0.4f,0},  armSwing*.4f,0,0, {0.20f,0.72f,0.20f}, pantC);
    PR({ 0.12f,0.4f,0}, -armSwing*.4f,0,0, {0.20f,0.72f,0.20f}, pantC);
    // Shoes
    P({-0.12f,0.04f, 0.06f},{0.22f,0.08f,0.28f},shoeC);
    P({ 0.12f,0.04f, 0.06f},{0.22f,0.08f,0.28f},shoeC);
    // Torso
    P({0,1.10f,0},{0.50f,0.70f,0.30f},shirtCol,48);
    // Head
    P({0,1.82f,0},{0.40f,0.44f,0.38f},skinC,32);
    // Hard hat
    if(hasHat){
        P({0,2.12f,0},{0.52f,0.10f,0.50f},hatC,64);
        P({0,2.06f,0},{0.60f,0.06f,0.58f},hatC,32);
    }
    // Arms
    PR({-0.32f,1.35f,0},  armSwing,0,0, {0.20f,0.62f,0.20f},shirtCol);
    PR({ 0.32f,1.35f,0}, -armSwing,0,0, {0.20f,0.62f,0.20f},shirtCol);
    // Hands
    PR({-0.32f,0.98f, 0.05f},  armSwing,0,0, {0.18f,0.18f,0.18f},skinC);
    PR({ 0.32f,0.98f, 0.05f}, -armSwing,0,0, {0.18f,0.18f,0.18f},skinC);
}

// ============================================================
//  drawRoboticArm  – 3-DOF industrial robot arm
//  base    : floor position of robot base
//  yaw     : base rotation (degrees, animated)
//  reach   : how extended the arm is (0..1)
// ============================================================
void drawRoboticArm(Cube& cube, Shader& sh,
    glm::vec3 base, float yaw, float reach)
{
    glm::vec3 armC(0.92f,0.40f,0.05f);  // orange industrial robot colour
    glm::vec3 jntC(0.22f,0.22f,0.26f);  // dark joint

    glm::mat4 bm = glm::translate(glm::mat4(1.f), base);
    bm = myRotate(bm, glm::radians(yaw), glm::vec3(0,1,0));

    auto S = [&](glm::mat4 m, glm::vec3 s, glm::vec3 c) {
        solidCube(cube,sh,glm::scale(m,s),c,64); };

    // Base platform
    S(glm::translate(glm::mat4(1.f),base), {0.90f,0.22f,0.90f}, jntC);
    // Rotating column
    S(glm::translate(bm,{0,1.1f,0}), {0.36f,2.20f,0.36f}, armC);
    // Joint 1
    S(glm::translate(bm,{0,2.25f,0}), {0.50f,0.30f,0.50f}, jntC);

    // Upper arm (tilted forward based on reach)
    glm::mat4 ua = glm::translate(bm,{0,2.25f,0});
    float tilt1 = -20.f + reach*60.f;
    ua = myRotate(ua, glm::radians(tilt1), glm::vec3(0,0,1));
    S(glm::translate(ua,{0.75f,0,0}), {1.50f,0.22f,0.22f}, armC);

    // Joint 2
    glm::mat4 j2 = glm::translate(ua,{1.50f,0,0});
    S(j2, {0.30f,0.30f,0.30f}, jntC);

    // Forearm
    glm::mat4 fa = j2;
    float tilt2 = 30.f - reach*70.f;
    fa = myRotate(fa, glm::radians(tilt2), glm::vec3(0,0,1));
    S(glm::translate(fa,{0.60f,0,0}), {1.20f,0.18f,0.18f}, armC);

    // End effector / gripper
    glm::mat4 ee = glm::translate(fa,{1.20f,0,0});
    S(ee, {0.20f,0.36f,0.10f}, jntC);
    S(glm::translate(ee,{0.14f, 0.12f,0}), {0.28f,0.08f,0.08f}, armC);
    S(glm::translate(ee,{0.14f,-0.12f,0}), {0.28f,0.08f,0.08f}, armC);
}

// ============================================================
//  drawConveyorBelt  – animated assembly conveyor
//  pos    : start position (one end of belt)
//  length : belt length along X
//  offset : animated belt-stripe offset (0..1)
// ============================================================
void drawConveyorBelt(Cube& cube, Shader& sh,
    glm::vec3 pos, float length, float offset)
{
    glm::vec3 frameC(0.40f,0.40f,0.44f);
    glm::vec3 beltC (0.12f,0.12f,0.12f);
    glm::vec3 stripeC(0.72f,0.60f,0.05f);  // amber belt stripes

    auto S = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2=32) {
        solidCube(cube,sh, glm::scale(glm::translate(glm::mat4(1.f),pos+t),s), c,sh2); };
    auto E = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c) {
        cube.drawEmissive(sh, glm::scale(glm::translate(glm::mat4(1.f),pos+t),s), c); };

    // Side rails
    S({length/2, 1.05f,-0.55f},{length,0.12f,0.12f},frameC,64);
    S({length/2, 1.05f, 0.55f},{length,0.12f,0.12f},frameC,64);
    S({length/2, 0.55f,-0.55f},{length,0.12f,0.12f},frameC,32);
    S({length/2, 0.55f, 0.55f},{length,0.12f,0.12f},frameC,32);
    // Legs every 3 units
    for(int li=0; li<=(int)(length/3); li++){
        float lx=li*3.f;
        S({lx,0.55f,-0.55f},{0.10f,1.10f,0.10f},frameC,32);
        S({lx,0.55f, 0.55f},{0.10f,1.10f,0.10f},frameC,32);
    }
    // Belt surface
    S({length/2,1.12f,0},{length,0.06f,1.00f},beltC,8);

    // Animated stripes (amber markers every 2 units, shifted by offset)
    float stripeSpacing = 2.0f;
    for(float sx=fmodf(offset*stripeSpacing, stripeSpacing);
        sx < length; sx += stripeSpacing)
    {
        E({sx, 1.15f, 0}, {0.10f,0.02f,0.90f}, stripeC);
    }
}

// ============================================================
//  drawSprayNozzle  – paint shop spray arm
// ============================================================
void drawSprayNozzle(Cube& cube, Shader& sh,
    glm::vec3 pos, float angle, bool active)
{
    glm::vec3 armC(0.50f,0.50f,0.55f);
    glm::vec3 nozzC(0.20f,0.20f,0.22f);

    glm::mat4 bm = glm::translate(glm::mat4(1.f), pos);
    bm = myRotate(bm, glm::radians(angle), glm::vec3(0,1,0));

    auto S = [&](glm::mat4 m, glm::vec3 s, glm::vec3 c) {
        solidCube(cube,sh,glm::scale(m,s),c,32); };

    S(glm::translate(glm::mat4(1.f),pos), {0.12f,3.5f,0.12f}, armC);   // vertical pole
    S(glm::translate(bm,{1.0f,3.2f,0}),  {2.0f,0.10f,0.10f}, armC);  // horizontal arm
    // Nozzle at arm end
    glm::mat4 nm = glm::translate(bm,{2.0f,3.0f,0});
    nm = myRotate(nm, glm::radians(-30.f), glm::vec3(0,0,1));
    S(nm, {0.08f,0.40f,0.08f}, nozzC);

    if(active){
        // Spray cone (emissive)
        glm::mat4 sc = glm::translate(bm, {2.0f,2.6f,0});
        cube.drawEmissive(sh, glm::scale(sc,{0.30f,0.50f,0.30f}),
            glm::vec3(0.60f,0.75f,0.95f));
    }
}

// Forward declarations for functions defined later in this file
void drawSteelTruss(Cube&, Shader&, glm::vec3, float, float);
void drawFireExtinguisher(Cube&, Shader&, glm::vec3);
void drawSofaChair(Cube&, Shader&, glm::vec3, float);
void drawCoffeeTable(Cube&, Shader&, glm::vec3);
void drawTVScreen(Cube&, Shader&, glm::vec3, float, float, float);

// ============================================================
//  drawComplexBuilding
//  4-section car manufacturing complex:
//   Section 0: Assembly Bay   (X: 0..50)
//   Section 1: Paint Shop     (X: 50..100)
//   Section 2: Body Shop      (X: 100..150)
//   Section 3: Showroom       (X: 150..200)
// ============================================================
void drawComplexBuilding(Cube& cube, Sphere& sphere, Shader& sh,
    glm::vec3 orig,
    float convOff, float robotAng, float sprayAng, bool sprayOn, float pressH,
    float workerSwing)
{
    const float BW=200.f, BD=48.f, BH=14.f, SW=50.f, flT=0.18f;
    const glm::vec3 wallC (0.90f,0.91f,0.93f);
    const glm::vec3 colC  (0.22f,0.22f,0.26f);
    const glm::vec3 roofC (0.44f,0.43f,0.42f);
    const glm::vec3 ceilC (0.86f,0.86f,0.88f);

    auto S = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2=32) {
        solidCube(cube,sh,
            glm::scale(glm::translate(glm::mat4(1.f),orig+t),s), c,sh2); };
    auto E = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c) {
        cube.drawEmissive(sh,
            glm::scale(glm::translate(glm::mat4(1.f),orig+t),s), c); };
    auto W = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2=16, unsigned int tx=0) {
        drawCube(cube,sh,
            glm::scale(glm::translate(glm::mat4(1.f),orig+t),s),
            c*.3f,c,c*.5f,sh2,tx); };

    // ── SECTION FLOORS ───────────────────────────────────────────────────────
    // Assembly: dark concrete with yellow safety grid
    S({SW*.5f,      -flT*.5f, BD*.5f}, {SW,  flT, BD}, {0.08f,0.09f,0.10f},  8);
    for(int gi=0;gi<=5;gi++){ float gx=gi*(SW/5.f);
        E({gx,-flT+.005f,BD*.5f},{.04f,.01f,BD},{.85f,.75f,.05f}); }
    // Paint Shop: clean bright white
    S({SW*1.5f,     -flT*.5f, BD*.5f}, {SW,  flT, BD}, {0.88f,0.88f,0.90f}, 64);
    // ── PAINT ZONE spotlight fixture on ceiling (above red box, world X=75, Z=24) ──
    {
        // Ceiling mount bracket
        S({SW+25.f, BH-0.1f, 24.f}, {1.2f,0.20f,1.2f}, {0.25f,0.25f,0.28f}, 32);
        // Lamp housing (dark cone-shaped body)
        S({SW+25.f, BH-0.8f, 24.f}, {0.9f,1.20f,0.9f}, {0.18f,0.18f,0.20f}, 32);
        // Emissive globe at the tip – bright warm white
        E({SW+25.f, BH-1.5f, 24.f}, {0.55f,0.55f,0.55f}, {1.0f,0.95f,0.82f});
    }
    // ── RED PAINT ZONE – one-car-sized rectangle on the floor ───────────────
    // Car footprint ≈ 4.4 wide × 8.5 long  →  box is 7 × 11 (snug single car)
    // Centred in the paint section (world X≈75, Z≈24)
    {
        const float Y  = 0.10f;   // raised above all ground/tile surfaces
        const float TH = 0.06f;   // line height
        const float LW = 0.35f;   // line width
        const float x1 = SW+21.f, x2 = SW+29.f;  // world X: 71 .. 79  (8 units)
        const float z1 = 18.f,    z2 = 30.f;      // world Z: 18 .. 30  (12 units)
        const float cx = (x1+x2)*.5f, cz = (z1+z2)*.5f;
        E({x1, Y, cz}, {LW, TH, z2-z1}, {1.f,0.f,0.f}); // left
        E({x2, Y, cz}, {LW, TH, z2-z1}, {1.f,0.f,0.f}); // right
        E({cx, Y, z1}, {x2-x1, TH, LW}, {1.f,0.f,0.f}); // front
        E({cx, Y, z2}, {x2-x1, TH, LW}, {1.f,0.f,0.f}); // back
    }
    // Body Shop: premium dark charcoal (motorsport workshop)
    S({SW*2.5f,     -flT*.5f, BD*.5f}, {SW,  flT, BD}, {0.13f,0.13f,0.15f}, 64);
    // Body Shop LED floor accent lines (bright cyan)
    for(int gi=0;gi<=5;gi++){ float gx=SW*2+gi*(SW/5.f);
        E({gx,-flT+.005f,BD*.5f},{.05f,.01f,BD},{.15f,.75f,.95f}); }
    E({SW*2.5f,-flT+.005f,BD*.5f},{SW*.95f,.01f,BD*.95f},{.05f,.12f,.22f});
    // Showroom: warm cream polished marble
    S({SW*3.5f,     -flT*.5f, BD*.5f}, {SW,  flT, BD}, {0.95f,0.93f,0.90f}, 256);
    // Showroom marble vein lines (subtle grey)
    for(int gi=0;gi<5;gi++){ float gz=4.f+gi*(BD/4.5f);
        E({SW*3.5f,.005f,gz},{SW*.92f,.008f,.18f},{.78f,.76f,.74f}); }
    // Showroom floor warm glow (ambient pool under each display)
    for(int di=0;di<3;di++){ float dx=SW*3+10.f+di*15.f;
        E({dx,.005f,BD*.5f},{10.f,.008f,12.f},{.95f,.82f,.50f}); }

    // ── EXTERIOR WALLS ───────────────────────────────────────────────────────
    // Back wall (Z=0)
    W({BW/2, BH/2, .14f}, {BW,BH,.28f}, wallC, 16, texBrick);
    // Left wall (X=0)
    W({.14f, BH/2, BD/2}, {.28f,BH,BD}, wallC, 16, texBrick);
    // Right wall (X=BW)
    W({BW-.14f,BH/2,BD/2},{.28f,BH,BD}, wallC, 16, texBrick);
    // Front wall: solid pillars + header (door opening X:[7,193])
    const float DXL=7.f, DXR=193.f, DOH=9.f;
    W({DXL/2,        BH/2,BD},{DXL*2,    BH,.28f}, wallC, 16, texBrick); // left pillar
    W({(DXR+BW)/2,   BH/2,BD},{(BW-DXR)*2,BH,.28f},wallC, 16, texBrick); // right pillar
    float hdrH=BH-DOH-.05f;
    W({BW/2,DOH+hdrH/2,BD},{DXR-DXL,hdrH,.28f}, wallC, 16, texBrick);  // header
    // Top fascia
    W({BW/2,BH-.3f,BD},{BW,.6f,.35f}, wallC, 16, texBrick);
    // Door frame
    S({DXL,   DOH/2,BD+.12f},{.28f,DOH,.22f},colC,64);
    S({DXR,   DOH/2,BD+.12f},{.28f,DOH,.22f},colC,64);
    S({BW/2,DOH+.14f,BD+.12f},{DXR-DXL+.30f,.28f,.22f},colC,64);

    // ── PARTITION WALLS (with drive-through passages at Z=21..27) ────────────
    // Wall at X=50
    S({SW,  BH/2,  10.5f},{.28f,BH,21.f},colC,32);
    S({SW,  BH/2,  37.5f},{.28f,BH,21.f},colC,32);
    // Wall at X=100
    S({SW*2,BH/2,  10.5f},{.28f,BH,21.f},colC,32);
    S({SW*2,BH/2,  37.5f},{.28f,BH,21.f},colC,32);
    // Wall at X=150
    S({SW*3,BH/2,  10.5f},{.28f,BH,21.f},colC,32);
    S({SW*3,BH/2,  37.5f},{.28f,BH,21.f},colC,32);
    // Passage headers above drive-through gap
    for(float px : {SW, SW*2, SW*3}){
        S({px,DOH+.5f,24.f},{.28f,BH-DOH-.5f,6.f},colC,32);
        // Door frame strips
        S({px-0.05f,DOH/2,21.f},{.12f,DOH,.20f},colC,64);
        S({px-0.05f,DOH/2,27.f},{.12f,DOH,.20f},colC,64);
    }

    // ── ROOF ─────────────────────────────────────────────────────────────────
    W({BW/2,BH+.15f,BD/2},{BW+.4f,.30f,BD+.4f},roofC,16,texRoof);
    // Roof edge fascia
    S({BW/2,BH,-.15f},{BW+.4f,.6f,.3f},{.20f,.20f,.22f},32);
    S({BW/2,BH,BD+.15f},{BW+.4f,.6f,.3f},{.20f,.20f,.22f},32);
    S({-.15f,BH,BD/2},{.3f,.6f,BD+.4f},{.20f,.20f,.22f},32);
    S({BW+.15f,BH,BD/2},{.3f,.6f,BD+.4f},{.20f,.20f,.22f},32);
    // Ceiling (inside)
    W({BW/2,BH-.08f,BD/2},{BW-.6f,.16f,BD-.6f},ceilC,8,0);
    sh.setBool("useTexture",false);

    // Skylights (emissive ceiling windows)
    for(int si=0;si<4;si++){
        float sx=SW*si+SW*.5f;
        E({sx,BH-.05f,BD/2},{SW*.5f,.04f,BD*.3f},{.85f,.90f,1.f});
    }

    // ── SECTION SIGNS (emissive coloured header strips) ──────────────────────
    glm::vec3 secCols[4]={{.05f,.55f,1.f},{.8f,.2f,.8f},{1.f,.5f,.05f},{.05f,.75f,.35f}};
    const char* labels[4]={"ASSEMBLY","PAINT SHOP","BODY SHOP","SHOWROOM"};
    for(int si=0;si<4;si++){
        float sx=SW*si+SW*.5f;
        E({sx,BH-.25f,BD-.3f},{SW*.60f,.35f,.05f},secCols[si]);
        S({sx,BH-.25f,BD-.35f},{SW*.62f,.45f,.08f},{.05f,.05f,.06f},128);
    }

    // ── OVERHEAD TRUSSES + LED strips (per section) ───────────────────────────
    for(int si=0;si<4;si++){
        float sx=SW*si+SW*.5f;
        // 2 trusses per section
        drawSteelTruss(cube,sh, orig+glm::vec3(SW*si,BH-1.4f,BD*.3f), SW, 1.35f);
        drawSteelTruss(cube,sh, orig+glm::vec3(SW*si,BH-1.4f,BD*.7f), SW, 1.35f);
        // LED strip
        E({sx,BH-1.5f,BD*.3f},{SW*.8f,.04f,.06f},{.95f,.95f,.85f});
        E({sx,BH-1.5f,BD*.7f},{SW*.8f,.04f,.06f},{.95f,.95f,.85f});
        // Ceiling fan
        drawCeilingFan(cube,sh, orig+glm::vec3(sx,BH-.6f,BD*.5f), fanAngle+si*45.f);
    }

    // ── [SECTION 0] ASSEMBLY BAY ──────────────────────────────────────────────
    {
        glm::vec3 O=orig+glm::vec3(0,0,0);

        // ── 3 × 4-post hydraulic lifts (front zone, z≈10) ────────────────────
        // Lift 0 (key J) — left bay
        draw4PostLift(cube,sh, O+glm::vec3( 8.f,0.f,10.f), liftH[0]);
        drawAttractiveCar(cube,sh,
            O+glm::vec3( 8.f, liftH[0]+0.14f, 10.f), 0.f, {.18f,.55f,.20f});
        // Lift 1 (key K) — centre bay
        draw4PostLift(cube,sh, O+glm::vec3(26.f,0.f,10.f), liftH[1]);
        drawAttractiveCar(cube,sh,
            O+glm::vec3(26.f, liftH[1]+0.14f, 10.f), 0.f, {.60f,.18f,.18f});
        // Lift 4 (key M) — NEW right bay
        draw4PostLift(cube,sh, O+glm::vec3(43.f,0.f,10.f), liftH[4]);
        drawAttractiveCar(cube,sh,
            O+glm::vec3(43.f, liftH[4]+0.14f, 10.f), 0.f, {.20f,.20f,.65f});
        // Emissive inspection lights above each lift (lifts 0&1 backed by sectionSpots 8&9)
        for (float lx : {8.f, 26.f, 43.f}) {
            S({lx, BH-0.08f, 10.f}, {1.1f,0.16f,1.1f}, {0.20f,0.20f,0.24f}, 32); // bracket
            S({lx, BH-0.70f, 10.f}, {0.80f,1.0f,0.80f}, {0.14f,0.14f,0.16f}, 32); // housing
            sphere.drawEmissive(sh, glm::scale(glm::translate(glm::mat4(1.f),
                O+glm::vec3(lx,BH-1.3f,10.f)),glm::vec3(.20f)),{1.f,.98f,.90f}); // globe
            E({lx, BH-0.04f, 10.f}, {1.8f,0.08f,1.8f}, {0.95f,0.94f,0.82f}); // cone hint
        }

        // ── 2 × scissor lifts (rear zone, z≈36) ──────────────────────────────
        // Lift 2 (key V)
        drawScissorLift(cube,sh, O+glm::vec3(12.f,0.f,36.f), liftH[2]);
        // Lift 3 (key N)
        drawScissorLift(cube,sh, O+glm::vec3(35.f,0.f,36.f), liftH[3]);

        // ── Conveyor belt (X: 4..46, Z=24) ───────────────────────────────────
        drawConveyorBelt(cube,sh, O+glm::vec3(4.f,0.f,22.f), 42.f, convOff);
        // 3 car bodies on belt (being assembled)
        float bodyPositions[3]={8.f,22.f,36.f};
        for(int bi=0;bi<3;bi++){
            float bx=bodyPositions[bi]+fmodf(convOff*8.f,42.f);
            if(bx>46.f) bx-=42.f;
            glm::mat4 cm=glm::translate(glm::mat4(1.f),O+glm::vec3(bx,1.15f,24.f));
            solidCube(cube,sh,glm::scale(cm,{4.0f,.5f,1.8f}),{.20f,.30f,.80f},32);
            solidCube(cube,sh,glm::scale(glm::translate(cm,{0,.4f,0}),{2.8f,.52f,1.68f}),{.22f,.32f,.82f},32);
        }
        // 2 robotic arms (sides of conveyor)
        drawRoboticArm(cube,sh, O+glm::vec3(15.f,0.f,20.5f), robotAng,
            .5f+.5f*sinf(glm::radians(robotAng)));
        drawRoboticArm(cube,sh, O+glm::vec3(30.f,0.f,27.5f), robotAng+120.f,
            .5f+.5f*sinf(glm::radians(robotAng+120.f)));
        // Workers
        drawWorker(cube,sh, O+glm::vec3(18.f,0.f,20.f),  45.f,{.20f,.40f,.75f}, workerSwing);
        drawWorker(cube,sh, O+glm::vec3(32.f,0.f,27.5f),-90.f,{.20f,.40f,.75f},-workerSwing);
        /*drawWorker(cube,sh, O+glm::vec3(12.f,0.f,36.f), 180.f,{.20f,.40f,.75f}, workerSwing);*/
      /*  drawWorker(cube,sh, O+glm::vec3(36.f,0.f,36.f),  90.f,{.20f,.40f,.75f},-workerSwing*.5f);*/
        // Parts bins along back wall (x=4,14,28,40)
        for(float px : {4.f,14.f,28.f,40.f}){
            int pi=(int)(px/10);
            S({px,0.4f,2.5f},{2.5f,.8f,1.8f},{.30f,.30f,.34f},16);
            S({px,.62f,2.5f},{2.2f,.20f,1.5f},{.15f+pi*.10f,.25f,.60f},8);
        }
        // Overhead hoist rail
        S({SW/2,BH-2.0f,24.f},{SW-.5f,.12f,.20f},{.45f,.45f,.48f},32);
        // Animated hoist trolley
        float hoistX=SW*.5f+SW*.4f*sinf(glm::radians(robotAng*.3f));
        S({hoistX,BH-2.5f,24.f},{.40f,.60f,.30f},{.30f,.30f,.34f},32);
        S({hoistX,BH-3.2f,24.f},{.06f,1.50f,.06f},{.60f,.60f,.62f},64);
        // Safety signage strip
        E({SW*.5f,.01f,BD-.5f},{SW*.8f,.02f,.40f},{.90f,.10f,.05f});

        // ── REPAIR ZONE – green rectangle on floor (world RPZX1..RPZX2, RPZZ1..RPZZ2) ──
        {
            const float Y  = 0.10f;   // raised above all ground/tile surfaces
            const float TH = 0.06f;
            const float LW = 0.35f;
            // world X: 18..30 (orig.x=0), Z: 36..46
            const float x1 = 18.f, x2 = 30.f;
            const float z1 = 36.f, z2 = 46.f;
            const float cx = (x1+x2)*.5f, cz = (z1+z2)*.5f;
            E({x1, Y, cz}, {LW, TH, z2-z1}, {0.f,1.f,0.f}); // left
            E({x2, Y, cz}, {LW, TH, z2-z1}, {0.f,1.f,0.f}); // right
            E({cx, Y, z1}, {x2-x1, TH, LW}, {0.f,1.f,0.f}); // front
            E({cx, Y, z2}, {x2-x1, TH, LW}, {0.f,1.f,0.f}); // back
        }
    }

    // ── [SECTION 1] PAINT SHOP ────────────────────────────────────────────────
    {
        glm::vec3 O=orig+glm::vec3(SW,0.f,0.f);
        glm::vec3 gC{.55f,.80f,.95f};
        const float DOOR_SLIDE_H = 6.0f; // door panel full height

        // ── 2 enclosed paint booths ───────────────────────────────────────────
        for(int bi=0;bi<2;bi++){
            float bz=8.f+bi*22.f;
            float bx=6.f;
            // Glass walls
            W({bx,3.f,bz},{.06f,6.f,14.f},gC,128,texWindow);    // left wall
            W({bx+12.f,3.f,bz},{.06f,6.f,14.f},gC,128,texWindow); // right wall
            W({bx+6.f,3.f,bz-7.f},{12.f,6.f,.06f},gC,128,texWindow); // back wall
            // Front sliding door: when paintDoorAmt>0, panel rises upward
            {
                float panelY = 3.0f + paintDoorAmt * DOOR_SLIDE_H;
                W({bx+6.f,panelY,bz+7.f},{12.f,6.f,.06f},gC,128,texWindow);
                // Door frame
                S({bx+.1f, 3.5f, bz+7.f},{.14f,7.f,.14f},{.22f,.22f,.26f},32);
                S({bx+12.f-.1f,3.5f,bz+7.f},{.14f,7.f,.14f},{.22f,.22f,.26f},32);
                // Status indicator: green=open, red=closed
                glm::vec3 indicC = (paintDoorAmt>0.5f) ?
                    glm::vec3(.05f,.90f,.10f) : glm::vec3(.90f,.05f,.05f);
                E({bx+6.f,7.2f,bz+7.3f},{1.5f,.20f,.08f},indicC);
            }
            W({bx+6.f,6.1f,bz},{12.f,.12f,14.f},{.30f,.30f,.32f},32,0);
            sh.setBool("useTexture",false);
            E({bx+6.f,6.2f,bz},{10.f,.04f,.60f},{.75f,.85f,1.f}); // vent
            // Car inside booth (rotates on turntable when sprayOn)
            float carYaw = (bi==0) ? paintTurntableAngle : paintTurntableAngle+180.f;
            // Turntable platform
            S({bx+6.f,.08f,bz},{4.8f,.16f,4.8f},{.38f,.38f,.40f},32);
            E({bx+6.f,.16f,bz},{4.6f,.02f,4.6f},{.10f,.35f,.70f}); // blue ring
            drawAttractiveCar(cube,sh,
                orig+glm::vec3(SW+bx+6.f, 0.18f, bz),
                carYaw, {.8f+(float)bi*.1f,.1f,(float)bi*.4f});
            // 4 spray nozzles around booth, all animated
            for(int ni=0;ni<4;ni++){
                float nAng = sprayAng + bi*60.f + ni*90.f;
                float nz_off = (ni%2==0) ? -3.f : 3.f;
                float nx_off = (ni<2)    ?  1.f : 11.f;
                drawSprayNozzle(cube,sh, O+glm::vec3(bx+nx_off,0.f,bz+nz_off),
                    nAng, sprayOn);
            }
            // UV curing lamp arm — swings over car
            {
                glm::mat4 uvBase = glm::translate(glm::mat4(1.f),
                    O+glm::vec3(bx+6.f, BH-1.5f, bz));
                uvBase = myRotate(uvBase,
                    glm::radians(sprayAng*0.5f + bi*90.f), glm::vec3(0,1,0));
                solidCube(cube,sh,
                    glm::scale(glm::translate(uvBase,{0,0,0}),{.10f,.10f,5.5f}),
                    {.35f,.35f,.38f},32);
                // UV emissive bar at tip
                cube.drawEmissive(sh,
                    glm::scale(glm::translate(uvBase,{0,-.08f,2.5f}),{.08f,.05f,4.5f}),
                    {.55f,.15f,.95f}); // UV purple
            }
        }

        // ── Paint mixing station (right wall, x=33..48) ───────────────────────
        // Base cabinet
        S({40.f,1.0f,24.f},{8.f,2.f,12.f},{.25f,.25f,.30f},32);
        // Rotating mixing drums (3 drums, animated with sprayAngle)
        for(int di=0;di<3;di++){
            float dz=19.f+di*4.f;
            float drumYaw=sprayAng*2.f+(float)di*120.f;
            glm::mat4 dm=glm::translate(glm::mat4(1.f),O+glm::vec3(40.f,2.7f,dz));
            dm=myRotate(dm, glm::radians(drumYaw), glm::vec3(0,1,0));
            // Drum body
            solidCube(cube,sh,glm::scale(dm,{1.2f,1.2f,1.2f}),
                {.30f+di*.10f,.20f,.55f-.1f*di},32);
            // Drum stirrer arm
            solidCube(cube,sh,
                glm::scale(glm::translate(dm,{.8f,0,0}),{1.4f,.08f,.08f}),
                {.60f,.60f,.62f},64);
            // Colour top indicator
            glm::vec3 cc{(float)di*.25f,.20f,.70f-.2f*di};
            cube.drawEmissive(sh,
                glm::scale(glm::translate(glm::mat4(1.f),O+glm::vec3(40.f,3.4f,dz)),
                    {.8f,.12f,.8f}),cc);
        }
        // Paint hose rack
        S({33.f,1.5f,12.f},{.15f,3.f,8.f},{.25f,.25f,.30f},16);
        for(int hi=0;hi<3;hi++){
            float hz=13.f+hi*2.8f;
            S({33.1f,2.5f+hi*.4f,hz},{.5f,.4f,.4f},{.65f,.20f,.08f},8); // coiled hose
            E({33.1f,2.5f+hi*.4f,hz},{.45f,.35f,.35f},{.70f,.25f,.08f});
        }
        // Paint-shop workers (white coveralls)
        glm::vec3 whiteC{.90f,.90f,.92f};
        drawWorker(cube,sh, O+glm::vec3(5.f,  0.f,10.f),180.f,whiteC, workerSwing,false);
        drawWorker(cube,sh, O+glm::vec3(13.f, 0.f,10.f),  0.f,whiteC,-workerSwing,false);
        drawWorker(cube,sh, O+glm::vec3(5.f,  0.f,32.f), 90.f,whiteC, workerSwing,false);
        drawWorker(cube,sh, O+glm::vec3(13.f, 0.f,32.f),-90.f,whiteC,-workerSwing,false);
        // ── Booth ceiling spotlight fixtures (spots 1 & 2 of sectionSpots) ──────
        // Booth 1 (world x=62, z=10) local: x=12, z=10
        // Booth 2 (world x=62, z=30) local: x=12, z=30
        for (float bz : {10.f, 30.f}) {
            S({12.f, BH-0.1f, bz}, {1.0f,0.18f,1.0f}, {0.22f,0.22f,0.26f}, 32); // bracket
            S({12.f, BH-0.8f, bz}, {0.75f,1.1f,0.75f}, {0.16f,0.16f,0.18f}, 32); // housing
            E({12.f, BH-1.5f, bz}, {0.50f,0.50f,0.50f}, {0.96f,0.96f,1.00f}); // cool-white globe
        }

        // Air extraction units on ceiling
        for(int ai=0;ai<3;ai++){
            float az=8.f+ai*14.f;
            S({SW*.5f,BH-2.5f,az},{8.f,1.f,4.f},{.55f,.55f,.58f},16);
            E({SW*.5f,BH-2.4f,az},{7.5f,.04f,3.5f},{.60f,.85f,.95f});
        }
        // Floor drain markings (blue lines to drain grating)
        for(int dr=0;dr<3;dr++){
            float dz=10.f+dr*14.f;
            E({SW*.3f,.01f,dz},{SW*.55f,.01f,.12f},{.20f,.45f,.75f});
        }
        // Turntable active glow on floor
        if(sprayOn){
            E({6.f+6.f,.005f, 8.f},{5.f,.01f,5.f},{.10f,.35f,.70f});
            E({6.f+6.f,.005f,30.f},{5.f,.01f,5.f},{.10f,.35f,.70f});
        }
    }

    // ── [SECTION 2] BODY SHOP ─────────────────────────────────────────────────
    // NOTE: BS/BE lambdas add SW*2 (100) to X so content lands in X:[100..150]
    {
        glm::vec3 O=orig+glm::vec3(SW*2,0.f,0.f);
        float OX=SW*2;
        auto BS=[&](glm::vec3 t,glm::vec3 s,glm::vec3 c,float sh2=32){
            solidCube(cube,sh,glm::scale(glm::translate(glm::mat4(1.f),
                orig+glm::vec3(OX,0,0)+t),s),c,sh2); };
        auto BE=[&](glm::vec3 t,glm::vec3 s,glm::vec3 c){
            cube.drawEmissive(sh,glm::scale(glm::translate(glm::mat4(1.f),
                orig+glm::vec3(OX,0,0)+t),s),c); };

        glm::vec3 steelC{.55f,.56f,.60f}, darkMet{.14f,.14f,.16f};
        glm::vec3 tireBlk{.09f,.09f,.10f}, rimSilv{.78f,.78f,.82f};
        glm::vec3 engineGrey{.28f,.30f,.32f}, weldC{.14f,.14f,.18f};
        glm::vec3 pressC{.15f,.30f,.65f};

        // ── Interior wall accents (premium dark workshop) ─────────────────
        // Cyan LED cove strips on all 4 walls
        BE({25.f,BH-.3f, .2f},{SW*.9f,.06f,.10f},{.15f,.75f,.95f}); // back
        BE({25.f,BH-.3f,BD-.2f},{SW*.9f,.06f,.10f},{.15f,.75f,.95f}); // front
        BE({ .3f,BH-.3f,BD*.5f},{.10f,.06f,BD*.85f},{.15f,.75f,.95f}); // left
        BE({49.7f,BH-.3f,BD*.5f},{.10f,.06f,BD*.85f},{.15f,.75f,.95f}); // right
        // Dark wall panelling
        for(int wp=0;wp<5;wp++){
            float wz=3.f+wp*8.8f;
            BS({ .4f,BH*.5f,wz},{.10f,BH*.8f,7.f},{.18f,.18f,.20f},16);
            BS({49.6f,BH*.5f,wz},{.10f,BH*.8f,7.f},{.18f,.18f,.20f},16);
        }

        // ── Hydraulic press (left zone, local x=12 → world x=112) ─────────
        float ph=pressH;
        BS({12.f,BH*.5f,22.f},{1.6f,BH,1.6f},pressC,32);
        BS({12.f,BH-1.f,22.f},{14.f,.90f,8.f},pressC,32);
        BS({12.f,ph+.5f,22.f},{12.f,.80f,6.f},pressC,32);
        for(float gz:{19.f,25.f}) BS({12.f,BH*.4f,gz},{.22f,BH*.7f,.22f},steelC,64);
        if(ph<1.5f){
            for(int sp=0;sp<6;sp++){
                float sx=7.f+(float)sp*2.f, sz=20.f+(float)(sp%3)*2.f;
                BE({sx,ph+.05f,sz},{.08f,.08f,.08f},{1.f,.60f,.05f});
            }
        }
        BE({12.f,.01f,22.f},{16.f,.01f,10.f},{.90f,.10f,.05f}); // danger zone

        // ── Cars being worked on ──────────────────────────────────────────
        drawAttractiveCar(cube,sh, O+glm::vec3( 7.f,0.f,10.f),  90.f,{.55f,.55f,.60f});
        drawAttractiveCar(cube,sh, O+glm::vec3(40.f,0.f,36.f), -90.f,{.50f,.50f,.55f});
        // Jack stands
        for(float jx:{5.f,9.f}){
            BS({jx,0.6f,9.f},{0.3f,1.2f,0.3f},steelC,64);
            BS({jx,1.2f,9.f},{0.8f,0.1f,0.5f},steelC,32);
        }
        // Spotlight emissive globes above each car
        sphere.drawEmissive(sh, glm::scale(glm::translate(glm::mat4(1.f),
            O+glm::vec3(7.f,BH-1.2f,10.f)),glm::vec3(.22f)),{1.f,.97f,.90f});
        sphere.drawEmissive(sh, glm::scale(glm::translate(glm::mat4(1.f),
            O+glm::vec3(40.f,BH-1.2f,36.f)),glm::vec3(.22f)),{1.f,.97f,.90f});
        // Cone hints
        BE({ 7.f,BH-.05f,10.f},{2.f,.10f,2.f},{.95f,.92f,.85f});
        BE({40.f,BH-.05f,36.f},{2.f,.10f,2.f},{.95f,.92f,.85f});

        // ── TIRE SECTION (back wall, local z=1..3) ────────────────────────
        for(int tc=0;tc<5;tc++){
            float tx=3.f+tc*9.f;
            for(int tr=0;tr<4;tr++){
                float ty=0.28f+tr*0.56f;
                BS({tx,ty,2.f},{2.0f,0.50f,2.0f},tireBlk,8);
                BS({tx,ty+.26f,2.f},{1.10f,0.08f,1.10f},rimSilv,64);
                BS({tx,ty+.27f,2.f},{0.28f,0.10f,0.28f},darkMet,32);
            }
            BE({tx,2.7f,2.f},{1.8f,.08f,.20f},{.85f,.75f,.05f});
        }
        // Wall-hung spare tires (vertical)
        for(int wt=0;wt<4;wt++){
            float wx=4.f+wt*10.f;
            BS({wx,5.0f,1.3f},{1.90f,1.90f,.46f},tireBlk,8);
            BS({wx,5.0f,1.1f},{0.80f,0.80f,.60f},darkMet,8);
            BS({wx,5.0f,1.4f},{1.00f,1.00f,.10f},rimSilv,64);
            BS({wx,6.1f,1.1f},{0.10f,.30f,.24f},steelC,64);
        }

        // ── ENGINE SECTION (centre, local x≈28..35) ───────────────────────
        // Engine on workstand
        BS({28.f,.55f,14.f},{4.5f,1.1f,3.f},steelC,32);
        for(float ex:{26.2f,29.8f}) for(float ez:{12.8f,15.2f})
            BS({ex,.28f,ez},{.22f,.56f,.22f},darkMet,32);
        BS({28.f,1.60f,14.f},{3.20f,1.40f,2.20f},engineGrey,16);
        BS({28.f,2.35f,14.f},{3.00f,0.55f,2.00f},{.32f,.34f,.36f},32);
        BS({28.f,2.65f,14.f},{2.60f,0.30f,1.60f},{.72f,.08f,.05f},32); // red valve cover
        for(int cyl=0;cyl<4;cyl++){
            float cx=26.6f+cyl*.55f;
            BS({cx,2.85f,13.4f},{.06f,.55f,.06f},{.10f,.12f,.90f},16);
        }
        BS({28.f,1.05f,14.f},{2.80f,.30f,1.80f},{.20f,.20f,.22f},16); // oil pan
        for(int ep=0;ep<4;ep++){
            float ex2=26.5f+ep*.60f;
            BS({ex2,1.80f,12.5f},{.18f,.80f,.18f},{.50f,.25f,.10f},16);
        }
        BS({28.f,1.55f,11.8f},{3.40f,.22f,.22f},{.48f,.22f,.08f},16);
        BS({29.6f,2.40f,15.4f},{1.10f,.70f,.90f},{.15f,.15f,.18f},32);
        BS({29.6f,2.78f,15.4f},{0.90f,.10f,.70f},{.50f,.50f,.52f},32);
        // Engine hoist chain
        BS({28.f,BH-2.0f,14.f},{.10f,3.5f,.10f},{.60f,.62f,.65f},64);
        BS({28.f,BH-2.2f,14.f},{1.80f,.12f,.12f},steelC,32);

        // Second engine on stand (spare)
        BS({42.f,1.60f,14.f},{2.80f,1.20f,2.00f},engineGrey,16);
        BS({42.f,2.25f,14.f},{2.60f,.50f,1.80f},{.32f,.34f,.36f},32);
        BS({42.f,2.52f,14.f},{2.20f,.24f,1.40f},{.68f,.08f,.05f},32);
        for(float ex:{40.5f,43.5f}) for(float ez:{13.f,15.f})
            BS({ex,.55f,ez},{.18f,1.10f,.18f},steelC,64);

        // ── BODY PANELS rack (right wall, local x≈47) ─────────────────────
        BS({47.f,2.f,24.f},{1.5f,4.f,.18f},steelC,32);
        for(int sh2=0;sh2<4;sh2++){
            float sy=.6f+sh2*.9f;
            BS({47.f,sy,24.f},{1.4f,.08f,18.f},steelC,32);
        }
        glm::vec3 panelCols[]={{.55f,.55f,.60f},{.58f,.56f,.62f},{.52f,.54f,.58f}};
        for(int dp=0;dp<6;dp++){
            float dz=15.f+dp*3.2f;
            BS({46.8f,1.6f,dz},{.10f,2.8f,2.5f},panelCols[dp%3],32);
        }
        // Hood panels
        for(int hp=0;hp<3;hp++){ float hx=3.f+hp*14.f;
            BS({hx,.35f,43.f},{8.f,.08f,7.f},steelC,32); }
        // Bumper shelf
        BS({22.f,1.f,2.f},{18.f,.08f,3.f},steelC,32);
        BS({22.f,1.5f,2.f},{17.5f,.08f,2.8f},{.50f,.20f,.08f},16);
        BS({22.f,2.0f,2.f},{17.2f,.08f,2.6f},{.48f,.20f,.08f},16);

        // ── Rubber tubes / hoses on hooks (front wall) ────────────────────
        for(int hi=0;hi<5;hi++){
            float hx=4.f+hi*9.f;
            BS({hx,5.5f,46.f},{1.4f,1.4f,.35f},{.12f,.12f,.14f},8);
            BS({hx,5.5f,45.9f},{.7f,.7f,.50f},{.10f,.10f,.12f},4);
            BS({hx,6.5f,45.8f},{.08f,.22f,.18f},steelC,64);
        }

        // ── Workers ───────────────────────────────────────────────────────
        drawWorker(cube,sh, O+glm::vec3(10.f,0.f,13.f),  90.f,weldC, workerSwing);
        drawWorker(cube,sh, O+glm::vec3(40.f,0.f,35.f), -90.f,weldC,-workerSwing);
        drawWorker(cube,sh, O+glm::vec3(12.f,0.f,27.f), 180.f,weldC, workerSwing*.5f);
        drawWorker(cube,sh, O+glm::vec3(28.f,0.f,20.f),  45.f,weldC,-workerSwing*.7f);
        BE({10.5f,1.2f,12.f},{.07f,.07f,.07f},{1.f,.65f,.05f});
        BE({40.5f,1.2f,35.f},{.07f,.07f,.07f},{1.f,.65f,.05f});

        // ── Tool cabinets (front wall) ─────────────────────────────────────
        for(int ti=0;ti<5;ti++){
            float tx=3.f+ti*9.f;
            BS({tx,1.0f,45.5f},{3.f,2.f,1.5f},{.12f,.14f,.18f},16); // dark blue-grey cabinets
            BS({tx,2.06f,45.5f},{3.f,.08f,1.5f},{.20f,.22f,.28f},32);
            for(int dr=0;dr<3;dr++)
                BS({tx,.5f+dr*.55f,44.85f},{1.8f,.08f,.06f},{.55f,.58f,.65f},64);
        }

        // ── Overhead rail + animated hoist ─────────────────────────────────
        BS({25.f,BH-1.5f,22.f},{SW-.5f,.12f,.20f},{.40f,.40f,.44f},32);
        float hoistX2=25.f+18.f*sinf(glm::radians(robotAng*.4f));
        BS({hoistX2,BH-2.0f,22.f},{.08f,1.2f,.08f},steelC,64);
        BS({hoistX2,BH-2.1f,22.f},{1.4f,.10f,.10f},steelC,32);
    }

    // ── [SECTION 3] SHOWROOM ──────────────────────────────────────────────────
    // NOTE: SS/SE lambdas add SW*3 (150) to X so content lands in X:[150..200]
    {
        glm::vec3 O=orig+glm::vec3(SW*3,0.f,0.f);
        float OX=SW*3;
        auto SS=[&](glm::vec3 t,glm::vec3 s,glm::vec3 c,float sh2=32){
            solidCube(cube,sh,glm::scale(glm::translate(glm::mat4(1.f),
                orig+glm::vec3(OX,0,0)+t),s),c,sh2); };
        auto SE=[&](glm::vec3 t,glm::vec3 s,glm::vec3 c){
            cube.drawEmissive(sh,glm::scale(glm::translate(glm::mat4(1.f),
                orig+glm::vec3(OX,0,0)+t),s),c); };

        // ── Premium wall finishes ──────────────────────────────────────────
        // Warm white wall panels
        for(int wp=0;wp<6;wp++){
            float wz=1.f+wp*7.8f;
            SS({ .4f,BH*.5f,wz},{.10f,BH*.9f,7.f},{.96f,.95f,.93f},128);
            SS({49.6f,BH*.5f,wz},{.10f,BH*.9f,7.f},{.96f,.95f,.93f},128);
        }
        // Gold accent trim at floor and ceiling
        SE({ .3f,.12f,BD*.5f},{.06f,.24f,BD*.9f},{.85f,.68f,.20f}); // left wall base
        SE({49.7f,.12f,BD*.5f},{.06f,.24f,BD*.9f},{.85f,.68f,.20f}); // right wall base
        SE({25.f,BH-.15f, .2f},{SW*.9f,.10f,.06f},{.85f,.68f,.20f}); // back wall top
        SE({25.f,BH-.15f,BD-.2f},{SW*.9f,.10f,.06f},{.85f,.68f,.20f}); // front top
        SE({25.f,.10f, .2f},{SW*.9f,.04f,.06f},{.85f,.68f,.20f}); // back wall base
        SE({25.f,.10f,BD-.2f},{SW*.9f,.04f,.06f},{.85f,.68f,.20f}); // front base

        // ── 3 display platforms ────────────────────────────────────────────
        float platX[3]={8.f,24.f,40.f};
        glm::vec3 carCols[3]={{.85f,.05f,.05f},{.05f,.08f,.40f},{.05f,.45f,.20f}};
        for(int di=0;di<3;di++){
            float px=platX[di];
            // Platform base (cream marble slab)
            SS({px,.14f,24.f},{9.f,.28f,11.f},{.92f,.90f,.86f},256);
            // Gold step edge
            SE({px,.28f,20.f},{8.5f,.06f,.10f},{.85f,.68f,.20f});
            SE({px,.28f,28.f},{8.5f,.06f,.10f},{.85f,.68f,.20f});
            SE({px-4.3f,.28f,24.f},{.10f,.06f,10.5f},{.85f,.68f,.20f});
            SE({px+4.3f,.28f,24.f},{.10f,.06f,10.5f},{.85f,.68f,.20f});
            // Floor glow pool around platform
            SE({px,.01f,24.f},{9.5f,.01f,12.f},{.95f,.82f,.50f});
            // Car on platform
            drawAttractiveCar(cube,sh,
                O+glm::vec3(px,.30f,24.f),
                (float)di*35.f, carCols[di], true, false, 0.f);
            // Ceiling spotlight fixture (bracket + housing + globe)
            SS({px, BH-0.08f, 24.f}, {1.3f,0.18f,1.3f}, {0.18f,0.16f,0.12f}, 128); // gold bracket
            SS({px, BH-0.70f, 24.f}, {0.85f,1.1f,0.85f}, {0.14f,0.13f,0.10f}, 128); // dark housing
            sphere.drawEmissive(sh, glm::scale(glm::translate(glm::mat4(1.f),
                O+glm::vec3(px,BH-1.3f,24.f)),glm::vec3(.22f)),{1.f,0.94f,0.72f}); // warm gold globe
            SE({px,BH-0.04f,24.f},{2.5f,0.10f,2.5f},{1.00f,0.90f,0.55f}); // cone base glow
            SE({px,BH-0.04f,24.f},{5.5f,0.06f,5.5f},{0.90f,0.75f,0.35f}); // outer halo
            // Secondary angled accent spots
            SE({px-2.5f,BH-.28f,21.5f},{.55f,.07f,.55f},{1.00f,0.88f,0.50f});
            SE({px+2.5f,BH-.28f,26.5f},{.55f,.07f,.55f},{1.00f,0.88f,0.50f});
        }

        // ── Customer lounge (rear) ─────────────────────────────────────────
        drawSofaChair(cube,sh, O+glm::vec3( 8.f,0.f,42.f), 90.f);
        drawSofaChair(cube,sh, O+glm::vec3(12.f,0.f,42.f), 90.f);
        drawSofaChair(cube,sh, O+glm::vec3(16.f,0.f,42.f), 90.f);
        drawCoffeeTable(cube,sh, O+glm::vec3(12.f,0.f,39.5f));
        drawTVScreen(cube,sh, O+glm::vec3(25.f,1.5f,46.5f), 0.f, 3.0f, 1.8f);
        // TV glow
        SE({25.f,1.5f,46.4f},{3.0f,1.8f,.05f},{.20f,.45f,.90f});

        // ── Sales desk (premium white) ─────────────────────────────────────
        SS({40.f,.95f,42.f},{9.f,1.9f,2.2f},{.96f,.95f,.93f},256);
        SS({40.f,1.96f,42.f},{9.2f,.12f,2.4f},{.88f,.82f,.65f},256); // gold counter top
        SS({36.f,.95f,43.f},{.18f,1.9f,2.f},{.92f,.92f,.94f},64);
        // Desk logo strip
        SE({40.f,1.2f,43.2f},{6.f,.60f,.04f},{.85f,.68f,.20f});
        // Reception screens
        for(int ri=0;ri<3;ri++){
            float rx=37.f+ri*3.f;
            SS({rx,2.1f,41.2f},{.38f,.28f,.04f},{.05f,.05f,.06f},128);
            cube.drawEmissive(sh,
                glm::scale(glm::translate(glm::mat4(1.f),O+glm::vec3(rx,2.1f,41.17f)),
                    glm::vec3(.34f,.24f,.01f)),{.1f,.55f,.90f});
        }
        // Sales worker
        drawWorker(cube,sh, O+glm::vec3(38.f,0.f,40.f),180.f,{.10f,.15f,.42f},0.f,false);

        // ── SHOWROOM neon sign on back wall ────────────────────────────────
        SE({25.f,8.5f,1.f},{SW*.75f,1.8f,.06f},{.85f,.68f,.20f}); // gold SHOWROOM text
        SS({25.f,8.5f,.88f},{SW*.77f,2.0f,.14f},{.06f,.06f,.07f},128); // backing
        SE({25.f,6.8f,1.f},{SW*.60f,.25f,.04f},{.85f,.68f,.20f}); // sub-line strip

        // ── Ambient ceiling warm glow (showroom atmosphere) ────────────────
        SE({25.f,BH-.06f,BD*.5f},{SW*.92f,.02f,BD*.85f},{.20f,.15f,.08f});
        // Perimeter cove lighting
        SE({25.f,BH-.4f, .3f},{SW*.88f,.08f,.08f},{.95f,.80f,.45f});
        SE({25.f,BH-.4f,BD-.3f},{SW*.88f,.08f,.08f},{.95f,.80f,.45f});
        SE({ .3f,BH-.4f,BD*.5f},{.08f,.08f,BD*.8f},{.95f,.80f,.45f});
        SE({49.7f,BH-.4f,BD*.5f},{.08f,.08f,BD*.8f},{.95f,.80f,.45f});
    }

    // ── FIRE EXTINGUISHERS at partition walls ─────────────────────────────────
    for(int si=1;si<4;si++){
        drawFireExtinguisher(cube,sh, orig+glm::vec3(SW*si+1.f, 0.f, 5.f));
        drawFireExtinguisher(cube,sh, orig+glm::vec3(SW*si+1.f, 0.f, 43.f));
    }
    drawFireExtinguisher(cube,sh, orig+glm::vec3(2.f,0.f,5.f));
    drawFireExtinguisher(cube,sh, orig+glm::vec3(2.f,0.f,43.f));

    // ── RECEPTION (front centre strip) ───────────────────────────────────────
    S({BW/2,.9f,BD-2.f},{14.f,.12f,2.4f},{.95f,.95f,.97f},64);
    S({BW/2,.5f,BD-2.f},{14.f,.8f,.18f}, {.92f,.92f,.94f},32);
    S({BW/2-6.f,.5f,BD-1.2f},{.18f,.8f,2.f},{.92f,.92f,.94f},32);
    // Reception screens
    for(int ri=0;ri<3;ri++){
        float rx=BW/2-4.f+ri*4.f;
        S({rx,1.06f,BD-1.f},{.42f,.28f,.04f},{.05f,.05f,.06f},128);
        cube.drawEmissive(sh,
            glm::scale(glm::translate(glm::mat4(1.f),orig+glm::vec3(rx,1.06f,BD-.97f)),
                glm::vec3(.38f,.24f,.01f)),{.1f,.55f,.90f});
    }
    drawWorker(cube,sh, orig+glm::vec3(BW/2-3.f,0.f,BD-3.5f),180.f,
        {.22f,.22f,.55f},0.f,false);
    // Glowing company sign above entrance
    E({BW/2,BH+.35f,BD+.06f},{24.f,.90f,.05f},{.0f,.6f,1.f});
    S({BW/2,BH+.35f,BD+.04f},{24.2f,1.f,.08f},{.05f,.05f,.06f},128);
}

// ============================================================
//  drawFractalBranchTree – recursive fractal tree
//  Branches are properly angled using axis-angle rotation;
//  leaf nodes carry 4 overlapping sphere clusters.
// ============================================================
// recursive impl – forward declare so wrapper can appear first
static void _fracBranch(Cube& cube, Sphere& sphere, Shader& sh,
    glm::vec3 base, glm::vec3 dir, float H, float W, int depth);

void drawFractalBranchTree(Cube& cube, Sphere& sphere, Shader& sh,
    glm::vec3 base)
{
    _fracBranch(cube, sphere, sh, base, {0.f,1.f,0.f}, 4.5f, 0.48f, 4);
}

static void _fracBranch(Cube& cube, Sphere& sphere, Shader& sh,
    glm::vec3 base, glm::vec3 dir, float H, float W, int depth)
{
    if (depth <= 0 || H < 0.08f) return;
    const float PI = 3.14159265f;

    // Bark darkens toward tips
    float dk = 0.55f + depth * 0.10f;
    glm::vec3 bk(0.34f*dk, 0.18f*dk, 0.07f*dk);

    glm::vec3 nd  = glm::normalize(dir);
    glm::vec3 mid = base + nd * (H * 0.5f);
    glm::vec3 top = base + nd * H;

    // Rotate cube so its Y-axis aligns with the branch direction
    glm::vec3 yup  = {0.f, 1.f, 0.f};
    glm::mat4 R    = glm::mat4(1.f);
    glm::vec3 axis = glm::cross(yup, nd);
    if (glm::length(axis) > 0.001f) {
        float ang = acosf(glm::clamp(glm::dot(yup, nd), -1.f, 1.f));
        R = glm::rotate(glm::mat4(1.f), ang, glm::normalize(axis));
    }
    solidCube(cube, sh,
        glm::translate(glm::mat4(1.f), mid) * R *
        glm::scale(glm::mat4(1.f), {W, H, W}),
        bk, 16.f);

    if (depth == 1) {
        // 4 overlapping leaf-sphere clusters in varying greens
        float ls = W * 4.2f;
        const glm::vec3 G[] = {
            {0.10f,0.42f,0.08f}, {0.07f,0.32f,0.06f},
            {0.14f,0.56f,0.10f}, {0.20f,0.50f,0.08f}
        };
        sphere.drawColor(sh,
            glm::scale(glm::translate(glm::mat4(1.f),
                top + glm::vec3(0.f, ls*0.40f, 0.f)), glm::vec3(ls)),       G[0], 8.f);
        sphere.drawColor(sh,
            glm::scale(glm::translate(glm::mat4(1.f),
                top + glm::vec3(W*1.5f, ls*0.12f, 0.f)), glm::vec3(ls*0.72f)), G[1], 8.f);
        sphere.drawColor(sh,
            glm::scale(glm::translate(glm::mat4(1.f),
                top + glm::vec3(-W, ls*0.22f, W*1.3f)), glm::vec3(ls*0.68f)), G[2], 8.f);
        sphere.drawColor(sh,
            glm::scale(glm::translate(glm::mat4(1.f),
                top + glm::vec3(0.f, -ls*0.08f, -W*1.6f)), glm::vec3(ls*0.65f)), G[3], 8.f);
    } else {
        // 3 sub-branches at higher depth, 2 near leaves
        int  N        = (depth >= 3) ? 3 : 2;
        float nextH   = H * 0.66f;
        float nextW   = W * 0.60f;
        float tiltRad = glm::radians(20.f + (4 - depth) * 5.f);

        // Two vectors perpendicular to nd for spreading branches
        glm::vec3 ref   = (fabsf(nd.y) < 0.9f) ? glm::vec3(0,1,0) : glm::vec3(1,0,0);
        glm::vec3 p1    = glm::normalize(glm::cross(nd, ref));
        glm::vec3 p2    = glm::normalize(glm::cross(nd, p1));

        for (int i = 0; i < N; ++i) {
            float yaw = i * (2.f * PI / N) + depth * 0.55f;
            glm::vec3 lat = p1 * cosf(yaw) + p2 * sinf(yaw);
            glm::vec3 sub = glm::normalize(nd * cosf(tiltRad) + lat * sinf(tiltRad));
            _fracBranch(cube, sphere, sh, top, sub, nextH, nextW, depth - 1);
        }
    }
}

// ============================================================
//  drawRealisticTree – sphere-canopy trees (6 distinct styles)
//  type: 1=tall pine  2=spreading oak  3=palm  4=medium oak
//        5=wide shrub  6=young sapling
// ============================================================
void drawRealisticTree(Cube& cube, Sphere& sphere, Shader& sh,
    glm::vec3 base, int type)
{
    const float PI = 3.14159265f;

    // Solid cube helper: p = world centre, sz = full extents
    auto SC = [&](glm::vec3 p, glm::vec3 sz, glm::vec3 c) {
        solidCube(cube, sh,
            glm::scale(glm::translate(glm::mat4(1.f), p), sz), c, 16.f); };

    // Sphere helper: p = centre, r = radius, c = colour
    auto SP = [&](glm::vec3 p, float r, glm::vec3 c) {
        sphere.drawColor(sh,
            glm::scale(glm::translate(glm::mat4(1.f), p), glm::vec3(r)), c, 8.f); };

    // Palette
    const glm::vec3 bk (0.36f,0.19f,0.08f);  // dark bark
    const glm::vec3 bkL(0.46f,0.25f,0.10f);  // lighter bark
    const glm::vec3 g1 (0.10f,0.42f,0.08f);  // deep green
    const glm::vec3 g2 (0.14f,0.56f,0.10f);  // bright green
    const glm::vec3 g3 (0.07f,0.32f,0.06f);  // dark green
    const glm::vec3 g4 (0.20f,0.50f,0.08f);  // yellow-green
    const glm::vec3 g5 (0.06f,0.38f,0.14f);  // teal-green
    const glm::vec3 gc[5] = { g1,g3,g2,g4,g5 };

    if (type == 1) {
        // ── TALL PINE: tapered trunk + 5-tier conical canopy ──────────────────
        SC(base + glm::vec3(0, 3.25f, 0), {0.50f, 6.5f, 0.50f}, bk);

        struct TierDef { float h, ringR, sr; int n; } tiers[] = {
            { 3.5f, 2.2f, 1.20f, 6 },
            { 5.0f, 1.7f, 1.00f, 5 },
            { 6.2f, 1.2f, 0.85f, 4 },
            { 7.2f, 0.7f, 0.65f, 3 },
        };
        for (int l = 0; l < 4; ++l) {
            auto& T = tiers[l];
            for (int i = 0; i < T.n; ++i) {
                float a = i * (2.f * PI / T.n) + l * 0.42f;
                SP(base + glm::vec3(cosf(a)*T.ringR, T.h, sinf(a)*T.ringR),
                   T.sr, gc[(l + i) % 5]);
            }
            SP(base + glm::vec3(0, T.h + 0.3f, 0), T.sr * 0.88f, g3);  // centre fill
        }
        SP(base + glm::vec3(0, 8.1f, 0), 0.55f, g1);  // pointed tip
    }

    else if (type == 2) {
        // ── SPREADING OAK: short trunk + 4 branches + wide dome canopy ────────
        SC(base + glm::vec3(0, 1.0f, 0), {0.65f, 2.0f, 0.65f}, bk);
        SC(base + glm::vec3(0, 2.8f, 0), {0.50f, 1.6f, 0.50f}, bk);
        for (int i = 0; i < 4; ++i) {
            float a = i * (PI / 2.f) + 0.35f;
            float bx = cosf(a) * 0.85f, bz = sinf(a) * 0.85f;
            SC(base + glm::vec3(bx, 4.0f, bz), {0.20f, 2.0f, 0.20f}, bkL);
            SP(base + glm::vec3(cosf(a)*2.0f, 5.2f, sinf(a)*2.0f), 1.30f, i%2==0?g1:g2);
            SP(base + glm::vec3(cosf(a)*1.3f, 4.7f, sinf(a)*1.3f), 1.15f, g3);
        }
        SP(base + glm::vec3( 0.0f, 5.8f,  0.0f), 1.50f, g1);
        SP(base + glm::vec3( 0.6f, 5.3f,  0.6f), 1.20f, g2);
        SP(base + glm::vec3(-0.6f, 5.1f, -0.4f), 1.15f, g3);
        SP(base + glm::vec3( 0.3f, 5.5f, -0.7f), 1.10f, g4);
    }

    else if (type == 3) {
        // ── PALM: slender tapered trunk + 8 drooping frond clusters ───────────
        SC(base + glm::vec3(0.0f, 1.75f, 0.0f), {0.28f, 3.50f, 0.28f}, bk);
        SC(base + glm::vec3(0.2f, 4.75f, 0.1f), {0.22f, 2.50f, 0.22f}, bkL);
        glm::vec3 tip = base + glm::vec3(0.5f, 7.2f, 0.2f);
        SP(tip, 0.55f, bk);  // crown node
        for (int i = 0; i < 8; ++i) {
            float a = i * (PI / 4.f);
            float fx = cosf(a), fz = sinf(a);
            // 3 spheres per frond, tapering and drooping outward
            SP(tip + glm::vec3(fx*0.8f, -0.40f, fz*0.8f), 0.62f, i%2==0?g1:g4);
            SP(tip + glm::vec3(fx*1.5f, -0.80f, fz*1.5f), 0.52f, i%3==0?g2:g1);
            SP(tip + glm::vec3(fx*2.1f, -1.20f, fz*2.1f), 0.40f, g3);
        }
    }

    else if (type == 4) {
        // ── MEDIUM OAK: medium trunk + 3 angled branches + dense round crown ──
        SC(base + glm::vec3(0, 1.25f, 0), {0.55f, 2.5f, 0.55f}, bk);
        SC(base + glm::vec3(0, 3.25f, 0), {0.40f, 1.5f, 0.40f}, bk);
        for (int i = 0; i < 3; ++i) {
            float a = i * (2.f * PI / 3.f) + 0.5f;
            float bx = cosf(a) * 0.8f, bz = sinf(a) * 0.8f;
            SC(base + glm::vec3(bx, 4.5f, bz), {0.22f, 1.8f, 0.22f}, bkL);
            SP(base + glm::vec3(cosf(a)*1.8f, 5.6f, sinf(a)*1.8f), 1.35f, gc[i]);
            SP(base + glm::vec3(cosf(a)*1.1f, 5.0f, sinf(a)*1.1f), 1.20f, g3);
        }
        SP(base + glm::vec3( 0.0f, 6.2f,  0.0f), 1.55f, g1);
        SP(base + glm::vec3( 0.7f, 5.6f,  0.5f), 1.20f, g2);
        SP(base + glm::vec3(-0.5f, 5.5f, -0.6f), 1.15f, g5);
        SP(base + glm::vec3( 0.3f, 5.8f, -0.7f), 1.10f, g4);
    }

    else if (type == 5) {
        // ── WIDE SHRUB: stumpy trunk + layered dome of sphere clusters ─────────
        SC(base + glm::vec3(0, 0.45f, 0), {0.42f, 0.9f, 0.42f}, bk);
        for (int i = 0; i < 6; ++i) {   // outer ring
            float a = i * (PI / 3.f);
            SP(base + glm::vec3(cosf(a)*2.0f, 1.0f, sinf(a)*2.0f),
               1.05f, gc[i % 5]);
        }
        for (int i = 0; i < 4; ++i) {   // inner ring, slightly higher
            float a = i * (PI / 2.f) + 0.4f;
            SP(base + glm::vec3(cosf(a)*1.1f, 1.6f, sinf(a)*1.1f),
               1.10f, i%2==0?g1:g3);
        }
        SP(base + glm::vec3( 0.0f, 2.4f,  0.0f), 1.20f, g3);
        SP(base + glm::vec3( 0.5f, 1.9f,  0.5f), 0.95f, g5);
    }

    else if (type == 6) {
        // ── YOUNG SAPLING: slim trunk + small 4-sphere top cluster ────────────
        SC(base + glm::vec3(0, 1.25f, 0), {0.18f, 2.5f, 0.18f}, bk);
        SC(base + glm::vec3(0, 3.25f, 0), {0.14f, 1.5f, 0.14f}, bkL);
        SP(base + glm::vec3( 0.0f, 4.5f,  0.0f), 1.00f, g1);
        SP(base + glm::vec3( 0.6f, 4.1f,  0.4f), 0.85f, g2);
        SP(base + glm::vec3(-0.5f, 4.2f, -0.4f), 0.80f, g3);
        SP(base + glm::vec3( 0.2f, 4.7f, -0.5f), 0.75f, g4);
    }
}

// ============================================================
//  drawGasStation – petrol pump with canopy, dispensers, kiosk
// ============================================================
void drawGasStation(Cube& cube, Sphere& sphere, Shader& sh, glm::vec3 pos)
{
    auto SC = [&](glm::vec3 p, glm::vec3 sz, glm::vec3 c, float sh2 = 16) {
        solidCube(cube, sh,
            glm::scale(glm::translate(glm::mat4(1.f), p), sz), c, sh2); };
    auto EC = [&](glm::vec3 p, glm::vec3 sz, glm::vec3 c) {
        cube.drawEmissive(sh,
            glm::scale(glm::translate(glm::mat4(1.f), p), sz), c); };

    // Concrete forecourt pad
    SC(pos + glm::vec3(0.f, -0.05f, 0.f),   {26.f, 0.10f, 20.f}, {0.56f,0.56f,0.57f}, 8);

    // 4 canopy support posts (corners)
    for (float px : {-8.f, 8.f})
        for (float pz : {-5.f, 5.f})
            SC(pos + glm::vec3(px, 3.0f, pz), {0.30f,6.0f,0.30f}, {0.25f,0.25f,0.28f}, 64);

    // Canopy roof slab
    SC(pos + glm::vec3(0.f, 6.15f, 0.f), {20.f,0.45f,12.f}, {0.88f,0.88f,0.90f}, 8);
    // Canopy underside emissive (lamp strip)
    EC(pos + glm::vec3(0.f, 5.92f, 0.f), {18.f,0.06f,10.f}, {0.98f,0.96f,0.82f});

    // Price-sign pole + board
    SC(pos + glm::vec3(11.f, 3.f, -8.f),    {0.22f,6.0f,0.22f}, {0.22f,0.22f,0.24f}, 64);
    EC(pos + glm::vec3(11.f, 6.6f, -8.f),   {2.6f,1.4f,0.20f},  {0.88f,0.10f,0.05f}); // red board
    EC(pos + glm::vec3(11.f, 6.6f, -8.f),   {1.9f,0.75f,0.22f}, {1.00f,0.95f,0.70f}); // price digits

    // Kiosk shop
    SC(pos + glm::vec3(-9.5f, 2.0f, -6.5f), {7.f,4.0f,5.5f}, {0.88f,0.84f,0.80f}, 16);
    SC(pos + glm::vec3(-9.5f, 4.3f, -6.5f), {7.2f,0.32f,5.7f}, {0.72f,0.70f,0.66f}, 8); // flat roof
    EC(pos + glm::vec3(-8.5f, 2.1f, -3.8f), {2.6f,1.8f,0.04f}, {0.78f,0.90f,0.96f}); // window
    EC(pos + glm::vec3(-9.5f, 1.2f, -3.8f), {1.1f,2.3f,0.04f}, {0.68f,0.82f,0.90f}); // door

    // 2 fuel dispensers
    for (int di = 0; di < 2; ++di) {
        glm::vec3 dp = pos + glm::vec3(-2.5f + di * 5.f, 0.f, 1.5f);
        SC(dp + glm::vec3(0, 0.85f, 0),  {0.72f,1.70f,0.36f}, {0.90f,0.28f,0.10f}, 32); // body
        SC(dp + glm::vec3(0, 1.78f, 0),  {0.74f,0.20f,0.38f}, {0.82f,0.82f,0.84f}, 16); // top cap
        EC(dp + glm::vec3(0, 1.22f,-0.22f),{0.52f,0.32f,0.04f},{0.20f,0.82f,1.00f}); // screen
        SC(dp + glm::vec3(0.40f,0.90f,0),{0.06f,0.06f,0.50f}, {0.12f,0.12f,0.12f}, 8); // hose
    }

    // ── Decorative stone bollard spheres at forecourt entrance ───────────────
    // Two polished granite balls on short square pedestals, flanking the entrance
    for (float sx : { -5.f, 5.f }) {
        glm::vec3 ped = pos + glm::vec3(sx, 0.f, -9.5f);
        SC(ped + glm::vec3(0, 0.28f, 0), {0.72f, 0.56f, 0.72f}, {0.55f,0.54f,0.52f}, 32);  // pedestal
        sphere.drawColor(sh,
            glm::scale(glm::translate(glm::mat4(1.f), ped + glm::vec3(0, 0.88f, 0)),
                       glm::vec3(0.48f)),
            {0.72f, 0.70f, 0.68f}, 96.f);   // polished stone ball
    }

    // ── Decorative illuminated globe on canopy ridge ──────────────────────────
    // Warm amber glowing sphere mounted at the centre peak of the canopy
    sphere.drawEmissive(sh,
        glm::scale(glm::translate(glm::mat4(1.f), pos + glm::vec3(0.f, 6.7f, 0.f)),
                   glm::vec3(0.55f)),
        {1.0f, 0.82f, 0.30f});   // amber glow

    // ── FUEL ZONE – red rectangle on forecourt (world FZ_X1..FZ_X2, FZ_Z1..FZ_Z2) ──
    {
        const float Y  = 0.12f;   // raised above all ground/tile surfaces
        const float TH = 0.06f;
        const float LW = 0.35f;
        // world coords: x1=-44, x2=-32, z1=17, z2=27 → pos=(-38,0,22), so local:
        const float x1 = pos.x - 6.f, x2 = pos.x + 6.f;
        const float z1 = pos.z - 5.f, z2 = pos.z + 5.f;
        const float cx = (x1+x2)*.5f, cz = (z1+z2)*.5f;
        EC({x1, Y, cz}, {LW, TH, z2-z1}, {1.f,0.f,0.f}); // left
        EC({x2, Y, cz}, {LW, TH, z2-z1}, {1.f,0.f,0.f}); // right
        EC({cx, Y, z1}, {x2-x1, TH, LW}, {1.f,0.f,0.f}); // front
        EC({cx, Y, z2}, {x2-x1, TH, LW}, {1.f,0.f,0.f}); // back
    }
}

// ============================================================
//  drawCityBlock  –  City environment (≤10 buildings, 6 trees)
//   • Concrete city ground
//   • Single main road with working street lights
//   • Gas station west of workstation
//   • 10 buildings on all sides
//   • 6 fractal trees with distinct designs
//   • 5 city pedestrians
// ============================================================
void drawCityBlock(Cube& cube, Sphere& sphere, Shader& sh,
    glm::vec3 orig, float BW, float BD,
    float ROAD_Z)
{
    auto SC = [&](glm::vec3 p, glm::vec3 sz, glm::vec3 c, float sh2 = 16) {
        solidCube(cube, sh,
            glm::scale(glm::translate(glm::mat4(1.f), p), sz), c, sh2); };
    auto EC = [&](glm::vec3 p, glm::vec3 sz, glm::vec3 c) {
        cube.drawEmissive(sh,
            glm::scale(glm::translate(glm::mat4(1.f), p), sz), c); };
    // Textured ground tile helper
    auto TC = [&](glm::vec3 p, glm::vec3 sz, glm::vec3 c, float sh2, unsigned int tex) {
        drawCube(cube, sh,
            glm::scale(glm::translate(glm::mat4(1.f), p), sz),
            c * 0.28f, c, c * 0.22f, sh2, tex);
    };
    // Tile a region with grass patches (40×40 each)
    auto grassField = [&](float x0, float z0, float x1, float z1) {
        const float T = 40.f;
        for (float gx = x0; gx < x1; gx += T) {
            float tw = (gx + T > x1) ? (x1 - gx) : T;
            for (float gz = z0; gz < z1; gz += T) {
                float td = (gz + T > z1) ? (z1 - gz) : T;
                TC({gx+tw*.5f, 0.03f, gz+td*.5f}, {tw, 0.06f, td},
                   {0.32f,0.64f,0.24f}, 4, texGrass);
            }
        }
    };
    // Tile a region with concrete patches (50×50 each)
    auto concField = [&](float x0, float z0, float x1, float z1,
                         float yc, glm::vec3 col) {
        const float T = 50.f;
        for (float cx = x0; cx < x1; cx += T) {
            float tw = (cx + T > x1) ? (x1 - cx) : T;
            for (float cz = z0; cz < z1; cz += T) {
                float td = (cz + T > z1) ? (z1 - cz) : T;
                TC({cx+tw*.5f, yc, cz+td*.5f}, {tw, 0.06f, td},
                   col, 8, texConcrete);
            }
        }
    };

    const float RZ0  = ROAD_Z;     // main front road z = 65
    const float RZB  = -22.f;      // back road z (behind building)
    const float RXW  = -45.f;      // west N-S cross road x
    const float RXE  = 245.f;      // east N-S cross road x
    const float RW   = 14.f;       // road width
    const float CRW  = 12.f;       // cross road width
    const glm::vec3 kerbC  {0.80f,0.80f,0.80f};
    const glm::vec3 markW  {0.94f,0.94f,0.94f};

    // ── 1. GROUND – compact city terrain ──────────────────────────────────────
    // Thin base slab (prevents see-through; largely hidden under tiles)
    SC({100.f,-0.30f,15.f},{500.f,0.44f,300.f},{0.33f,0.32f,0.31f},4);

    // ── Grass: fill all open city areas ──────────────────────────────────────
    // South park (beyond front sidewalk → city buildings south of road)
    grassField(-60.f, RZ0+7.5f,   310.f, 140.f);
    // North area (behind building, beyond back road)
    grassField(-90.f, -105.f,     310.f, RZB-6.5f);
    // West zone (left of west road)
    grassField(-115.f, -85.f,     RXW-6.5f, 90.f);
    // East zone (right of east road)
    grassField(RXE+6.5f, -85.f,   320.f, 90.f);
    // Left strip between west road and building – split to avoid gas station (X:-38,Z:12-32)
    grassField(RXW+6.5f, RZB+6.5f, -50.f, RZ0-7.5f); // far left (beyond gas station)
    grassField(-30.f,    RZB+6.5f,   0.f, RZ0-7.5f);  // close to building, east of gas station
    // Right strip between building and east road
    grassField(200.f,  RZB+6.5f,  RXE-6.5f, RZ0-7.5f);
    // Behind building (between back road and building back wall)
    grassField(0.f,  RZB+6.5f,    200.f,  0.f);
    // Extra grass median between building front and parking
    grassField(0.f,  BD+1.f,      200.f,  BD+3.5f);

    // ── Concrete apron around building – 4 strips, skipping building interior ──
    // (building interior is X:0→200, Z:0→48 — kept clear so zone markers are visible)
    concField(-35.f, -18.f,   0.f,  62.f, 0.04f, {0.78f,0.77f,0.76f}); // left
    concField(200.f, -18.f, 235.f,  62.f, 0.04f, {0.78f,0.77f,0.76f}); // right
    concField(  0.f, -18.f, 200.f,   0.f, 0.04f, {0.78f,0.77f,0.76f}); // behind bldg
    concField(  0.f,  48.f, 200.f,  62.f, 0.04f, {0.78f,0.77f,0.76f}); // in front
    // Front plaza between building entrance and parking
    concField(-15.f, BD+3.5f, 215.f, BD+14.f, 0.05f, {0.84f,0.83f,0.82f});
    // Concrete pads in front of city buildings (south of road)
    concField(  5.f, RZ0+8.f,  40.f, RZ0+28.f, 0.04f, {0.76f,0.75f,0.74f}); // bldg 1
    concField( 68.f, RZ0+8.f, 100.f, RZ0+28.f, 0.04f, {0.76f,0.75f,0.74f}); // bldg 2
    concField(142.f, RZ0+8.f, 180.f, RZ0+28.f, 0.04f, {0.76f,0.75f,0.74f}); // bldg 3
    // Note: gas station has its own SC forecourt – no extra concField tile needed there

    // ── 2. ROAD NETWORK (textured asphalt) ───────────────────────────────────
    // Front E-W road – tiled in 25-unit sections for texture repeat
    for (int ri = 0; ri < 28; ri++) {
        float rx = -340.f + ri*25.f + 12.5f;
        TC({rx, 0.06f, RZ0}, {25.f, 0.08f, RW}, {0.24f,0.24f,0.26f}, 8, texRoad);
    }
    // Back E-W road
    for (int ri = 0; ri < 23; ri++) {
        float rx = -100.f + ri*25.f + 12.5f;
        TC({rx, 0.06f, RZB}, {25.f, 0.08f, CRW}, {0.24f,0.24f,0.26f}, 8, texRoad);
    }
    // West N-S road – tiled in 25-unit sections
    for (int rj = 0; rj < 11; rj++) {
        float rz = -105.f + rj*25.f + 12.5f;
        TC({RXW, 0.06f, rz}, {CRW, 0.08f, 25.f}, {0.24f,0.24f,0.26f}, 8, texRoad);
    }
    // East N-S road
    for (int rj = 0; rj < 11; rj++) {
        float rz = -105.f + rj*25.f + 12.5f;
        TC({RXE, 0.06f, rz}, {CRW, 0.08f, 25.f}, {0.24f,0.24f,0.26f}, 8, texRoad);
    }
    // Intersection fills
    for (float iz : {RZ0, RZB}) {
        TC({RXW,0.07f,iz},{CRW+2.f,0.05f,RW+2.f},{0.24f,0.24f,0.26f},8,texRoad);
        TC({RXE,0.07f,iz},{CRW+2.f,0.05f,RW+2.f},{0.24f,0.24f,0.26f},8,texRoad);
    }

    // Kerbs on all roads
    for (float s : {-1.f,1.f}) {
        SC({100.f,0.11f,RZ0+s*7.3f},{700.f,0.10f,0.65f},kerbC,32);
        SC({100.f,0.11f,RZB+s*6.3f},{560.f,0.10f,0.65f},kerbC,32);
        SC({RXW+s*6.3f,0.11f,24.f},{0.65f,0.10f,260.f},kerbC,32);
        SC({RXE+s*6.3f,0.11f,24.f},{0.65f,0.10f,260.f},kerbC,32);
    }

    // Lane markings – front road
    for (int i = -24; i <= 35; i++) {
        float lx = i*7.f+3.5f;
        SC({lx,0.11f,RZ0},     {4.f,0.02f,0.18f},markW,4);
        SC({lx,0.11f,RZ0+3.8f},{4.f,0.02f,0.20f},{0.88f,0.88f,0.88f},4);
        SC({lx,0.11f,RZ0-3.8f},{4.f,0.02f,0.20f},{0.88f,0.88f,0.88f},4);
    }
    // Lane markings – back road
    for (int i = -10; i <= 25; i++) {
        float lx = i*7.f+3.5f;
        SC({lx,0.11f,RZB},{4.f,0.02f,0.18f},markW,4);
    }
    // Lane markings – cross roads (N-S)
    for (int j = -4; j <= 12; j++) {
        float lz = j*7.f+3.5f;
        SC({RXW,0.11f,lz},{0.18f,0.02f,4.f},markW,4);
        SC({RXE,0.11f,lz},{0.18f,0.02f,4.f},markW,4);
    }
    // Crosswalk in front of building
    for (int ci = 0; ci < 9; ci++)
        SC({orig.x+BW/2.f-4.f+(float)ci,0.12f,RZ0-7.2f},{0.58f,0.02f,4.4f},{0.92f,0.92f,0.94f},4);

    // ── 3. SIDEWALKS (textured) & PARKING ────────────────────────────────────
    TC({orig.x+BW/2.f,0.05f,BD+9.f},{BW+30.f,0.08f,24.f},{0.72f,0.72f,0.74f},16,texSidewalk);
    TC({100.f,0.05f,RZ0-10.f},{500.f,0.06f,5.f},{0.70f,0.70f,0.72f},16,texSidewalk);
    TC({100.f,0.05f,RZ0+10.f},{500.f,0.06f,5.f},{0.70f,0.70f,0.72f},16,texSidewalk);
    TC({100.f,0.05f,RZB-9.f},{460.f,0.06f,4.f},{0.70f,0.70f,0.72f},16,texSidewalk);
    for (float s:{-1.f,1.f}) {
        TC({RXW+s*9.5f,0.05f,24.f},{4.f,0.06f,200.f},{0.70f,0.70f,0.72f},16,texSidewalk);
        TC({RXE+s*9.5f,0.05f,24.f},{4.f,0.06f,200.f},{0.70f,0.70f,0.72f},16,texSidewalk);
    }
    // Parking lot (asphalt texture)
    {
        float pyZ = BD+6.f;
        TC({orig.x+BW/2.f,0.06f,pyZ},{BW+4.f,0.04f,10.f},{0.52f,0.52f,0.54f},16,texAsphalt);
        for (int pi=0;pi<7;pi++) SC({orig.x+4.f+pi*9.f,0.10f,pyZ},{0.12f,0.02f,9.f},{0.88f,0.76f,0.05f},4);
        SC({orig.x+BW/2.f,0.10f,pyZ-4.5f},{BW*.92f,0.02f,0.14f},{0.88f,0.76f,0.05f},4);
        SC({orig.x+BW/2.f,0.10f,pyZ+4.5f},{BW*.92f,0.02f,0.14f},{0.88f,0.76f,0.05f},4);
    }

    // ── 4. STREET LIGHTS — every post globe sits at its exact point-light pos ─
    // Helper: pole on sidewalk, arm over road, globe exactly at (ptX, 7, ptZ)
    auto lampPostFront = [&](float ptX) {
        // Pole on south side of front road
        float poleZ = RZ0 + 8.5f;
        SC({ptX,3.5f,poleZ},{0.18f,7.f,0.18f},{0.22f,0.22f,0.24f},64);
        SC({ptX,7.1f,(RZ0+poleZ)*.5f},{0.12f,0.12f,poleZ-RZ0+0.4f},{0.22f,0.22f,0.24f},64);
        sphere.drawEmissive(sh,
            glm::scale(glm::translate(glm::mat4(1.f),{ptX,7.f,RZ0}),{0.42f,0.42f,0.42f}),
            {1.f,0.92f,0.60f});
    };
    auto lampPostBack = [&](float ptX) {
        // Pole on south side of back road
        float poleZ = RZB + 7.5f;
        SC({ptX,3.5f,poleZ},{0.18f,7.f,0.18f},{0.22f,0.22f,0.24f},64);
        SC({ptX,7.1f,(RZB+poleZ)*.5f},{0.12f,0.12f,poleZ-RZB+0.4f},{0.22f,0.22f,0.24f},64);
        sphere.drawEmissive(sh,
            glm::scale(glm::translate(glm::mat4(1.f),{ptX,7.f,RZB}),{0.42f,0.42f,0.42f}),
            {1.f,0.92f,0.60f});
    };
    auto lampPostNS = [&](float ptZ, float ptX, float sideSign) {
        // Pole on east/west side of N-S road
        float poleX = ptX + sideSign * 8.5f;
        SC({poleX,3.5f,ptZ},{0.18f,7.f,0.18f},{0.22f,0.22f,0.24f},64);
        SC({(ptX+poleX)*.5f,7.1f,ptZ},{poleX-ptX+0.4f,0.12f,0.12f},{0.22f,0.22f,0.24f},64);
        sphere.drawEmissive(sh,
            glm::scale(glm::translate(glm::mat4(1.f),{ptX,7.f,ptZ}),{0.42f,0.42f,0.42f}),
            {1.f,0.92f,0.60f});
    };

    // Front road — lights[4,5,6] at (-28,7,65), (80,7,65), (170,7,65)
    lampPostFront(-28.f);
    lampPostFront( 80.f);
    lampPostFront(170.f);
    // Back road — lights[7,8] at (50,7,-22), (150,7,-22)
    lampPostBack(50.f);
    lampPostBack(150.f);
    // West cross road — lights[9] at (-45,7,24)
    lampPostNS(24.f, RXW, -1.f);
    // East cross road — lights[10] at (245,7,24)
    lampPostNS(24.f, RXE, +1.f);

    // ── 5. GAS STATION ────────────────────────────────────────────────────────
    drawGasStation(cube, sphere, sh, glm::vec3(-38.f, 0.f, 22.f));

    // ── 6. CITY BUILDINGS — 10 total ─────────────────────────────────────────
    auto cityBldg = [&](float x, float z, float w, float d, float h, glm::vec3 col) {
        SC({x,h/2.f,z},{w,h,d},col,32);
        SC({x,h+0.22f,z},{w+0.18f,0.40f,d+0.18f},col*0.68f,16);
        SC({x,h+0.46f,z},{w*0.50f,0.26f,d*0.50f},col*0.56f,16);
        int fl=(int)(h/2.8f);
        for(int f=1;f<fl;f++){
            float wy=f*2.8f;
            EC({x,wy,z+d*.5f+.04f},{w*.78f,.60f,.04f},{.72f,.88f,1.f});
            EC({x,wy,z-d*.5f-.04f},{w*.78f,.60f,.04f},{.72f,.88f,1.f});
            EC({x+w*.5f+.04f,wy,z},{.04f,.60f,d*.68f},{.72f,.88f,1.f});
        }
    };
    cityBldg( 20.f, RZ0+25.f,16.f,11.f,16.f,{0.75f,0.52f,0.40f}); // 1 south-orange
    cityBldg( 82.f, RZ0+23.f,14.f,10.f,26.f,{0.50f,0.58f,0.72f}); // 2 south-blue
    cityBldg(158.f, RZ0+24.f,18.f,12.f,20.f,{0.62f,0.60f,0.55f}); // 3 south-grey
    cityBldg( 55.f, -52.f,   20.f,13.f,22.f,{0.55f,0.60f,0.65f}); // 4 north
    cityBldg(145.f, -48.f,   16.f,11.f,28.f,{0.52f,0.58f,0.68f}); // 5 north-tall
    cityBldg(-82.f,  30.f,   14.f,10.f,18.f,{0.60f,0.62f,0.65f}); // 6 west
    cityBldg(-90.f,  68.f,   16.f,10.f,30.f,{0.48f,0.54f,0.62f}); // 7 west-tall
    cityBldg(268.f,  18.f,   14.f,10.f,20.f,{0.62f,0.65f,0.70f}); // 8 east
    cityBldg(268.f,  58.f,   16.f,11.f,32.f,{0.45f,0.52f,0.62f}); // 9 east-tall
    cityBldg(268.f,  96.f,   12.f, 9.f,16.f,{0.68f,0.62f,0.58f}); // 10 east-low

    // ── 7. 6 TREES — realistic sphere-canopy trees ───────────────────────────
    // T1 – Tall pine: conical 5-tier sphere canopy
    drawRealisticTree(cube, sphere, sh, {-18.f, 0.f, RZ0-11.f}, 1);
    // T2 – Spreading oak: 4 branches + wide dome
    drawRealisticTree(cube, sphere, sh, {162.f, 0.f, RZ0-11.f}, 2);
    // T3 – Palm: tapered trunk + 8 drooping fronds
    drawRealisticTree(cube, sphere, sh, {-28.f, 0.f,     8.f }, 3);
    // T4 – Medium oak: 3 branches + rounded crown
    drawRealisticTree(cube, sphere, sh, { 88.f, 0.f,   -56.f }, 4);
    // T5 – Wide shrub: layered dome, no real trunk
    drawRealisticTree(cube, sphere, sh, {238.f, 0.f,    48.f }, 5);
    // T6 – Young sapling: slim trunk + small cluster
    drawRealisticTree(cube, sphere, sh, {196.f, 0.f, RZ0+13.f}, 6);

    // ── 8. CITY PEDESTRIANS ───────────────────────────────────────────────────
    float sw = sinf((float)glfwGetTime() * 1.2f) * 0.38f;
    drawWorker(cube,sh,{120.f,0.f,RZ0+11.f},-90.f,{0.70f,0.18f,0.12f}, sw, false); // red jacket east
    drawWorker(cube,sh,{ 40.f,0.f,RZ0+11.f}, 90.f,{0.18f,0.18f,0.22f},-sw, false); // dark coat west
    drawWorker(cube,sh,{ 96.f,0.f,BD+5.f},    0.f,{0.35f,0.28f,0.55f}, sw, false); // entrance
    drawWorker(cube,sh,{-24.f,0.f,18.f},       0.f,{0.20f,0.42f,0.65f},-sw, false); // gas station
    drawWorker(cube,sh,{ 98.f,0.f,RZ0-8.f}, 180.f,{0.55f,0.45f,0.28f}, sw, false); // crossing
}

// ============================================================
//  drawWorkshopBuilding  (unchanged from original)
// ============================================================
void drawWorkshopBuilding(Cube& cube, Shader& sh, glm::vec3 orig)
{
    const float BW = 70.0f, BD = 34.0f, BH = 12.0f, RD = 8.0f, flThick = 0.18f;
    const glm::vec3 wallC(0.93f, 0.94f, 0.95f);
    const glm::vec3 darkFlC(0.07f, 0.07f, 0.08f);
    const glm::vec3 recFlC(0.70f, 0.57f, 0.40f);
    const glm::vec3 colC(0.22f, 0.22f, 0.25f);
    const glm::vec3 roofC(0.48f, 0.46f, 0.44f);   // medium warm grey – clearly visible
    const glm::vec3 winC(0.55f, 0.72f, 0.88f);
    const glm::vec3 ceilC(0.88f, 0.88f, 0.90f);
    const glm::vec3 ledC(0.95f, 0.95f, 0.85f);

    auto B = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2 = 16, unsigned int tx = 0) {
        drawCube(cube, sh, glm::scale(glm::translate(glm::mat4(1.0f), orig + t), s), c * 0.3f, c, c * 0.5f, sh2, tx);
        };
    auto S = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2 = 32) {
        B(t, s, c, sh2, texWhite); sh.setBool("useTexture", false);
        };
    auto E = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c) {
        cube.drawEmissive(sh, glm::scale(glm::translate(glm::mat4(1.0f), orig + t), s), c);
        };

    // ── Floors ───────────────────────────────────────────────────────────────────
    // Dark epoxy service floor – solid dark blue-grey (texture stretches at 70m scale)
    S({ BW/2,-flThick/2,(BD-RD)/2 }, { BW,flThick,BD-RD }, darkFlC, 8);
    // Lighter mid-tone grid lines to break up the floor visually
    for (int gi = 0; gi <= 7; ++gi) {
        float gx = gi * (BW / 7.0f);
        S({ gx,-flThick+0.005f,(BD-RD)/2 }, { 0.06f,0.01f,BD-RD }, glm::vec3(0.12f,0.14f,0.18f), 8);
    }
    for (int gj = 0; gj <= 3; ++gj) {
        float gz = gj * ((BD-RD) / 3.0f);
        S({ BW/2,-flThick+0.005f,gz }, { BW,0.01f,0.06f }, glm::vec3(0.12f,0.14f,0.18f), 8);
    }
    // Yellow safety stripes around each bay (thin slabs on top of floor)
    for (int bi = 0; bi < 5; ++bi) {
        float bx = 5.5f + bi * 10.0f;
        E({ bx + 0,  -flThick + 0.01f, 5.0f  }, { 9.0f,0.02f,0.08f }, glm::vec3(0.90f,0.80f,0.05f));
        E({ bx + 0,  -flThick + 0.01f, 17.0f }, { 9.0f,0.02f,0.08f }, glm::vec3(0.90f,0.80f,0.05f));
        E({ bx - 4.5f,-flThick + 0.01f,11.0f }, { 0.08f,0.02f,12.0f }, glm::vec3(0.90f,0.80f,0.05f));
        E({ bx + 4.5f,-flThick + 0.01f,11.0f }, { 0.08f,0.02f,12.0f }, glm::vec3(0.90f,0.80f,0.05f));
    }
    // Wood-look reception floor – warm honey-brown solid colour
    S({ BW/2,-flThick/2,BD-RD/2 }, { BW,flThick,RD }, glm::vec3(0.55f,0.35f,0.16f), 32);
    // Back wall
    B({ BW / 2,BH / 2,0 }, { BW,BH,0.28f }, wallC, 16, texBrick);
    // Left wall with windows
    B({ 0.14f,1.25f,(BD - RD) / 2 }, { 0.28f,2.5f,BD - RD }, wallC, 16, texBrick);
    for (int i = 0; i < 4; ++i) {
        float wz = 3.0f + i * 5.0f;
        B({ 0.14f,5.5f,wz + 1.5f }, { 0.10f,3.5f,2.8f }, winC, 128, texWindow);
        S({ 0.16f,5.5f,wz + 1.5f }, { 0.12f,3.6f,0.08f }, colC);
        S({ 0.16f,5.5f,wz + 0.1f }, { 0.12f,0.08f,2.6f }, colC);
        S({ 0.16f,5.5f,wz + 3.0f }, { 0.12f,0.08f,0.12f }, colC);
        S({ 0.16f,7.3f,wz + 1.5f }, { 0.12f,0.08f,2.6f }, colC);
        S({ 0.16f,3.8f,wz + 1.5f }, { 0.12f,0.08f,2.6f }, colC);
    }
    B({ 0.14f,BH - 0.75f,(BD - RD) / 2 }, { 0.28f,1.5f,BD - RD }, wallC, 16, texBrick);
    // Right wall
    B({ BW - 0.14f,1.25f,(BD - RD) / 2 }, { 0.28f,2.5f,BD - RD }, wallC, 16, texBrick);
    for (int i = 0; i < 4; ++i) {
        float wz = 3.0f + i * 5.0f;
        B({ BW - 0.14f,5.5f,wz + 1.5f }, { 0.10f,3.5f,2.8f }, winC, 128, texWindow);
    }
    B({ BW - 0.14f,BH - 0.75f,(BD - RD) / 2 }, { 0.28f,1.5f,BD - RD }, wallC, 16, texBrick);
    // ── Front wall – proper shutter opening, no glass in the bay opening ──────────
    // Single large door opening  x∈[7, BW-7] = 38 m wide, y∈[0, SHUTTER_H=8.8]
    // NO geometry in that rectangle → shutter panels drawn separately via drawShutterDoor.
    const float doorXL = 7.0f,  doorXR = BW - 7.0f;  // x = 7 .. 45
    const float doorOpenH = 8.8f;                      // matches SHUTTER_H
    // Solid end pillars (left & right of opening, full height)
    B({ doorXL / 2,         BH / 2, BD }, { doorXL * 2.0f,    BH, 0.28f }, wallC, 16, texBrick);
    B({ (doorXR + BW) / 2,  BH / 2, BD }, { (BW - doorXR) * 2, BH, 0.28f }, wallC, 16, texBrick);
    // Header above the opening  (doorOpenH → BH)
    float hdrH = BH - doorOpenH - 0.05f;
    B({ BW / 2, doorOpenH + hdrH / 2, BD }, { doorXR - doorXL, hdrH, 0.28f }, wallC, 16, texBrick);
    // Top fascia band (full width)
    B({ BW / 2, BH - 0.30f, BD }, { BW, 0.60f, 0.35f }, wallC, 16, texBrick);
    // Steel door frame – vertical posts left/right of opening
    S({ doorXL,       doorOpenH / 2, BD + 0.12f }, { 0.25f, doorOpenH, 0.20f }, colC, 64);
    S({ doorXR,       doorOpenH / 2, BD + 0.12f }, { 0.25f, doorOpenH, 0.20f }, colC, 64);
    // Steel lintel across top of opening
    S({ BW / 2, doorOpenH + 0.14f, BD + 0.12f }, { doorXR - doorXL + 0.30f, 0.28f, 0.20f }, colC, 64);
    // Glowing signage strip above door
    E({ BW / 2, BH + 0.35f, BD + 0.06f }, { 14.0f, 0.90f, 0.05f }, glm::vec3(0.0f, 0.6f, 1.0f));
    S({ BW / 2, BH + 0.35f, BD + 0.04f }, { 14.2f, 1.0f,  0.08f }, glm::vec3(0.05f, 0.05f, 0.06f), 128);
    // Steel columns
    for (int i = 0; i <= 5; ++i) {
        float cx = (float)i * (BW / 5);
        S({ cx,BH / 2,0.15f }, { 0.30f,BH,0.30f }, colC, 64);
        S({ cx,BH / 2,BD - RD - 0.15f }, { 0.30f,BH,0.30f }, colC, 64);
    }
    // Roof
    B({ BW / 2,BH + 0.15f,BD / 2 }, { BW + 0.4f,0.30f,BD + 0.4f }, roofC, 16, texRoof);
    float zzArr[2] = { -0.15f,BD + 0.15f };
    for (int zi = 0; zi < 2; zi++)
        S({ BW / 2,BH,zzArr[zi] }, { BW + 0.4f,0.6f,0.3f }, { 0.20f,0.20f,0.22f }, 32);
    float xxArr[2] = { -0.15f,BW + 0.15f };
    for (int xi = 0; xi < 2; xi++)
        S({ xxArr[xi],BH,BD / 2 }, { 0.3f,0.6f,BD + 0.4f }, { 0.20f,0.20f,0.22f }, 32);
    // Ceiling
    B({ BW / 2,BH - 0.08f,(BD - RD) / 2 }, { BW - 0.6f,0.16f,BD - RD - 0.6f }, ceilC, 8, 0);
    sh.setBool("useTexture", false);
    B({ BW / 2,BH - 0.08f,BD - RD / 2 }, { BW - 0.6f,0.16f,RD - 0.6f }, ceilC, 8, 0);
    sh.setBool("useTexture", false);
    // LED strips
    for (int row = 0; row < 3; ++row) {
        float lz = 4.0f + row * 5.5f;
        for (int col = 0; col < 5; ++col)
            E({ 5.0f + col * 10.0f,BH - 0.16f,lz }, { 2.5f,0.05f,0.35f }, ledC);
    }
    for (int col = 0; col < 4; ++col)
        E({ 6.5f + col * 12.0f,BH - 0.16f,BD - RD / 2 }, { 3.0f,0.05f,0.25f }, ledC);
    // Glowing sign
    E({ BW / 2,BH + 0.35f,BD + 0.06f }, { 14.0f,0.9f,0.05f }, { 0.0f,0.6f,1.0f });
    S({ BW / 2,BH + 0.35f,BD + 0.04f }, { 14.2f,1.0f,0.08f }, { 0.05f,0.05f,0.06f }, 128);

    // Reception furniture
    glm::vec3 rO = orig + glm::vec3(0, 0, BD - RD);
    auto RD2 = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2 = 32) {
        solidCube(cube, sh, glm::scale(glm::translate(glm::mat4(1.0f), rO + t), s), c, sh2);
        };
    RD2({ BW / 2,0.90f,1.8f }, { 10.0f,0.12f,1.8f }, { 0.95f,0.95f,0.95f }, 64);
    RD2({ BW / 2,0.50f,1.8f }, { 10.0f,0.80f,0.18f }, { 0.92f,0.92f,0.94f }, 32);
    RD2({ BW / 2 - 4.8f,0.50f,2.5f }, { 0.18f,0.80f,1.6f }, { 0.92f,0.92f,0.94f }, 32);
    float mxArr[3] = { BW / 2 - 3.5f, BW / 2, BW / 2 + 3.5f };
    for (int mi = 0; mi < 3; mi++) {
        RD2({ mxArr[mi],1.05f,1.0f }, { 0.38f,0.28f,0.04f }, { 0.05f,0.05f,0.05f }, 128);
        cube.drawEmissive(sh, glm::scale(glm::translate(glm::mat4(1.0f), rO + glm::vec3(mxArr[mi], 1.05f, 1.02f)),
            glm::vec3(0.34f, 0.24f, 0.01f)), { 0.1f,0.5f,0.9f });
        RD2({ mxArr[mi],0.95f,1.0f }, { 0.06f,0.12f,0.04f }, { 0.1f,0.1f,0.1f }, 32);
    }
    for (int ci = 0; ci < 4; ++ci) {
        float cx = 8.0f + ci * 8.5f, cz = 4.5f;
        glm::vec3 cp = rO + glm::vec3(cx, 0, cz);
        glm::vec3 chC(0.15f, 0.15f, 0.18f);
        solidCube(cube, sh, TRS(cp + glm::vec3(0, 0.22f, 0), { 0,0,0 }, { 0.75f,0.08f,0.75f }), chC, 64);
        solidCube(cube, sh, TRS(cp + glm::vec3(0, 0.58f, -0.32f), { 15,0,0 }, { 0.75f,0.65f,0.08f }), chC, 64);
        float lxs[2] = { -0.3f,0.3f }, lzs[2] = { -0.3f,0.3f };
        for (int ix = 0; ix < 2; ix++) for (int iz = 0; iz < 2; iz++)
            solidCube(cube, sh, TRS(cp + glm::vec3(lxs[ix], 0.1f, lzs[iz]), { 0,0,0 }, { 0.06f,0.22f,0.06f }), { 0.6f,0.6f,0.62f }, 32);
    }
    RD2({ BW / 2,0.40f,4.5f }, { 3.0f,0.08f,1.0f }, { 0.15f,0.15f,0.18f }, 128);
    float tlxA[2] = { -1.3f,1.3f }, tlzA[2] = { -0.35f,0.35f };
    for (int ix = 0; ix < 2; ix++) for (int iz = 0; iz < 2; iz++)
        RD2({ BW / 2 + tlxA[ix],0.2f,4.5f + tlzA[iz] }, { 0.06f,0.40f,0.06f }, { 0.15f,0.15f,0.18f }, 64);
    E({ BW / 2,3.5f,RD - 0.25f }, { 10.0f,2.2f,0.04f }, { 0.05f,0.1f,0.25f });
    S({ BW / 2,3.5f,RD - 0.28f }, { 10.2f,2.4f,0.10f }, { 0.05f,0.05f,0.06f }, 64);
}

// ============================================================
//  drawWorkbench  – wall-mounted workbench with tools
// ============================================================
void drawWorkbench(Cube& cube, Shader& sh, glm::vec3 pos, float yaw = 0)
{
    const glm::vec3 woodC(0.50f, 0.32f, 0.14f);
    const glm::vec3 metC(0.55f, 0.55f, 0.58f);
    glm::mat4 base = myRotate(glm::translate(glm::mat4(1.0f), pos), glm::radians(yaw), glm::vec3(0, 1, 0));
    auto B = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2 = 32) {
        solidCube(cube, sh, glm::scale(glm::translate(base, t), s), c, sh2); };
    auto E = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c) {
        cube.drawEmissive(sh, glm::scale(glm::translate(base, t), s), c); };
    // Table top
    B({ 0,0.90f,0 }, { 2.20f,0.08f,0.75f }, woodC, 32);
    // Legs
    float lxArr2[2] = { -0.95f,0.95f };
    float lzArr2[2] = { -0.30f,0.30f };
    for (int ix = 0; ix < 2; ix++) for (int iz = 0; iz < 2; iz++)
        B({ lxArr2[ix],0.45f,lzArr2[iz] }, { 0.08f,0.90f,0.08f }, metC, 64);
    // Lower shelf
    B({ 0,0.35f,0 }, { 2.10f,0.05f,0.65f }, woodC, 16);
    // Pegboard wall above bench
    B({ 0,1.55f,-0.36f }, { 2.20f,1.30f,0.06f }, glm::vec3(0.70f, 0.70f, 0.72f), 16);
    // Peg hooks
    for (int i = 0; i < 6; i++) {
        float px = -0.90f + i * 0.36f;
        B({ px,1.65f,-0.30f }, { 0.04f,0.04f,0.12f }, metC, 64);
    }
    // Diagnostic monitor on bench
    B({ 0.80f,0.98f,-0.20f }, { 0.40f,0.28f,0.04f }, glm::vec3(0.05f, 0.05f, 0.06f), 128);
    E({ 0.80f,0.98f,-0.17f }, { 0.36f,0.24f,0.01f }, glm::vec3(0.1f, 0.6f, 0.9f));
    // Tools on bench – wrench, socket box
    B({ -0.20f,0.942f,0.10f }, { 0.45f,0.04f,0.06f }, metC, 64);
    B({ 0.22f,0.942f,-0.10f }, { 0.28f,0.06f,0.18f }, glm::vec3(0.65f, 0.08f, 0.08f), 32);
    // Hanging lamp over bench (emissive)
    B({ 0,1.92f,0 }, { 0.04f,0.10f,0.04f }, metC, 32);
    E({ 0,1.82f,0 }, { 0.50f,0.06f,0.18f }, glm::vec3(0.95f, 0.90f, 0.75f));
    // Vice
    B({ -0.80f,0.95f,0.28f }, { 0.28f,0.16f,0.18f }, metC, 64);
}

// ============================================================
//  drawTireRack  – stacked tire storage unit
// ============================================================
void drawTireRack(Cube& cube, Shader& sh, glm::vec3 pos, float yaw = 0)
{
    const glm::vec3 metC(0.45f, 0.45f, 0.48f);
    glm::mat4 base = myRotate(glm::translate(glm::mat4(1.0f), pos), glm::radians(yaw), glm::vec3(0, 1, 0));
    auto B = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2 = 32) {
        solidCube(cube, sh, glm::scale(glm::translate(base, t), s), c, sh2); };
    // Uprights
    float fxArr[2] = { -1.30f,1.30f };
    for (int i = 0; i < 2; i++) {
        B({ fxArr[i],1.55f,0 }, { 0.08f,3.10f,0.08f }, metC, 32);
        B({ fxArr[i],1.55f,0.28f }, { 0.08f,3.10f,0.08f }, metC, 32);
    }
    // 3 tiers of tires
    for (int row = 0; row < 3; row++) {
        float ry = 0.40f + row * 1.05f;
        B({ 0,ry - 0.05f,0 }, { 2.60f,0.08f,0.10f }, metC, 32);
        for (int ti = 0; ti < 4; ti++) {
            float tx = -0.95f + ti * 0.65f;
            drawCube(cube, sh, glm::scale(glm::translate(base, { tx,ry + 0.28f,0 }),
                { 0.58f,0.58f,0.24f }),
                glm::vec3(0.08f) * 0.3f, glm::vec3(0.08f), glm::vec3(0.05f), 8, texTire);
            solidCube(cube, sh, glm::scale(glm::translate(base, { tx,ry + 0.28f,0.13f }),
                { 0.34f,0.34f,0.05f }), glm::vec3(0.78f, 0.78f, 0.80f), 128);
        }
    }
    B({ 0,3.10f,0 }, { 2.60f,0.08f,0.10f }, metC, 32);
}

// ============================================================
//  drawWheelBalancer  – wheel balancing machine
// ============================================================
void drawWheelBalancer(Cube& cube, Shader& sh, glm::vec3 pos, float yaw = 0)
{
    const glm::vec3 whtC(0.90f, 0.90f, 0.92f);
    const glm::vec3 blkC(0.10f, 0.10f, 0.12f);
    const glm::vec3 redC(0.80f, 0.12f, 0.06f);
    glm::mat4 base = myRotate(glm::translate(glm::mat4(1.0f), pos), glm::radians(yaw), glm::vec3(0, 1, 0));
    auto B = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2 = 32) {
        solidCube(cube, sh, glm::scale(glm::translate(base, t), s), c, sh2); };
    auto E = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c) {
        cube.drawEmissive(sh, glm::scale(glm::translate(base, t), s), c); };
    B({ 0,0.50f,0 }, { 1.20f,1.0f,0.80f }, whtC, 32);
    B({ 0,0.08f,0 }, { 1.25f,0.16f,0.85f }, blkC, 16);
    B({ 0,0.05f,0 }, { 1.22f,0.04f,0.82f }, redC, 32);
    B({ 0.30f,1.05f,0 }, { 0.60f,0.10f,0.10f }, blkC, 32);
    B({ 0.65f,1.05f,0 }, { 0.08f,0.08f,0.08f }, glm::vec3(0.60f, 0.60f, 0.62f), 128);
    B({ -0.35f,0.90f,0.41f }, { 0.42f,0.28f,0.05f }, blkC, 128);
    E({ -0.35f,0.90f,0.44f }, { 0.38f,0.24f,0.01f }, glm::vec3(0.2f, 0.8f, 0.2f));
    drawCube(cube, sh, glm::scale(glm::translate(base, { 0.65f,1.05f,0 }), { 0.58f,0.58f,0.22f }),
        glm::vec3(0.08f) * 0.3f, glm::vec3(0.08f), glm::vec3(0.05f), 8, texTire);
    B({ 0.65f,1.05f,0.12f }, { 0.36f,0.36f,0.05f }, glm::vec3(0.78f, 0.78f, 0.80f), 128);
}

// ============================================================
//  drawRimDisplayWall  – premium alloy rim showcase wall
// ============================================================
void drawRimDisplayWall(Cube& cube, Shader& sh, glm::vec3 pos, float yaw = 0, int count = 8)
{
    const glm::vec3 wallC(0.88f, 0.88f, 0.90f);
    glm::vec3 rimCols[8] = {
        {0.85f,0.85f,0.88f},{0.92f,0.80f,0.55f},{0.30f,0.30f,0.32f},{0.88f,0.62f,0.10f},
        {0.85f,0.85f,0.88f},{0.20f,0.22f,0.25f},{0.92f,0.80f,0.55f},{0.70f,0.70f,0.72f}
    };
    glm::mat4 base = myRotate(glm::translate(glm::mat4(1.0f), pos), glm::radians(yaw), glm::vec3(0, 1, 0));
    float panW = (float)count * 0.80f + 0.40f;
    solidCube(cube, sh, glm::scale(glm::translate(base, { 0,1.40f,-0.06f }), { panW,2.80f,0.12f }), wallC, 16);
    cube.drawEmissive(sh, glm::scale(glm::translate(base, { 0,2.86f,0 }), { panW - 0.20f,0.05f,0.10f }),
        glm::vec3(0.90f, 0.90f, 1.0f));
    for (int i = 0; i < count; i++) {
        float rx = -((float)(count - 1) / 2.0f) * 0.80f + i * 0.80f;
        float ry = 1.10f + (i % 2) * 0.90f;
        glm::vec3 rc = rimCols[i % 8];
        drawCube(cube, sh, glm::scale(glm::translate(base, { rx,ry,-0.01f }), { 0.62f,0.62f,0.18f }),
            glm::vec3(0.08f) * 0.3f, glm::vec3(0.08f), glm::vec3(0.05f), 8, texTire);
        solidCube(cube, sh, glm::scale(glm::translate(base, { rx,ry,0.09f }), { 0.42f,0.42f,0.14f }), rc, 256);
        solidCube(cube, sh, glm::scale(glm::translate(base, { rx,ry,0.16f }), { 0.08f,0.40f,0.04f }), rc, 256);
        solidCube(cube, sh, glm::scale(glm::translate(base, { rx,ry,0.16f }), { 0.40f,0.08f,0.04f }), rc, 256);
        solidCube(cube, sh, glm::scale(glm::translate(base, { rx,ry,0.16f }), { 0.08f,0.08f,0.04f }),
            glm::vec3(0.40f, 0.40f, 0.42f), 128);
        // Price tag
        solidCube(cube, sh, glm::scale(glm::translate(base, { rx,ry - 0.34f,0.12f }), { 0.22f,0.08f,0.02f }),
            glm::vec3(0.95f, 0.95f, 0.95f), 64);
    }
}

// ============================================================
//  drawPressureWasher
// ============================================================
void drawPressureWasher(Cube& cube, Shader& sh, glm::vec3 pos)
{
    const glm::vec3 yelC(0.88f, 0.80f, 0.05f);
    const glm::vec3 blkC(0.10f, 0.10f, 0.12f);
    auto B = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2 = 32) {
        solidCube(cube, sh, TRS(pos + t, glm::vec3(0), s), c, sh2); };
    auto E = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c) {
        cube.drawEmissive(sh, TRS(pos + t, glm::vec3(0), s), c); };
    B({ 0,0.40f,0 }, { 0.60f,0.80f,0.45f }, yelC, 32);
    B({ 0,0.08f,0 }, { 0.65f,0.16f,0.50f }, blkC, 16);
    B({ 0.28f,0.42f,0 }, { 0.06f,0.36f,0.06f }, blkC, 32);
    B({ 0.28f,0.65f,0 }, { 0.22f,0.06f,0.06f }, blkC, 32);
    B({ 0.50f,0.65f,0 }, { 0.06f,0.06f,0.28f }, blkC, 32);
    B({ -0.28f,0.50f,0 }, { 0.18f,0.42f,0.18f }, glm::vec3(0.20f, 0.55f, 0.80f), 16);
    B({ 0,0.82f,0.28f }, { 0.25f,0.25f,0.10f }, blkC, 32);
    E({ 0,0.90f,0 }, { 0.08f,0.04f,0.08f }, glm::vec3(0.5f, 0.9f, 0.5f));
}

// ============================================================
//  drawFireExtinguisher
// ============================================================
void drawFireExtinguisher(Cube& cube, Shader& sh, glm::vec3 pos)
{
    solidCube(cube, sh, TRS(pos + glm::vec3(0, 0.42f, 0), glm::vec3(0), { 0.18f,0.80f,0.18f }),
        glm::vec3(0.88f, 0.08f, 0.08f), 32);
    solidCube(cube, sh, TRS(pos + glm::vec3(0, 0.88f, 0), glm::vec3(0), { 0.14f,0.12f,0.14f }),
        glm::vec3(0.50f, 0.50f, 0.52f), 64);
    solidCube(cube, sh, TRS(pos + glm::vec3(0, 0.95f, 0), glm::vec3(0), { 0.06f,0.12f,0.06f }),
        glm::vec3(0.85f, 0.82f, 0.05f), 64);
    solidCube(cube, sh, TRS(pos + glm::vec3(0.05f, 0.96f, 0), glm::vec3(0, 0, 30), { 0.25f,0.04f,0.04f }),
        glm::vec3(0.50f, 0.50f, 0.52f), 64);
    cube.drawEmissive(sh, TRS(pos + glm::vec3(0, 0.44f, 0.10f), glm::vec3(0), { 0.10f,0.14f,0.01f }),
        glm::vec3(0.95f, 0.95f, 0.95f));
}

// ============================================================
//  drawSofaChair  – lounge chair
// ============================================================
void drawSofaChair(Cube& cube, Shader& sh, glm::vec3 pos, float yaw = 0)
{
    const glm::vec3 sofaC(0.15f, 0.20f, 0.38f);
    const glm::vec3 legC(0.45f, 0.35f, 0.20f);
    glm::mat4 base = myRotate(glm::translate(glm::mat4(1.0f), pos), glm::radians(yaw), glm::vec3(0, 1, 0));
    auto B = [&](glm::vec3 t, glm::vec3 s, glm::vec3 c, float sh2 = 32) {
        solidCube(cube, sh, glm::scale(glm::translate(base, t), s), c, sh2); };
    B({ 0,0.44f,0 }, { 1.60f,0.22f,0.80f }, sofaC, 32);
    B({ 0,0.78f,-0.35f }, { 1.60f,0.70f,0.18f }, sofaC, 32);
    B({ -0.76f,0.52f,0 }, { 0.12f,0.40f,0.80f }, sofaC, 32);
    B({ 0.76f,0.52f,0 }, { 0.12f,0.40f,0.80f }, sofaC, 32);
    B({ 0,0.56f,-0.02f }, { 1.44f,0.10f,0.66f }, sofaC * 1.15f, 16);
    float lxArr3[2] = { -0.65f,0.65f };
    float lzArr3[2] = { -0.32f,0.32f };
    for (int ix = 0; ix < 2; ix++) for (int iz = 0; iz < 2; iz++)
        B({ lxArr3[ix],0.18f,lzArr3[iz] }, { 0.06f,0.36f,0.06f }, legC, 64);
}

// ============================================================
//  drawCoffeeTable
// ============================================================
void drawCoffeeTable(Cube& cube, Shader& sh, glm::vec3 pos)
{
    const glm::vec3 glassC(0.50f, 0.70f, 0.85f);
    const glm::vec3 metC(0.62f, 0.62f, 0.64f);
    solidCube(cube, sh, TRS(pos + glm::vec3(0, 0.48f, 0), glm::vec3(0), { 1.10f,0.06f,0.60f }), glassC, 128);
    float lxArr4[2] = { -0.45f,0.45f };
    float lzArr4[2] = { -0.22f,0.22f };
    for (int ix = 0; ix < 2; ix++) for (int iz = 0; iz < 2; iz++)
        solidCube(cube, sh, TRS(pos + glm::vec3(lxArr4[ix], 0.24f, lzArr4[iz]), glm::vec3(0), { 0.05f,0.48f,0.05f }), metC, 64);
    solidCube(cube, sh, TRS(pos + glm::vec3(0.15f, 0.52f, 0.08f), glm::vec3(0, 15, 0), { 0.28f,0.02f,0.20f }),
        glm::vec3(0.80f, 0.20f, 0.12f), 16);
}

// ============================================================
//  drawTVScreen  – wall-mounted display
// ============================================================
void drawTVScreen(Cube& cube, Shader& sh, glm::vec3 pos, float yaw = 0, float W = 1.4f, float H = 0.85f)
{
    glm::mat4 base = myRotate(glm::translate(glm::mat4(1.0f), pos), glm::radians(yaw), glm::vec3(0, 1, 0));
    solidCube(cube, sh, glm::scale(glm::translate(base, { 0,H / 2 + 0.05f,0 }), { W + 0.10f,H + 0.10f,0.10f }),
        glm::vec3(0.06f, 0.06f, 0.07f), 128);
    cube.drawEmissive(sh, glm::scale(glm::translate(base, { 0,H / 2 + 0.05f,0.06f }), { W,H,0.02f }),
        glm::vec3(0.08f, 0.40f, 0.75f));
    solidCube(cube, sh, glm::scale(glm::translate(base, { 0,0.20f,0 }), { 0.06f,0.40f,0.06f }),
        glm::vec3(0.25f, 0.25f, 0.28f), 64);
    solidCube(cube, sh, glm::scale(glm::translate(base, { 0,0.04f,0 }), { 0.50f,0.06f,0.28f }),
        glm::vec3(0.25f, 0.25f, 0.28f), 64);
}

// ============================================================
//  drawSteelTruss  – roof truss (Warren truss)
// ============================================================
void drawSteelTruss(Cube& cube, Shader& sh, glm::vec3 pos, float span, float depth = 1.40f)
{
    const glm::vec3 steelC(0.38f, 0.40f, 0.42f);
    float h = depth;
    solidCube(cube, sh, TRS(pos + glm::vec3(span / 2, 0, 0), glm::vec3(0), { span,0.14f,0.12f }), steelC, 64);
    solidCube(cube, sh, TRS(pos + glm::vec3(span / 2, -h, 0), glm::vec3(0), { span,0.12f,0.10f }), steelC, 64);
    int nPanels = 8;
    for (int i = 0; i <= nPanels; i++) {
        float vx = (float)i * (span / nPanels);
        solidCube(cube, sh, TRS(pos + glm::vec3(vx, -h / 2, 0), glm::vec3(0), { 0.10f,h,0.08f }), steelC, 64);
    }
    for (int i = 0; i < nPanels; i++) {
        float panW = span / nPanels;
        float angDeg = glm::degrees(atanf(h / panW));
        float diagLen = sqrtf(panW * panW + h * h) * 1.05f;
        solidCube(cube, sh,
            TRS(pos + glm::vec3((float)i * panW + panW / 2, -h / 2, 0),
                glm::vec3(0, 0, (i % 2 == 0) ? angDeg : -angDeg),
                { diagLen,0.07f,0.07f }), steelC, 64);
    }
}

// ============================================================
//  drawCarPathMarkings  – floor guides from entrance to bays
// ============================================================
void drawCarPathMarkings(Cube& cube, Shader& sh,
    glm::vec3 BLDG, float BD, float BW, const float bayX[], float bayZ)
{
    // Centre dashed entry lane
    float pathX = BW / 2.0f;
    for (int i = 0; i < 10; i++) {
        float pz = (BD - 1.0f) - (float)i * 2.8f;
        if (pz < bayZ - 1.5f) break;
        cube.drawEmissive(sh,
            TRS(BLDG + glm::vec3(pathX, 0.01f, pz), glm::vec3(0), { 0.18f,0.02f,1.40f }),
            glm::vec3(0.88f, 0.88f, 0.90f));
    }
    // Green entry arrow (outside, in front of shutter)
    cube.drawEmissive(sh,
        TRS(BLDG + glm::vec3(pathX, 0.01f, BD + 1.0f), glm::vec3(0), { 1.60f,0.02f,0.18f }),
        glm::vec3(0.05f, 0.75f, 0.12f));
    cube.drawEmissive(sh,
        TRS(BLDG + glm::vec3(pathX, 0.01f, BD + 1.6f), glm::vec3(0, 0, -30), { 0.90f,0.02f,0.10f }),
        glm::vec3(0.05f, 0.75f, 0.12f));
    cube.drawEmissive(sh,
        TRS(BLDG + glm::vec3(pathX, 0.01f, BD + 1.6f), glm::vec3(0, 0, 30), { 0.90f,0.02f,0.10f }),
        glm::vec3(0.05f, 0.75f, 0.12f));
    // Yellow bay markers
    for (int b = 0; b < 5; b++) {
        float bx = bayX[b];
        cube.drawEmissive(sh,
            TRS(BLDG + glm::vec3(bx, 0.01f, BD - 1.8f), glm::vec3(0), { 0.15f,0.02f,3.0f }),
            glm::vec3(0.90f, 0.78f, 0.05f));
        cube.drawEmissive(sh,
            TRS(BLDG + glm::vec3(bx, 0.01f, bayZ), glm::vec3(0, 90, 0), { 0.15f,0.02f,8.0f }),
            glm::vec3(0.90f, 0.78f, 0.05f));
    }
}

// ============================================================
//  setupSpotLight
// ============================================================
// ============================================================
//  drawHUD  – 2-D overlay bars for Fuel and Damage (drive mode only)
// ============================================================
void drawHUD(Cube& cube, Shader& sh, int vpW, int vpH)
{
    if (!driveMode) return;

    glDisable(GL_DEPTH_TEST);
    sh.use();

    // Orthographic screen-space projection; identity view
    sh.setMat4("projection",
        glm::ortho(0.f, (float)vpW, 0.f, (float)vpH, -1.f, 1.f));
    sh.setMat4("view",    glm::mat4(1.f));
    sh.setVec3("viewPos", {0.f, 0.f, 1.f});
    // All lighting bypassed: emissive path returns immediately
    sh.setBool("useTexture", false);

    // Helper: draw a filled screen-space rectangle using emissive cube
    auto rect = [&](float x, float y, float w, float h, glm::vec3 col) {
        cube.drawEmissive(sh,
            glm::scale(glm::translate(glm::mat4(1.f),
                {x + w * .5f, y + h * .5f, 0.f}),
                {w, h, 1.f}), col);
    };

    const float BW  = 220.f, BH = 22.f;
    const float BX  = 28.f;
    const float BY  = (float)vpH - 52.f;   // fuel bar Y
    const float DY  = BY - 32.f;           // damage bar Y
    const float BRD = 2.f;                  // border thickness

    // ── FUEL BAR ─────────────────────────────────────────────
    rect(BX, BY, BW, BH, {0.15f,0.15f,0.15f});  // background
    if (fuelLevel > 0.001f) {
        glm::vec3 fc = fuelLevel > 0.5f  ? glm::vec3{0.08f,0.88f,0.08f} :
                       fuelLevel > 0.25f ? glm::vec3{0.92f,0.72f,0.04f} :
                                           glm::vec3{0.92f,0.08f,0.04f};
        rect(BX, BY, BW * fuelLevel, BH, fc);
    }
    // Border
    rect(BX, BY,           BW, BRD, {0.9f,0.9f,0.9f});
    rect(BX, BY+BH-BRD,    BW, BRD, {0.9f,0.9f,0.9f});
    rect(BX, BY,           BRD, BH, {0.9f,0.9f,0.9f});
    rect(BX+BW-BRD, BY,    BRD, BH, {0.9f,0.9f,0.9f});
    // Yellow icon  (⛽ fuel indicator)
    rect(BX - 18.f, BY + 4.f, 14.f, 14.f, {0.96f,0.78f,0.04f});
    // Cyan refill-progress strip (shown while in fuel zone)
    if (inFuelZone && fuelZoneTimer > 0.f) {
        float p = fuelZoneTimer / FUEL_FILL_SECS;
        rect(BX, BY - 8.f, BW * p, 5.f, {0.10f,0.85f,1.00f});
    }

    // ── DAMAGE BAR ───────────────────────────────────────────
    rect(BX, DY, BW, BH, {0.15f,0.15f,0.15f});  // background
    if (damageLevel > 0.001f) {
        glm::vec3 dc = damageLevel < 0.3f ? glm::vec3{0.08f,0.88f,0.08f} :
                       damageLevel < 0.6f ? glm::vec3{0.92f,0.72f,0.04f} :
                                            glm::vec3{0.92f,0.08f,0.04f};
        rect(BX, DY, BW * damageLevel, BH, dc);
    }
    // Border
    rect(BX, DY,           BW, BRD, {0.9f,0.9f,0.9f});
    rect(BX, DY+BH-BRD,    BW, BRD, {0.9f,0.9f,0.9f});
    rect(BX, DY,           BRD, BH, {0.9f,0.9f,0.9f});
    rect(BX+BW-BRD, DY,    BRD, BH, {0.9f,0.9f,0.9f});
    // Red icon (damage indicator)
    rect(BX - 18.f, DY + 4.f, 14.f, 14.f, {0.92f,0.08f,0.04f});

    glEnable(GL_DEPTH_TEST);
}

// ============================================================
//  drawSkybox – physics-based Rayleigh/Mie atmospheric sky
// ============================================================
// Draw AFTER renderScene so depth buffer is populated; LEQUAL fills sky where no geometry is.
void drawSkybox(Shader& skySh, const glm::mat4& view, const glm::mat4& proj)
{
    // Strip translation – sky cube is infinitely far, moves only with camera rotation
    glm::mat4 rotView = glm::mat4(glm::mat3(view));

    glDepthFunc(GL_LEQUAL);   // pass where depth = 1.0 (far plane, i.e. no geometry)
    glDepthMask(GL_FALSE);    // sky doesn't write to depth buffer

    skySh.use();
    skySh.setMat4("view",       rotView);
    skySh.setMat4("projection", proj);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxTex);
    skySh.setInt("skybox", 1);  // ensure sampler points to unit 1 every frame

    glBindVertexArray(skyboxVAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glActiveTexture(GL_TEXTURE0); // restore active unit
}

// ============================================================
//  setupSectionSpotLights – 10 spots across all 4 sections
// ============================================================
static void setSectionSpot(Shader& sh, int i,
    glm::vec3 pos, glm::vec3 diff, glm::vec3 spec, glm::vec3 amb,
    float innerDeg, float outerDeg, float lin, float quad)
{
    auto n = [&](const char* f) {
        return std::string("sectionSpots[") + std::to_string(i) + "]." + f;
    };
    sh.setVec3 (n("position").c_str(),    pos);
    sh.setVec3 (n("direction").c_str(),   {0.f,-1.f,0.f});
    sh.setFloat(n("cutOff").c_str(),      cosf(glm::radians(innerDeg)));
    sh.setFloat(n("outerCutOff").c_str(), cosf(glm::radians(outerDeg)));
    sh.setVec3 (n("ambient").c_str(),     amb);
    sh.setVec3 (n("diffuse").c_str(),     diff);
    sh.setVec3 (n("specular").c_str(),    spec);
    sh.setFloat(n("constant").c_str(),    1.0f);
    sh.setFloat(n("linear").c_str(),      lin);
    sh.setFloat(n("quadratic").c_str(),   quad);
}

void setupSectionSpotLights(Shader& sh, bool on)
{
    sh.setBool("sectionSpotsOn", on);
    // 0: Paint zone red box (warm white, tight cone)
    setSectionSpot(sh, 0, {75.f,13.5f,24.f},
        {1.00f,0.95f,0.85f},{0.80f,0.80f,0.70f},{0.12f,0.12f,0.10f}, 18.f,26.f, 0.045f,0.003f);
    // 1: Paint booth 1 (neutral white)
    setSectionSpot(sh, 1, {62.f,13.5f,10.f},
        {0.95f,0.96f,1.00f},{0.70f,0.72f,0.82f},{0.08f,0.08f,0.10f}, 22.f,32.f, 0.055f,0.003f);
    // 2: Paint booth 2 (neutral white)
    setSectionSpot(sh, 2, {62.f,13.5f,30.f},
        {0.95f,0.96f,1.00f},{0.70f,0.72f,0.82f},{0.08f,0.08f,0.10f}, 22.f,32.f, 0.055f,0.003f);
    // 3: Showroom car 1 – warm gold display spot
    setSectionSpot(sh, 3, {158.f,13.5f,24.f},
        {1.00f,0.88f,0.55f},{0.90f,0.78f,0.45f},{0.14f,0.12f,0.06f}, 16.f,24.f, 0.040f,0.002f);
    // 4: Showroom car 2 – warm gold display spot
    setSectionSpot(sh, 4, {174.f,13.5f,24.f},
        {1.00f,0.88f,0.55f},{0.90f,0.78f,0.45f},{0.14f,0.12f,0.06f}, 16.f,24.f, 0.040f,0.002f);
    // 5: Showroom car 3 – warm gold display spot
    setSectionSpot(sh, 5, {190.f,13.5f,24.f},
        {1.00f,0.88f,0.55f},{0.90f,0.78f,0.45f},{0.14f,0.12f,0.06f}, 16.f,24.f, 0.040f,0.002f);
    // 6: Body shop car 1 – cool work light
    setSectionSpot(sh, 6, {107.f,13.5f,10.f},
        {0.88f,0.93f,1.00f},{0.68f,0.75f,0.92f},{0.06f,0.07f,0.10f}, 22.f,32.f, 0.055f,0.004f);
    // 7: Body shop car 2 – cool work light
    setSectionSpot(sh, 7, {140.f,13.5f,36.f},
        {0.88f,0.93f,1.00f},{0.68f,0.75f,0.92f},{0.06f,0.07f,0.10f}, 22.f,32.f, 0.055f,0.004f);
    // 8: Assembly lift 1 – bright work light
    setSectionSpot(sh, 8, { 8.f,13.5f,10.f},
        {0.96f,0.95f,0.90f},{0.80f,0.80f,0.75f},{0.09f,0.09f,0.08f}, 20.f,30.f, 0.050f,0.003f);
    // 9: Assembly lift 2 – bright work light
    setSectionSpot(sh, 9, {26.f,13.5f,10.f},
        {0.96f,0.95f,0.90f},{0.80f,0.80f,0.75f},{0.09f,0.09f,0.08f}, 20.f,30.f, 0.050f,0.003f);
}

void setupSpotLight(Shader& sh, glm::vec3 position, bool on)
{
    sh.setBool("spotLightOn", on);
    sh.setVec3("spotLight.position", position);
    sh.setVec3("spotLight.direction", { 0,-1,0 });
    sh.setFloat("spotLight.cutOff", cosf(glm::radians(18.0f)));
    sh.setFloat("spotLight.outerCutOff", cosf(glm::radians(25.0f)));
    sh.setVec3("spotLight.ambient", { 0.10f,0.10f,0.08f });
    sh.setVec3("spotLight.diffuse", { 0.85f,0.80f,0.60f });
    sh.setVec3("spotLight.specular", { 0.50f,0.50f,0.40f });
    sh.setFloat("spotLight.constant", 1.0f);
    sh.setFloat("spotLight.linear", 0.04f);
    sh.setFloat("spotLight.quadratic", 0.002f);
}

// ============================================================
//  drawBezierConceptCar
//  Draws a concept car using 3 Bezier surfaces of revolution:
//    body  – smooth egg body shell (profile: radius,height)
//    cabin – glass dome windshield+roof
//    wheel – torus tyre (profile: C-section, rotated 90° to stand)
//  pos: world centre at display pedestal top
//  yaw: heading in degrees
// ============================================================
void drawBezierConceptCar(
    BezierSurface& body, BezierSurface& cabin, BezierSurface& wheel,
    Shader& sh, glm::vec3 pos, float yaw)
{
    const float rad = glm::radians(yaw);
    const glm::mat4 Ry = glm::rotate(glm::mat4(1.f), rad, {0.f,1.f,0.f});

    // ── Body: profile y 0→1, scale to (length=2.1, height=0.68, width=1.15) ──
    body.draw(sh,
        glm::translate(glm::mat4(1.f), pos) * Ry *
        glm::scale(glm::mat4(1.f), {2.1f, 0.68f, 1.15f}));

    // ── Cabin glass dome: sits at shoulder height (≈60 % up the body) ─────────
    cabin.draw(sh,
        glm::translate(glm::mat4(1.f), pos + glm::vec3(0.f, 0.42f, 0.f)) * Ry *
        glm::scale(glm::mat4(1.f), {1.05f, 0.58f, 0.82f}));

    // ── 4 torus wheels ─────────────────────────────────────────────────────────
    // Profile C-section: r 0.52→0.83, y 0→0.44. Rotated 90° around Z so wheel
    // stands upright with axle along X.
    // After rotation+scale ws: outer radius in world-Y = 0.83*ws ≈ 0.35
    const float ws = 0.42f;              // uniform wheel scale
    const float wr = 0.83f * ws;         // outer radius ≈ 0.35

    // Wheel offsets in local car frame (±lx front/rear, ±lz left/right)
    const float lx = 1.10f, lz = 0.68f;
    float locs[4][2] = { {lx, lz}, {lx,-lz}, {-lx, lz}, {-lx,-lz} };

    for (auto& wl : locs) {
        // Apply car yaw to local offset
        float wx = pos.x + cosf(rad)*wl[0] - sinf(rad)*wl[1];
        float wz = pos.z + sinf(rad)*wl[0] + cosf(rad)*wl[1];

        wheel.draw(sh,
            glm::translate(glm::mat4(1.f), {wx, pos.y + wr, wz}) *
            Ry *
            glm::rotate(glm::mat4(1.f), glm::radians(90.f), {0.f,0.f,1.f}) *
            glm::scale(glm::mat4(1.f), glm::vec3(ws)));
    }
}

// ============================================================
//  renderScene
// ============================================================
void renderScene(Cube& cube, Sphere& sphere, Shader& sh,
    BezierSurface& potSurf, BezierSurface& trunkSurf,
    BezierSurface& canopySurf, BezierSurface& bushSurf,
    BezierSurface& bezCarBody, BezierSurface& bezCarCabin,
    BezierSurface& bezCarWheel,
    const glm::vec3& BLDG, float ROAD_Z,
    float BW, float BD,
    const float bayX[], float bayZ)
{
    // 1. City ground + road + street furniture + fractal trees
    drawCityBlock(cube, sphere, sh, BLDG, BW, BD, ROAD_Z);

    // 2. Complex building (4 sections)
    drawComplexBuilding(cube, sphere, sh, BLDG,
        conveyorOff, robotAngle, sprayAngle, sprayOn, pressHeight,
        sinf((float)glfwGetTime() * 1.4f) * 0.45f);  // worker arm swing

    // 3. Shutter door (spans full entrance X:[7,193], height 9m)
    {
        const float doorW = 186.0f;  // BW-14 = 200-14
        const float doorH = 9.0f;
        glm::vec3 shutPos = BLDG + glm::vec3(7.0f, 0.0f, BD - 0.02f);
        drawShutterDoor(cube, sh, shutPos, doorW, doorH, shutterOpenProgress);
    }

    // 4. Parking lot vehicles (in front of building)
    {
        float parkZ = BD + 4.0f;
        drawAttractiveCar(cube, sh, BLDG+glm::vec3( 15.f,0,parkZ+1.f),180.f,{.10f,.12f,.65f});
        drawWhiteCar     (cube, sh, BLDG+glm::vec3( 35.f,0,parkZ+1.f),180.f);
        drawAttractiveCar(cube, sh, BLDG+glm::vec3( 55.f,0,parkZ+1.f),180.f,{.45f,.05f,.05f});
        drawWhiteCar     (cube, sh, BLDG+glm::vec3( 80.f,0,parkZ+1.f),180.f);
        drawAttractiveCar(cube, sh, BLDG+glm::vec3(100.f,0,parkZ+1.f),180.f,{.05f,.32f,.20f});
        drawAttractiveCar(cube, sh, BLDG+glm::vec3(120.f,0,parkZ+1.f),180.f,{.65f,.55f,.05f});
        drawWhiteCar     (cube, sh, BLDG+glm::vec3(145.f,0,parkZ+1.f),180.f);
        drawAttractiveCar(cube, sh, BLDG+glm::vec3(165.f,0,parkZ+1.f),180.f,{.30f,.10f,.50f});
    }

    // Road stationary cars
    drawAttractiveCar(cube, sh, {-18.f,0,ROAD_Z-2.5f},  90.f,{.65f,.55f,.05f});
    drawWhiteCar     (cube, sh, {220.f,0,ROAD_Z+2.5f}, 270.f);

    // ── Fractal branch tree near workshop west entrance ───────────────────────
    drawFractalBranchTree(cube, sphere, sh, {8.f, 0.f, 57.f});

    // ── Bezier concept car on display pedestal (showroom exterior) ────────────
    {
        // Pedestal: polished dark grey marble stand
        solidCube(cube, sh,
            glm::scale(glm::translate(glm::mat4(1.f), {192.f, 0.5f, 55.5f}),
                       {9.0f, 1.0f, 6.5f}),
            {0.25f,0.25f,0.27f}, 64.f);
        // Pedestal top face highlight (lighter trim strip)
        solidCube(cube, sh,
            glm::scale(glm::translate(glm::mat4(1.f), {192.f, 1.02f, 55.5f}),
                       {9.2f, 0.06f, 6.7f}),
            {0.42f,0.42f,0.45f}, 32.f);
        // Concept car on top of pedestal (pos.y = 1.0 = pedestal surface)
        drawBezierConceptCar(bezCarBody, bezCarCabin, bezCarWheel,
                             sh, {192.f, 1.0f, 55.5f}, 20.f);
        // Name placard on pedestal front face
        cube.drawEmissive(sh,
            glm::scale(glm::translate(glm::mat4(1.f), {192.f, 0.52f, 52.15f}),
                       {5.5f, 0.32f, 0.04f}),
            {0.92f, 0.88f, 0.72f});
    }

    // 7. Drive-mode car (player) – colour set permanently by paint zone
    drawAttractiveCar(cube, sh,
        { drivePosX, 0.0f, drivePosZ },
        driveYaw,
        driveCarColor,
        true, driveSpeed > 0.5f, driveWheelRot);
}

// ============================================================
//  Callbacks
// ============================================================
void framebuffer_size_callback(GLFWwindow*, int w, int h) { glViewport(0, 0, w, h); }
void scroll_callback(GLFWwindow*, double, double y) { camera.ProcessMouseScroll((float)y); }

void key_callback(GLFWwindow* win, int key, int, int action, int)
{
    if (action == GLFW_PRESS) {
        switch (key) {
            // ---- Lighting ----
        case GLFW_KEY_1: dirLightOn = !dirLightOn;    break;
        case GLFW_KEY_2: pointLightsOn = !pointLightsOn; break;
        case GLFW_KEY_3: spotLightOn = !spotLightOn;   break;
        case GLFW_KEY_5: ambientOn = !ambientOn;     break;
        case GLFW_KEY_6: diffuseOn = !diffuseOn;     break;
        case GLFW_KEY_7: specularOn = !specularOn;    break;
        case GLFW_KEY_8:
            emissiveOn = !emissiveOn;
            cout << "Emissive glow: " << (emissiveOn ? "ON" : "OFF") << "\n";
            break;
        case GLFW_KEY_L: masterLightOn = !masterLightOn; break;

            // ---- Viewport toggle ----
        case GLFW_KEY_4:
            viewportMode = (viewportMode == 0) ? 1 : 0;
            cout << "Viewport mode: " << (viewportMode == 0 ? "Full screen" : "4 quadrants") << "\n";
            break;

            // ---- Shutter ----
        case GLFW_KEY_T:
            if (shutterOpenProgress < 0.05f) {
                shutterOpening = true;  shutterClosing = false;
                cout << "Shutter: OPENING\n";
            }
            else {
                shutterClosing = true;  shutterOpening = false;
                cout << "Shutter: CLOSING\n";
            }
            break;

            // ---- 4-post lifts ----
        case GLFW_KEY_J:
            if (liftH[0] < 0.1f || liftGoingDn[0]) { liftGoingUp[0] = true; liftGoingDn[0] = false; }
            else { liftGoingDn[0] = true; liftGoingUp[0] = false; }
            cout << "Lift 1 (4-post): " << (liftGoingUp[0] ? "RISING" : "LOWERING") << "\n";
            break;
        case GLFW_KEY_K:
            if (liftH[1] < 0.1f || liftGoingDn[1]) { liftGoingUp[1] = true; liftGoingDn[1] = false; }
            else { liftGoingDn[1] = true; liftGoingUp[1] = false; }
            cout << "Lift 2 (4-post): " << (liftGoingUp[1] ? "RISING" : "LOWERING") << "\n";
            break;
        case GLFW_KEY_M:
            if (liftH[4] < 0.1f || liftGoingDn[4]) { liftGoingUp[4] = true; liftGoingDn[4] = false; }
            else { liftGoingDn[4] = true; liftGoingUp[4] = false; }
            cout << "Lift 5 (4-post NEW): " << (liftGoingUp[4] ? "RISING" : "LOWERING") << "\n";
            break;

            // ---- Scissor lifts ----
        case GLFW_KEY_V:
            if (liftH[2] < 0.1f || liftGoingDn[2]) { liftGoingUp[2] = true; liftGoingDn[2] = false; }
            else { liftGoingDn[2] = true; liftGoingUp[2] = false; }
            cout << "Lift 3 (scissor): " << (liftGoingUp[2] ? "RISING" : "LOWERING") << "\n";
            break;
        case GLFW_KEY_N:
            if (liftH[3] < 0.1f || liftGoingDn[3]) { liftGoingUp[3] = true; liftGoingDn[3] = false; }
            else { liftGoingDn[3] = true; liftGoingUp[3] = false; }
            cout << "Lift 4 (scissor): " << (liftGoingUp[3] ? "RISING" : "LOWERING") << "\n";
            break;

            // ---- Section equipment controls ----
        case GLFW_KEY_C:
            conveyorOn = !conveyorOn;
            cout << "Conveyor belt: " << (conveyorOn ? "ON" : "OFF") << "\n";
            break;
        case GLFW_KEY_G:
            fanRotating = !fanRotating;
            cout << "Ceiling fans: " << (fanRotating ? "ON" : "OFF") << "\n";
            break;
        case GLFW_KEY_O:
            robotOn = !robotOn;
            cout << "Robotic arms: " << (robotOn ? "ON" : "OFF") << "\n";
            break;
        case GLFW_KEY_B:
            sprayOn = !sprayOn;
            cout << "Paint spray / turntable: " << (sprayOn ? "ON" : "OFF") << "\n";
            break;
        case GLFW_KEY_H:
            pressActive = !pressActive;
            cout << "Hydraulic press: " << (pressActive ? "LOWERING" : "RAISING") << "\n";
            break;
        case GLFW_KEY_I:
            paintDoorOpen = !paintDoorOpen;
            cout << "Paint shop door: " << (paintDoorOpen ? "OPENING" : "CLOSING") << "\n";
            break;

            // ---- Bird's Eye View ----
        case GLFW_KEY_0:
            birdEyeView = !birdEyeView;
            cout << "Bird's Eye View: " << (birdEyeView ? "ON" : "OFF") << "\n";
            break;

            // ---- Drive mode toggle ----
        case GLFW_KEY_P:
            driveMode = !driveMode;
            driveSpeed = 0.0f;
            cout << "Drive mode: " << (driveMode ? "ON" : "OFF") << "\n";
            break;
        }
    }

    // Drive key tracking (continuous)
    bool pressed = (action == GLFW_PRESS || action == GLFW_REPEAT);
    bool released = (action == GLFW_RELEASE);
    if (driveMode) {
        if (key == GLFW_KEY_UP) { driveKeyUp = pressed; if (released) driveKeyUp = false; }
        if (key == GLFW_KEY_DOWN) { driveKeyDn = pressed; if (released) driveKeyDn = false; }
        if (key == GLFW_KEY_LEFT) { driveKeyLt = pressed; if (released) driveKeyLt = false; }
        if (key == GLFW_KEY_RIGHT) { driveKeyRt = pressed; if (released) driveKeyRt = false; }
        if (key == GLFW_KEY_SPACE) driveHandbrake = (action != GLFW_RELEASE);
    }
}

void processInput(GLFWwindow* win)
{
    if (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(win, true);

    // In drive mode, arrow keys are consumed by car; skip camera
    if (driveMode) return;

    float rs = camera.RotSpeed * deltaTime;
    if (glfwGetKey(win, GLFW_KEY_W) == GLFW_PRESS) camera.moveForward(deltaTime);
    if (glfwGetKey(win, GLFW_KEY_S) == GLFW_PRESS) camera.moveBackward(deltaTime);
    if (glfwGetKey(win, GLFW_KEY_A) == GLFW_PRESS) camera.moveLeft(deltaTime);
    if (glfwGetKey(win, GLFW_KEY_D) == GLFW_PRESS) camera.moveRight(deltaTime);
    if (glfwGetKey(win, GLFW_KEY_E) == GLFW_PRESS) camera.moveUp(deltaTime);
    if (glfwGetKey(win, GLFW_KEY_R) == GLFW_PRESS) camera.moveDown(deltaTime);
    if (glfwGetKey(win, GLFW_KEY_UP) == GLFW_PRESS) camera.changePitch(rs);
    if (glfwGetKey(win, GLFW_KEY_DOWN) == GLFW_PRESS) camera.changePitch(-rs);
    if (glfwGetKey(win, GLFW_KEY_LEFT) == GLFW_PRESS) camera.changeYaw(-rs);
    if (glfwGetKey(win, GLFW_KEY_RIGHT) == GLFW_PRESS) camera.changeYaw(rs);
    if (glfwGetKey(win, GLFW_KEY_X) == GLFW_PRESS) camera.changePitch(rs);
    if (glfwGetKey(win, GLFW_KEY_Y) == GLFW_PRESS) camera.changeYaw(rs);
    if (glfwGetKey(win, GLFW_KEY_Z) == GLFW_PRESS) camera.changeRoll(rs);
    if (glfwGetKey(win, GLFW_KEY_F) == GLFW_PRESS) camera.orbitAroundTarget(rs);
}

// ============================================================
//  resolveCircleAABB
//  Pushes (cx,cz) out of box [x1,z1,x2,z2] if within radius r.
//  Returns true if a collision was resolved.
// ============================================================
static bool resolveCircleAABB(float& cx, float& cz, float r,
    float x1, float z1, float x2, float z2)
{
    float nearX = fmaxf(x1, fminf(cx, x2));
    float nearZ = fmaxf(z1, fminf(cz, z2));
    float dx = cx - nearX, dz = cz - nearZ;
    float dist2 = dx * dx + dz * dz;
    if (dist2 >= r * r) return false;

    if (dist2 < 1e-6f) {
        // Centre is inside box — push out along shortest axis
        float dL = cx - x1, dR = x2 - cx, dF = cz - z1, dB = z2 - cz;
        float mn = fminf(fminf(dL, dR), fminf(dF, dB));
        if      (mn == dL) cx = x1 - r;
        else if (mn == dR) cx = x2 + r;
        else if (mn == dF) cz = z1 - r;
        else               cz = z2 + r;
    } else {
        float dist = sqrtf(dist2);
        cx += (dx / dist) * (r - dist);
        cz += (dz / dist) * (r - dist);
    }
    return true;
}

// ============================================================
//  updateDriveCar  – arcade physics for road car
// ============================================================
void updateDriveCar()
{
    if (!driveMode) return;

    // Steering – only when car is actually moving
    if (fabsf(driveSpeed) > 0.3f) {
        float steerDir = (driveSpeed > 0) ? 1.0f : -1.0f;
        if (driveKeyLt) driveYaw += DRIVE_STEER_SPD * deltaTime * steerDir;
        if (driveKeyRt) driveYaw -= DRIVE_STEER_SPD * deltaTime * steerDir;
    }

    // ── Fuel depletion (only while moving) ───────────────────────────────────
    if (fabsf(driveSpeed) > 0.4f) {
        fuelLevel -= FUEL_BURN_RATE * deltaTime;
        if (fuelLevel < 0.f) fuelLevel = 0.f;
    }

    // ── Block acceleration when out of fuel ───────────────────────────────────
    float targetSpeed = driveSpeed;
    if (fuelLevel <= 0.f) {
        // Coast to a stop – no engine power
        if (driveKeyUp && targetSpeed < 0.f) { /* allow reversing out */ }
        else if (driveKeyUp) driveKeyUp = false; // ignore throttle
        targetSpeed -= glm::sign(targetSpeed) * DRIVE_FRICTION * 2.f * deltaTime;
        if (fabsf(targetSpeed) < 0.05f) targetSpeed = 0.f;
    } else {
        // Normal acceleration / braking
        if (driveKeyUp) {
            targetSpeed += DRIVE_ACCEL * deltaTime;
            if (targetSpeed > DRIVE_MAX_SPD) targetSpeed = DRIVE_MAX_SPD;
        } else if (driveKeyDn) {
            targetSpeed -= DRIVE_BRAKE * deltaTime;
            if (targetSpeed < -DRIVE_MAX_SPD * 0.4f) targetSpeed = -DRIVE_MAX_SPD * 0.4f;
        }
        if (driveHandbrake) {
            targetSpeed *= 0.92f;
        } else if (!driveKeyUp && !driveKeyDn) {
            targetSpeed -= glm::sign(targetSpeed) * DRIVE_FRICTION * deltaTime;
            if (fabsf(targetSpeed) < 0.05f) targetSpeed = 0.f;
        }
    }
    driveSpeed = targetSpeed;

    float preHitSpeed = fabsf(driveSpeed); // speed before collision resolution
    bool  hitThisFrame = false;            // true if any wall/object was hit

    // ── Move car ──────────────────────────────────────────────────────────────
    float rad = glm::radians(driveYaw);
    float newX = drivePosX + cosf(rad) * driveSpeed * deltaTime;
    float newZ = drivePosZ - sinf(rad) * driveSpeed * deltaTime;

    // ── Static AABB collisions ────────────────────────────────────────────────
    const float CAR_R = 2.2f;
    for (auto& b : g_collBoxes) {
        if (resolveCircleAABB(newX, newZ, CAR_R, b[0], b[1], b[2], b[3])) {
            driveSpeed *= -0.15f;
            hitThisFrame = true;
        }
    }

    // ── Building walls ────────────────────────────────────────────────────────
    const float BLDG_X1 = 0.0f,   BLDG_X2 = 200.0f, BLDG_Z2 = 48.0f;
    const float DOOR_XL = 7.0f,   DOOR_XR  = 193.0f;
    const float CAR_HW  = 1.1f,   CAR_HL   = 2.3f;

    bool inBldgX  = (newX > BLDG_X1 + CAR_HL && newX < BLDG_X2 - CAR_HL);
    bool inDoorX  = (newX > DOOR_XL + CAR_HW  && newX < DOOR_XR  - CAR_HW);
    bool doorOpen = (shutterOpenProgress > 0.45f);

    // Front wall (outside → inside)
    if (inBldgX && newZ < BLDG_Z2 && drivePosZ >= BLDG_Z2) {
        if (!inDoorX || !doorOpen) {
            newZ = BLDG_Z2; driveSpeed *= -0.20f; hitThisFrame = true;
        }
    }
    // Interior boundary walls
    if (newZ < BLDG_Z2 && newX > BLDG_X1 && newX < BLDG_X2) {
        if (newZ < 1.0f)            { newZ = 1.0f;            driveSpeed *= -0.20f; hitThisFrame = true; }
        if (newX < BLDG_X1 + 1.0f) { newX = BLDG_X1 + 1.0f; driveSpeed *= -0.20f; hitThisFrame = true; }
        if (newX > BLDG_X2 - 1.0f) { newX = BLDG_X2 - 1.0f; driveSpeed *= -0.20f; hitThisFrame = true; }
    }
    // Side walls near entrance (prevent clipping)
    if (newZ >= BLDG_Z2 - 0.5f && newZ < BLDG_Z2 + 1.0f) {
        if (newX < BLDG_X1 + 0.3f && drivePosX >= BLDG_X1 + 0.3f)
            { newX = BLDG_X1 + 0.3f; driveSpeed *= -0.20f; hitThisFrame = true; }
        if (newX > BLDG_X2 - 0.3f && drivePosX <= BLDG_X2 - 0.3f)
            { newX = BLDG_X2 - 0.3f; driveSpeed *= -0.20f; hitThisFrame = true; }
    }

    // ── World map hard limits ─────────────────────────────────────────────────
    const float MAP_XMIN = -80.0f, MAP_XMAX = 280.0f;
    const float MAP_ZMAX =  85.0f, MAP_ZMIN = -35.0f;
    if (newX < MAP_XMIN) { newX = MAP_XMIN; driveSpeed *= -0.20f; hitThisFrame = true; }
    if (newX > MAP_XMAX) { newX = MAP_XMAX; driveSpeed *= -0.20f; hitThisFrame = true; }
    if (newZ < MAP_ZMIN) { newZ = MAP_ZMIN; driveSpeed *= -0.20f; hitThisFrame = true; }
    if (newZ > MAP_ZMAX) { newZ = MAP_ZMAX; driveSpeed *= -0.20f; hitThisFrame = true; }

    // ── Apply damage (5% per impact, only if moving fast enough) ─────────────
    if (hitThisFrame && preHitSpeed > 2.0f) {
        damageLevel = (damageLevel + 0.05f < 1.0f) ? damageLevel + 0.05f : 1.0f;
    }

    drivePosX = newX;
    drivePosZ = newZ;

    // Wheel rotation based on speed
    driveWheelRot += driveSpeed * deltaTime * 1.8f;
}

// ============================================================
//  loadTexture
// ============================================================
unsigned int loadTexture(const char* path)
{
    unsigned int id;
    glGenTextures(1, &id);
    int w, h, nc;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* data = stbi_load(path, &w, &h, &nc, 0);
    if (data) {
        GLenum fmt = (nc == 4) ? GL_RGBA : (nc == 3 ? GL_RGB : GL_RED);
        glBindTexture(GL_TEXTURE_2D, id);
        glTexImage2D(GL_TEXTURE_2D, 0, fmt, w, h, 0, fmt, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
    else {
        unsigned char px[3] = { 200,200,200 };
        glBindTexture(GL_TEXTURE_2D, id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, px);
    }
    stbi_image_free(data);
    return id;
}

// ============================================================
//  MAIN
// ============================================================
int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* win = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "CarWorkStation", nullptr, nullptr);
    if (!win) { cerr << "Window failed\n"; glfwTerminate(); return -1; }
    glfwMakeContextCurrent(win);
    glfwSetFramebufferSizeCallback(win, framebuffer_size_callback);
    glfwSetScrollCallback(win, scroll_callback);
    glfwSetKeyCallback(win, key_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { cerr << "GLAD failed\n"; return -1; }
    glEnable(GL_DEPTH_TEST);

    Shader sh("shaders/vertexShaderForPhongShadingWithTexture.vs",
        "shaders/fragmentShaderForPhongShadingWithTexture.fs");
    Shader skySh("shaders/skybox.vs", "shaders/skybox.fs");

    // ---- Skybox VAO/VBO  (LearnOpenGL canonical winding – CCW from inside) ----
    {
        static const float sv[] = {
            // -Z face (back)
            -1.0f,  1.0f, -1.0f,  -1.0f, -1.0f, -1.0f,   1.0f, -1.0f, -1.0f,
             1.0f, -1.0f, -1.0f,   1.0f,  1.0f, -1.0f,  -1.0f,  1.0f, -1.0f,
            // -X face (left)
            -1.0f, -1.0f,  1.0f,  -1.0f, -1.0f, -1.0f,  -1.0f,  1.0f, -1.0f,
            -1.0f,  1.0f, -1.0f,  -1.0f,  1.0f,  1.0f,  -1.0f, -1.0f,  1.0f,
            // +X face (right)
             1.0f, -1.0f, -1.0f,   1.0f, -1.0f,  1.0f,   1.0f,  1.0f,  1.0f,
             1.0f,  1.0f,  1.0f,   1.0f,  1.0f, -1.0f,   1.0f, -1.0f, -1.0f,
            // +Z face (front)
            -1.0f, -1.0f,  1.0f,  -1.0f,  1.0f,  1.0f,   1.0f,  1.0f,  1.0f,
             1.0f,  1.0f,  1.0f,   1.0f, -1.0f,  1.0f,  -1.0f, -1.0f,  1.0f,
            // +Y face (top)
            -1.0f,  1.0f, -1.0f,   1.0f,  1.0f, -1.0f,   1.0f,  1.0f,  1.0f,
             1.0f,  1.0f,  1.0f,  -1.0f,  1.0f,  1.0f,  -1.0f,  1.0f, -1.0f,
            // -Y face (bottom)
            -1.0f, -1.0f, -1.0f,  -1.0f, -1.0f,  1.0f,   1.0f, -1.0f, -1.0f,
             1.0f, -1.0f, -1.0f,  -1.0f, -1.0f,  1.0f,   1.0f, -1.0f,  1.0f,
        };
        glGenVertexArrays(1, &skyboxVAO);
        glGenBuffers(1, &skyboxVBO);
        glBindVertexArray(skyboxVAO);
        glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(sv), sv, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glBindVertexArray(0);
    }

    // ---- Skybox cubemap texture (uses texture unit 1) ----
    {
        const char* faces[6] = {
            "images/skybox/right.jpg",
            "images/skybox/left.jpg",
            "images/skybox/top.jpg",
            "images/skybox/bottom.jpg",
            "images/skybox/front.jpg",
            "images/skybox/back.jpg"
        };
        glGenTextures(1, &skyboxTex);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxTex);
        stbi_set_flip_vertically_on_load(false); // cubemaps must NOT be flipped
        int loadedFaces = 0;
        for (int i = 0; i < 6; i++) {
            int w, h, ch;
            unsigned char* data = stbi_load(faces[i], &w, &h, &ch, 0);
            if (data) {
                GLenum fmt = (ch == 4) ? GL_RGBA : GL_RGB;
                glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                    0, fmt, w, h, 0, fmt, GL_UNSIGNED_BYTE, data);
                stbi_image_free(data);
                ++loadedFaces;
                cout << "Skybox face " << i << " loaded: " << faces[i]
                     << " (" << w << "x" << h << ")\n";
            } else {
                cerr << "Skybox face FAILED: " << faces[i]
                     << " – " << stbi_failure_reason() << "\n";
            }
        }
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
        stbi_set_flip_vertically_on_load(true); // restore for all subsequent loads
        glActiveTexture(GL_TEXTURE0);           // restore active unit to 0

        // Bind skybox sampler to unit 1 once
        skySh.use();
        skySh.setInt("skybox", 1);
        cout << "Skybox: " << loadedFaces << "/6 faces loaded (tex ID=" << skyboxTex << ")\n";
    }

    // ---- Textures ----
    texRoad = loadTexture("images/road.jpg");
    texGrass = loadTexture("images/grass.jpg");
    texBrick = loadTexture("images/brick.jpg");
    texConcrete = loadTexture("images/concrete.jpg");
    texMetal = loadTexture("images/metal.jpg");
    texTire = loadTexture("images/tire.jpg");
    texBark = loadTexture("images/bark.jpg");
    texLeaf = loadTexture("images/leaf.jpg");
    texPot = loadTexture("images/pot.jpg");
    texWhite = loadTexture("images/white.jpg");
    texOrange = loadTexture("images/orange.jpg");
    texRed = loadTexture("images/red.jpg");
    texWindow = loadTexture("images/window.jpg");
    texRoof = loadTexture("images/roof.jpg");
    texLaneMark = loadTexture("images/lanemark.jpg");
    texWood = loadTexture("images/wood.jpg");
    texDarkFloor = loadTexture("images/darkfloor.jpg");
    texStripe = loadTexture("images/stripe.jpg");
    texColor    = loadTexture("images/color.jpg");
    texAsphalt  = loadTexture("images/road.jpg");
    texSidewalk = loadTexture("images/concrete.jpg");
    texGlassBlue = loadTexture("images/window.jpg");

    Cube cube;
    Sphere sphere;

    // ---- Bezier surfaces ----
    BezierSurface potSurf(
        { {0.20f,0},{0.42f,0.1f},{0.38f,0.4f},{0.32f,0.6f},{0.38f,0.75f},{0.45f,0.90f} },
        { 0.55f,0.25f,0.10f }, { 0.65f,0.30f,0.12f }, { 0.3f,0.1f,0.05f }, 16, texPot);
    BezierSurface trunkSurf(
        { {0.10f,0},{0.10f,0.3f},{0.09f,0.7f},{0.08f,1.1f} },
        { 0.3f,0.18f,0.08f }, { 0.38f,0.22f,0.10f }, { 0.1f,0.1f,0.1f }, 8, texBark);
    BezierSurface canopySurf(
        { {0.0f,0},{0.65f,0.1f},{0.90f,0.45f},{0.85f,0.85f},{0.55f,1.1f},{0.0f,1.2f} },
        { 0.1f,0.4f,0.1f }, { 0.15f,0.55f,0.15f }, { 0.05f,0.2f,0.05f }, 16, texLeaf);
    BezierSurface bushSurf(
        { {0.0f,0},{0.5f,0.05f},{0.65f,0.35f},{0.5f,0.62f},{0.0f,0.7f} },
        { 0.08f,0.38f,0.08f }, { 0.10f,0.5f,0.10f }, { 0.04f,0.1f,0.04f }, 16, texLeaf);

    // ---- Bezier concept car (3 surfaces of revolution, no texture) ----------
    // Body: smooth egg profile (r,y) – rotated around Y gives a compact car body
    BezierSurface bezCarBody(
        { {0.0f,0.0f},{0.55f,0.04f},{1.0f,0.22f},{1.1f,0.50f},
          {0.9f,0.78f},{0.5f,0.96f},{0.0f,1.0f} },
        {0.22f,0.02f,0.02f}, {0.72f,0.06f,0.05f}, {0.90f,0.55f,0.45f}, 96.f, 0);
    // Cabin: dome profile for windshield + roof (glass tinted dark blue)
    BezierSurface bezCarCabin(
        { {0.0f,0.0f},{0.4f,0.05f},{0.88f,0.28f},{0.72f,0.62f},{0.0f,0.85f} },
        {0.04f,0.07f,0.14f}, {0.09f,0.16f,0.36f}, {0.85f,0.90f,1.0f}, 128.f, 0);
    // Wheel: C-section torus profile – stands upright when rotated 90° around Z
    BezierSurface bezCarWheel(
        { {0.52f,0.0f},{0.76f,0.09f},{0.83f,0.22f},{0.76f,0.35f},{0.52f,0.44f} },
        {0.05f,0.05f,0.05f}, {0.13f,0.13f,0.13f}, {0.22f,0.22f,0.22f}, 20.f, 0);

    // ---- Scene constants (new 4-section complex) ----
    const glm::vec3 BLDG(0.0f, 0.0f, 0.0f);
    const float ROAD_Z = 65.0f, BW = 200.0f, BD = 48.0f;
    const float bayX[5] = { 25.0f, 75.0f, 100.0f, 125.0f, 175.0f }; // kept for spotPos
    const float bayZ = 24.0f;
    const glm::vec3 spotPos(bayX[3], 11.0f, bayZ);
    const glm::vec3 bldgCenter(BW / 2, 0.0f, BD / 2);

    // ---- Collision boxes {x1,z1,x2,z2} ----
    g_collBoxes.clear();
    auto addBox = [&](float x1, float z1, float x2, float z2, float pad = 0.2f) {
        g_collBoxes.push_back({ x1-pad, z1-pad, x2+pad, z2+pad });
    };
    auto addCarX = [&](float cx, float cz, float hl=2.3f, float hw=1.1f) {
        addBox(cx-hl, cz-hw, cx+hl, cz+hw);
    };
    auto addCarZ = [&](float cx, float cz, float hl=2.3f, float hw=1.1f) {
        addBox(cx-hw, cz-hl, cx+hw, cz+hl);
    };

    // ── Workshop building exterior walls ─────────────────────────────────────
    addBox(-0.5f,-0.5f, BW+0.5f, 0.3f);             // back wall  (Z=0)
    addBox(-0.5f,-0.5f,  0.3f, BD+0.5f);             // left wall  (X=0)
    addBox(BW-0.3f,-0.5f, BW+0.5f, BD+0.5f);         // right wall (X=BW)
    addBox(-0.5f, BD-0.3f, 7.5f, BD+0.5f);           // front-left pillar
    addBox(BW-7.5f,BD-0.3f, BW+0.5f, BD+0.5f);       // front-right pillar

    // ── Partition walls at X=50, 100, 150 (passage gap Z=21..27) ─────────────
    for (float px : {50.f, 100.f, 150.f}) {
        addBox(px-0.3f, -0.3f, px+0.3f, 21.f);       // solid front half
        addBox(px-0.3f, 27.f,  px+0.3f, BD+0.3f);    // solid rear half
    }

    // ── Parking lot cars (drawn at parkZ = BD+4, centre z = BD+5) ────────────
    {
        float pz = BD + 5.0f;
        addCarX(BLDG.x+ 15.f, BLDG.z+pz);
        addCarX(BLDG.x+ 35.f, BLDG.z+pz);
        addCarX(BLDG.x+ 55.f, BLDG.z+pz);
        addCarX(BLDG.x+ 80.f, BLDG.z+pz);
        addCarX(BLDG.x+100.f, BLDG.z+pz);
        addCarX(BLDG.x+120.f, BLDG.z+pz);
        addCarX(BLDG.x+145.f, BLDG.z+pz);
        addCarX(BLDG.x+165.f, BLDG.z+pz);
    }

    // ── Road stationary cars ─────────────────────────────────────────────────
    addCarZ(-18.f, ROAD_Z-2.5f);   // west of building, facing east
    addCarZ(220.f, ROAD_Z+2.5f);   // east of building, facing west

    // ── 6 fractal trees (exact positions from drawCityBlock) ─────────────────
    // T1 pine   (-18, ROAD_Z-11)
    addBox(-18.f-1.8f, (ROAD_Z-11.f)-1.8f, -18.f+1.8f, (ROAD_Z-11.f)+1.8f, 0.f);
    // T2 oak    (162, ROAD_Z-11)
    addBox(162.f-1.8f, (ROAD_Z-11.f)-1.8f, 162.f+1.8f, (ROAD_Z-11.f)+1.8f, 0.f);
    // T3 palm   (-28, 8) – near gas station
    addBox(-28.f-1.8f,  8.f-1.8f, -28.f+1.8f,  8.f+1.8f, 0.f);
    // T4 oak    (88, -56) – north of back road
    addBox( 88.f-1.8f,-56.f-1.8f,  88.f+1.8f,-56.f+1.8f, 0.f);
    // T5 shrub  (238, 48) – east side
    addBox(238.f-1.8f, 48.f-1.8f, 238.f+1.8f, 48.f+1.8f, 0.f);
    // T6 sapling (196, ROAD_Z+13) – south of front road
    addBox(196.f-1.8f, (ROAD_Z+13.f)-1.8f, 196.f+1.8f, (ROAD_Z+13.f)+1.8f, 0.f);

    // ── 10 city buildings (match cityBldg() calls in drawCityBlock) ───────────
    // South-of-road trio
    addBox( 20.f- 8.f, ROAD_Z+25.f- 5.5f,  20.f+ 8.f, ROAD_Z+25.f+ 5.5f); // 1 orange
    addBox( 82.f- 7.f, ROAD_Z+23.f- 5.0f,  82.f+ 7.f, ROAD_Z+23.f+ 5.0f); // 2 blue
    addBox(158.f- 9.f, ROAD_Z+24.f- 6.0f, 158.f+ 9.f, ROAD_Z+24.f+ 6.0f); // 3 grey
    // North of back road
    addBox( 55.f-10.f, -52.f- 6.5f,  55.f+10.f, -52.f+ 6.5f);              // 4
    addBox(145.f- 8.f, -48.f- 5.5f, 145.f+ 8.f, -48.f+ 5.5f);              // 5 tall
    // West side
    addBox(-82.f- 7.f,  30.f- 5.0f, -82.f+ 7.f,  30.f+ 5.0f);              // 6
    addBox(-90.f- 8.f,  68.f- 5.0f, -90.f+ 8.f,  68.f+ 5.0f);              // 7 tall
    // East side
    addBox(268.f- 7.f,  18.f- 5.0f, 268.f+ 7.f,  18.f+ 5.0f);              // 8
    addBox(268.f- 8.f,  58.f- 5.5f, 268.f+ 8.f,  58.f+ 5.5f);              // 9 tall
    addBox(268.f- 6.f,  96.f- 4.5f, 268.f+ 6.f,  96.f+ 4.5f);              // 10 low

    // ── Gas station (drawn at pos = {-38, 0, 22}) ────────────────────────────
    // Kiosk shop: centre (-47.5, _, 15.5), footprint 7 × 5.5
    addBox(-47.5f-3.5f, 15.5f-2.75f, -47.5f+3.5f, 15.5f+2.75f);
    // Fuel dispensers (2): grouped footprint
    addBox(-41.2f, 22.8f, -34.8f, 24.2f);
    // Price-sign pole: (-27, _, 14), thin 0.22 × 0.22
    addBox(-27.2f, 13.8f, -26.8f, 14.2f, 0.f);

    // ── Lamp-post poles (7 poles, 0.18 × 7 × 0.18) ───────────────────────────
    // Front road (south side poles at Z = ROAD_Z + 8.5)
    for (float px : {-28.f, 80.f, 170.f})
        addBox(px-0.09f, ROAD_Z+8.41f, px+0.09f, ROAD_Z+8.59f, 0.f);
    // Back road (south side poles at Z = -22 + 7.5 = -14.5)
    for (float px : {50.f, 150.f})
        addBox(px-0.09f, -14.59f, px+0.09f, -14.41f, 0.f);
    // West N-S road (pole at X = RXW - 8.5 = -53.5, Z = 24)
    addBox(-53.59f, 23.91f, -53.41f, 24.09f, 0.f);
    // East N-S road (pole at X = RXE + 8.5 = 253.5, Z = 24)
    addBox(253.41f, 23.91f, 253.59f, 24.09f, 0.f);

    // ── Fractal branch tree trunk at (8, 0, 57) ───────────────────────────────
    addBox(8.f-1.5f, 57.f-1.5f, 8.f+1.5f, 57.f+1.5f, 0.f);

    // ── Bezier concept car display pedestal at (192, 0, 55.5) ─────────────────
    addBox(192.f-4.5f, 55.5f-3.25f, 192.f+4.5f, 55.5f+3.25f);

    // ---- Point lights (12 total: 4 building + 3 front road + 2 back road + 2 side road + 1 exterior) ----
    PointLight lights[12] = {
        // ---- Building interior ----
        PointLight(BLDG.x+25,  10.f,BLDG.z+24, .28f,.26f,.20f, .85f,.82f,.75f, .9f,.9f,.85f, 1,.038f,.006f,  1), // Assembly
        PointLight(BLDG.x+75,  10.f,BLDG.z+24, .22f,.22f,.26f, .80f,.82f,.88f, .9f,.9f,.95f, 1,.038f,.006f,  2), // Paint shop
        PointLight(BLDG.x+125, 10.f,BLDG.z+24, .28f,.25f,.20f, .82f,.80f,.72f, .9f,.9f,.85f, 1,.038f,.006f,  3), // Body shop
        PointLight(BLDG.x+175, 10.f,BLDG.z+24, .25f,.25f,.28f, .90f,.90f,.95f, .95f,.95f,1.f, 1,.038f,.006f,  4), // Showroom
        // ---- Front road (3 lamps, matching lamp-post globe positions) ----
        PointLight(-28.f,  7.f, ROAD_Z,         .18f,.15f,.08f, .68f,.62f,.42f, .7f,.7f,.60f, 1,.045f,.010f,  5),
        PointLight( 80.f,  7.f, ROAD_Z,         .18f,.15f,.08f, .68f,.62f,.42f, .7f,.7f,.60f, 1,.045f,.010f,  6),
        PointLight(170.f,  7.f, ROAD_Z,         .18f,.15f,.08f, .64f,.58f,.38f, .7f,.7f,.60f, 1,.045f,.010f,  7),
        // ---- Back road (2 lamps) ----
        PointLight( 50.f,  7.f, -22.f,          .15f,.13f,.07f, .58f,.54f,.34f, .6f,.6f,.52f, 1,.050f,.012f,  8),
        PointLight(150.f,  7.f, -22.f,          .15f,.13f,.07f, .58f,.54f,.34f, .6f,.6f,.52f, 1,.050f,.012f,  9),
        // ---- West cross road ----
        PointLight(-45.f,  7.f,  24.f,          .15f,.13f,.07f, .58f,.54f,.34f, .6f,.6f,.52f, 1,.050f,.012f, 10),
        // ---- East cross road ----
        PointLight(245.f,  7.f,  24.f,          .15f,.13f,.07f, .58f,.54f,.34f, .6f,.6f,.52f, 1,.050f,.012f, 11),
        // ---- Building exterior ----
        PointLight(BLDG.x+100, 6.f,BLDG.z+55,  .15f,.15f,.12f, .55f,.55f,.50f, .6f,.6f,.55f, 1,.050f,.010f, 12),
    };

    // ---- Fixed view matrices for quadrant viewports ----
    glm::mat4 topView = glm::lookAt(
        bldgCenter + glm::vec3(0, 90, 0), bldgCenter, glm::vec3(0, 0, -1));
    glm::mat4 frontView = glm::lookAt(
        bldgCenter + glm::vec3(0, 10, 100), bldgCenter + glm::vec3(0, 5, 0), glm::vec3(0, 1, 0));
    glm::mat4 isoView = glm::lookAt(
        bldgCenter + glm::vec3(-60, 50, -40), bldgCenter, glm::vec3(0, 1, 0));

    // ---- Print controls ----
    cout << "\n===== CarWorkStation Controls =====\n";
    cout << "CAMERA:        W/S/A/D/E/R move | Arrows/X/Y/Z look/rotate\n";
    cout << "               F = orbit building | 0 = Bird's Eye View | Scroll = zoom\n";
    cout << "LIGHTING:      1=dir | 2=points | 3=spot(single cutoff) | L=master\n";
    cout << "               5=ambient | 6=diffuse | 7=specular\n";
    cout << "VIEWPORT:      4 = Full / 4-quadrant toggle\n";
    cout << "DOORS:         T = main shutter | I = paint shop booth door\n";
    cout << "LIFTS:         J/K = 4-post lifts 1/2 | M = 4-post lift 3 (NEW)\n";
    cout << "               V/N = scissor lifts 1/2\n";
    cout << "EQUIPMENT:     C=conveyor | O=robot arms | B=spray+turntable\n";
    cout << "               H=press | G=ceiling fans\n";
    cout << "DRIVE:         P = enter/exit | Arrows = steer/accel | SPACE = brake\n";
    cout << "               (sounds trigger automatically per section)\n";
    cout << "===================================\n\n";

    // ---- Sound system ----
    g_sound.init();

    // ============================================================
    //  Main loop
    // ============================================================
    while (!glfwWindowShouldClose(win))
    {
        float now = (float)glfwGetTime();
        deltaTime = now - lastFrame; lastFrame = now;

        processInput(win);

        // ── Shutter animation ──────────────────────────────────
        if (shutterOpening) {
            shutterOpenProgress += SHUTTER_SPD * deltaTime;
            if (shutterOpenProgress >= 1.0f) {
                shutterOpenProgress = 1.0f;
                shutterOpening = false;
            }
        }
        if (shutterClosing) {
            shutterOpenProgress -= SHUTTER_SPD * deltaTime;
            if (shutterOpenProgress <= 0.0f) {
                shutterOpenProgress = 0.0f;
                shutterClosing = false;
            }
        }

        // ── Lift animation ────────────────────────────────────
        for (int i = 0; i < 5; i++) {
            if (liftGoingUp[i]) {
                liftH[i] += LIFT_SPD * deltaTime;
                if (liftH[i] >= LIFT_MAX) { liftH[i] = LIFT_MAX; liftGoingUp[i] = false; }
            }
            if (liftGoingDn[i]) {
                liftH[i] -= LIFT_SPD * deltaTime;
                if (liftH[i] <= 0) { liftH[i] = 0; liftGoingDn[i] = false; }
            }
        }

        // ── Drive car physics ─────────────────────────────────
        updateDriveCar();

        // ── Ceiling fan (G key toggles) ──────────────────────
        if (fanRotating) {
            fanAngle += 45.0f * deltaTime;
            if (fanAngle > 360.0f) fanAngle -= 360.0f;
        }

        // Section animation updates
        if (conveyorOn) {
            conveyorOff += 4.0f * deltaTime;   // belt moves 4 units/s
            if (conveyorOff > 42.0f) conveyorOff -= 42.0f;
        }
        if (robotOn) {
            robotAngle += 22.0f * deltaTime;   // sweep 22 deg/s
            if (robotAngle > 360.0f) robotAngle -= 360.0f;
        }
        if (sprayOn) {
            sprayAngle += 18.0f * deltaTime;
            if (sprayAngle > 360.0f) sprayAngle -= 360.0f;
            // Paint turntable rotates slowly when spray is active
            paintTurntableAngle += 8.0f * deltaTime;
            if (paintTurntableAngle > 360.0f) paintTurntableAngle -= 360.0f;
        }
        // Hydraulic press animation
        if (pressActive) {
            pressHeight -= 1.8f * deltaTime;
            if (pressHeight < 0.1f) { pressHeight = 0.1f; pressActive = false; }
        } else {
            pressHeight += 1.5f * deltaTime;
            if (pressHeight > 3.0f) pressHeight = 3.0f;
        }
        // Paint shop door slide animation
        if (paintDoorOpen) {
            paintDoorAmt += 1.5f * deltaTime;
            if (paintDoorAmt > 1.0f) paintDoorAmt = 1.0f;
        } else {
            paintDoorAmt -= 1.5f * deltaTime;
            if (paintDoorAmt < 0.0f) paintDoorAmt = 0.0f;
        }

        // ── Section detection for sound ─────────────────────────────────
        {
            SectionID sec = SEC_NONE;
            const float NBD = 48.0f;  // new building depth
            if (drivePosZ < NBD && drivePosX >= 0 && drivePosX < 200.0f) {
                int si = (int)(drivePosX / 50.0f);
                if      (si == 0) sec = SEC_ASSEMBLY;
                else if (si == 1) sec = SEC_PAINT;
                else if (si == 2) sec = SEC_BODYSHOP;
                else              sec = SEC_SHOWROOM;
            }
            if (sec != currentSection) {
                currentSection = sec;
                g_sound.setSection(sec);
            }
        }

        // ── Paint-zone colour change ─────────────────────────
        {
            bool nowIn = (drivePosX >= PZ_X1 && drivePosX <= PZ_X2 &&
                          drivePosZ >= PZ_Z1 && drivePosZ <= PZ_Z2);
            if (nowIn && !driveInPaintZone) {
                // Car just entered zone – cycle to next paint colour
                static const glm::vec3 palette[] = {
                    {0.10f,0.30f,0.82f},  // deep blue
                    {0.08f,0.62f,0.18f},  // forest green
                    {0.80f,0.55f,0.04f},  // golden yellow
                    {0.55f,0.08f,0.58f},  // purple
                    {0.05f,0.58f,0.55f},  // teal
                    {0.80f,0.08f,0.08f},  // original red
                };
                static int pIdx = 0;
                pIdx = (pIdx + 1) % 6;
                driveCarColor = palette[pIdx];
            }
            driveInPaintZone = nowIn;
        }

        // ── Fuel zone (gas station forecourt) ────────────────
        {
            bool nowInFuel = (drivePosX >= FZ_X1 && drivePosX <= FZ_X2 &&
                              drivePosZ >= FZ_Z1 && drivePosZ <= FZ_Z2);
            if (nowInFuel) {
                fuelZoneTimer += deltaTime;
                if (fuelZoneTimer >= FUEL_FILL_SECS) {
                    fuelLevel     = 1.0f;
                    fuelZoneTimer = 0.0f;
                }
            } else {
                fuelZoneTimer = 0.0f;
            }
            inFuelZone = nowInFuel;
        }

        // ── Repair zone (assembly bay) ────────────────────────
        {
            bool nowInRepair = (drivePosX >= RPZX1 && drivePosX <= RPZX2 &&
                                drivePosZ >= RPZZ1 && drivePosZ <= RPZZ2);
            if (nowInRepair && !inRepairZone)
                damageLevel = 0.0f;   // instant repair on entry
            inRepairZone = nowInRepair;
        }

        // ── Render ───────────────────────────────────────────
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        sh.use();
        sh.setVec3("dirLightDir", { -0.3f,-1.0f,-0.2f });
        sh.setVec3("dirLightAmb",  { 0.45f, 0.45f, 0.42f });   // strong ambient – no black shadows
        sh.setVec3("dirLightDiff", { 0.75f, 0.72f, 0.62f });   // warm daylight diffuse
        sh.setVec3("dirLightSpec", { 0.40f, 0.40f, 0.36f });
        sh.setBool("emissive", false);
        sh.setBool("emissiveGlobalOn", emissiveOn);
        setupSpotLight(sh, spotPos, spotLightOn && masterLightOn);
        setupSectionSpotLights(sh, masterLightOn);
        for (int i = 0; i < 12; i++) {
            if (pointLightsOn && masterLightOn) lights[i].turnOn();
            else                               lights[i].turnOff();
            lights[i].setUpPointLight(sh);
        }

        int vpW = SCR_WIDTH;
        int vpH = SCR_HEIGHT;

        if (viewportMode == 0)
        {
            // ── Full screen – single viewport ─────────────────
            glViewport(0, 0, vpW, vpH);
            float aspect = (float)vpW / (float)vpH;

            // In drive mode the camera chases the car;
            // bird's-eye view overrides both; otherwise free-fly camera
            glm::mat4 view;
            glm::vec3 camPos;
            if (birdEyeView) {
                // Top-down orthographic-like view centred on the building
                camPos = glm::vec3(100.0f, 100.0f, 24.0f);
                view   = glm::lookAt(camPos,
                    glm::vec3(100.0f, 0.0f, 24.0f),
                    glm::vec3(0.0f,   0.0f, -1.0f));   // north = up on screen
            }
            else if (driveMode) {
                // Chase camera sits 12 units BEHIND the car
                float rad  = glm::radians(driveYaw);
                float chaseX = drivePosX - cosf(rad) * 12.0f;
                float chaseZ = drivePosZ + sinf(rad) * 12.0f;
                camPos = { chaseX, 4.5f, chaseZ };
                view   = glm::lookAt(camPos,
                    { drivePosX, 1.2f, drivePosZ }, { 0,1,0 });
            }
            else {
                camPos = camera.Position;
                view   = camera.createViewMatrix();
            }
            glm::mat4 proj = glm::perspective(glm::radians(camera.Zoom), aspect, 0.1f, 600.0f);

            sh.use();
            sh.setMat4("view", view);
            sh.setMat4("projection", proj);
            sh.setVec3("viewPos", camPos);
            sh.setBool("ambientOn", ambientOn && masterLightOn);
            sh.setBool("diffuseOn", diffuseOn && masterLightOn);
            sh.setBool("specularOn", specularOn && masterLightOn);
            sh.setBool("dirLightOn", dirLightOn && masterLightOn);
            for (int i = 0; i < 12; i++) {
                if (pointLightsOn && masterLightOn) lights[i].turnOn(); else lights[i].turnOff();
                lights[i].setUpPointLight(sh);
            }
            setupSpotLight(sh, spotPos, spotLightOn && masterLightOn);
            setupSectionSpotLights(sh, masterLightOn);

            renderScene(cube, sphere, sh, potSurf, trunkSurf, canopySurf, bushSurf,
                    bezCarBody, bezCarCabin, bezCarWheel,
                    BLDG, ROAD_Z, BW, BD, bayX, bayZ);
            drawSkybox(skySh, view, proj);   // drawn LAST – fills pixels where depth == 1.0
            drawHUD(cube, sh, vpW, vpH);
        }
        else
        {
            // ── 4 Quadrant viewports ──────────────────────────
            int qW = vpW / 2;
            int qH = vpH / 2;
            float aspect = (float)qW / (float)qH;
            glm::mat4 mainView = camera.createViewMatrix();
            glm::mat4 mainProj = glm::perspective(glm::radians(camera.Zoom), aspect, 0.1f, 600.0f);
            glm::mat4 fixedProj = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 600.0f);

            struct VP {
                int x, y;
                glm::vec3 camPos;
                glm::mat4 view, proj;
                bool ambOn, diffOn, specOn, dirOn, ptOn, spotOn;
            };

            VP vps[4] = {
                // Top-Left: Combined lighting (interactive camera)
                { 0,    qH, camera.Position, mainView, mainProj,
                  ambientOn && masterLightOn, diffuseOn && masterLightOn, specularOn && masterLightOn,
                  dirLightOn && masterLightOn, pointLightsOn && masterLightOn, spotLightOn && masterLightOn },
                  // Top-Right: Ambient only (top-down)
                  { qW,   qH, bldgCenter + glm::vec3(0,60,0), topView, fixedProj,
                    true, false, false, true, true, false },
                    // Bottom-Left: Diffuse only (front view)
                    { 0,    0,  bldgCenter + glm::vec3(0,6,65), frontView, fixedProj,
                      false, true, false, true, true, false },
                      // Bottom-Right: Directional only (isometric)
                      { qW,   0,  bldgCenter + glm::vec3(-35,28,-25), isoView, fixedProj,
                        true, true, true, true, false, false },
            };

            for (int vi = 0; vi < 4; vi++) {
                VP& vp = vps[vi];
                glViewport(vp.x, vp.y, qW, qH);
                sh.setMat4("view", vp.view);
                sh.setMat4("projection", vp.proj);
                sh.setVec3("viewPos", vp.camPos);
                sh.setBool("ambientOn", vp.ambOn);
                sh.setBool("diffuseOn", vp.diffOn);
                sh.setBool("specularOn", vp.specOn);
                sh.setBool("dirLightOn", vp.dirOn);
                for (int i = 0; i < 12; i++) {
                    if (vp.ptOn) lights[i].turnOn(); else lights[i].turnOff();
                    lights[i].setUpPointLight(sh);
                }
                setupSpotLight(sh, spotPos, vp.spotOn);
                setupSectionSpotLights(sh, masterLightOn);
                renderScene(cube, sphere, sh, potSurf, trunkSurf, canopySurf, bushSurf,
                    bezCarBody, bezCarCabin, bezCarWheel,
                    BLDG, ROAD_Z, BW, BD, bayX, bayZ);
                drawSkybox(skySh, vp.view, vp.proj); // drawn LAST per viewport
            }
            // HUD drawn over the full window in quad mode
            glViewport(0, 0, vpW, vpH);
            drawHUD(cube, sh, vpW, vpH);
        }

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    g_sound.stop();
    glfwTerminate();
    return 0;
}