#ifndef basic_camera_h
#define basic_camera_h

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

// ============================================================
//  BasicCamera – 6-DOF fly-simulator camera
//
//  Movement   : W/S forward/back, A/D strafe, E up, R down
//  Look       : Arrow keys pitch/yaw (continuous)
//  Rotations  : X = pitch, Y = yaw, Z = roll  (hold key)
//  Orbit      : F key – orbit around orbitTarget (worldY axis)
// ============================================================
class BasicCamera {
public:
    // Position & orientation (Euler angles in degrees)
    glm::vec3 Position;
    float     Yaw;    // rotation around world-Y  (+CW looking from above)
    float     Pitch;  // rotation around right axis (+up)
    float     Roll;   // rotation around front axis (+CW from behind)
    float     Zoom;   // FOV in degrees

    glm::vec3 WorldUp;
    glm::vec3 orbitTarget;  // target for F-key orbit

    // Derived axes (updated by updateVectors)
    glm::vec3 Front, Right, Up;

    // Legacy aliases kept so existing main.cpp code compiles unchanged
    glm::vec3 eye;
    glm::vec3 lookAt;
    glm::vec3 V;

    float MoveSpeed;  // world-units per second
    float RotSpeed;   // degrees per second

    BasicCamera(float eyeX  = 0,  float eyeY  = 5,  float eyeZ  = 20,
                float lx    = 0,  float ly    = 0,  float lz    = 0,
                glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f))
    {
        Position    = glm::vec3(eyeX, eyeY, eyeZ);
        WorldUp     = up;
        orbitTarget = glm::vec3(lx, ly, lz);

        // Derive initial Yaw/Pitch from look direction
        glm::vec3 dir = glm::normalize(glm::vec3(lx, ly, lz) - Position);
        Yaw   = glm::degrees(atan2f(dir.z, dir.x));
        Pitch = glm::degrees(asinf(glm::clamp(dir.y, -1.0f, 1.0f)));
        Roll  = 0.0f;
        Zoom  = 45.0f;

        MoveSpeed = 10.0f;
        RotSpeed  = 60.0f;

        updateVectors();
    }

    // Recompute Front/Right/Up from Yaw/Pitch/Roll
    void updateVectors()
    {
        // Front from Yaw + Pitch (standard Euler)
        float pitchR = glm::radians(Pitch);
        float yawR   = glm::radians(Yaw);
        glm::vec3 f;
        f.x   = cosf(yawR) * cosf(pitchR);
        f.y   = sinf(pitchR);
        f.z   = sinf(yawR) * cosf(pitchR);
        Front = glm::normalize(f);

        // Right and Up (no roll yet)
        Right = glm::normalize(glm::cross(Front, WorldUp));
        Up    = glm::normalize(glm::cross(Right, Front));

        // Apply roll around Front axis (Rodrigues)
        if (Roll != 0.0f) {
            float r = glm::radians(Roll);
            float c = cosf(r), s = sinf(r);
            glm::vec3 nr =  Right * c + Up * s;
            glm::vec3 nu = -Right * s + Up * c;
            Right = glm::normalize(nr);
            Up    = glm::normalize(nu);
        }

        // Sync legacy aliases
        eye    = Position;
        lookAt = Position + Front;
        V      = Up;
    }

    glm::mat4 createViewMatrix()
    {
        return glm::lookAt(Position, Position + Front, Up);
    }

    void ProcessMouseScroll(float yoffset)
    {
        Zoom -= yoffset;
        Zoom  = glm::clamp(Zoom, 5.0f, 90.0f);
    }

    // ---- Movement ----
    void moveForward (float dt) { Position += Front   * MoveSpeed * dt; updateVectors(); }
    void moveBackward(float dt) { Position -= Front   * MoveSpeed * dt; updateVectors(); }
    void moveLeft    (float dt) { Position -= Right   * MoveSpeed * dt; updateVectors(); }
    void moveRight   (float dt) { Position += Right   * MoveSpeed * dt; updateVectors(); }
    void moveUp      (float dt) { Position += WorldUp * MoveSpeed * dt; updateVectors(); }
    void moveDown    (float dt) { Position -= WorldUp * MoveSpeed * dt; updateVectors(); }

    // ---- Rotation (key X = pitch, Y = yaw, Z = roll) ----
    void changePitch(float deg)
    {
        Pitch += deg;
        Pitch  = glm::clamp(Pitch, -89.0f, 89.0f);
        updateVectors();
    }
    void changeYaw(float deg)
    {
        Yaw += deg;
        updateVectors();
    }
    void changeRoll(float deg)
    {
        Roll += deg;
        Roll  = glm::clamp(Roll, -45.0f, 45.0f);
        updateVectors();
    }

    // ---- Orbit around orbitTarget (F key, horizontal) ----
    void orbitAroundTarget(float deg)
    {
        glm::vec3 offset = Position - orbitTarget;
        float r  = glm::radians(deg);
        float c  = cosf(r), s = sinf(r);
        float nx = c * offset.x - s * offset.z;
        float nz = s * offset.x + c * offset.z;
        Position = orbitTarget + glm::vec3(nx, offset.y, nz);
        // Repoint at target
        glm::vec3 dir = glm::normalize(orbitTarget - Position);
        Yaw   = glm::degrees(atan2f(dir.z, dir.x));
        Pitch = glm::degrees(asinf(glm::clamp(dir.y, -0.99f, 0.99f)));
        Roll  = 0.0f;
        updateVectors();
    }

    // Legacy compatibility
    void move(int direction, float dt)
    {
        switch (direction) {
        case 0: moveForward(dt);  break;
        case 1: moveBackward(dt); break;
        case 2: moveLeft(dt);     break;
        case 3: moveRight(dt);    break;
        }
    }
};

#endif
