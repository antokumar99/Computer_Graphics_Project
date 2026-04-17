// cylinder.h
// Bezier Curve Surface of Revolution
// Theory: Curve.pptx - 3D object drawing using Bezier curves
// A 2D Bezier profile (radius, height) is rotated around the Y-axis to produce
// a 3D surface of revolution: X = r*cos(theta), Y = y, Z = r*sin(theta)

#ifndef CYLINDER_H
#define CYLINDER_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <cmath>
#include "shader.h"

using namespace std;

const float MY_PI = 3.14159265358979f;

// ============================================================
// BezierSurface - Surface of Revolution from Bezier Profile
// ============================================================
class BezierSurface {
public:
    glm::vec3 ambient, diffuse, specular;
    float shininess;
    unsigned int textureMap;

    // ctrlPoints: 2D control points (x=radius, y=height)
    BezierSurface(vector<glm::vec2> ctrl,
        glm::vec3 amb  = glm::vec3(0.8f, 0.4f, 0.2f),
        glm::vec3 diff = glm::vec3(0.8f, 0.4f, 0.2f),
        glm::vec3 spec = glm::vec3(0.5f, 0.5f, 0.5f),
        float shine    = 32.0f,
        unsigned int tex = 0)
        : ctrlPoints(ctrl), ambient(amb), diffuse(diff),
          specular(spec), shininess(shine), textureMap(tex)
    {
        generateSurface(40, 48);
    }

    ~BezierSurface() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
    }

    void draw(Shader& shader, glm::mat4 model) {
        shader.use();
        shader.setVec3("material.ambient",   ambient);
        shader.setVec3("material.diffuse",   diffuse);
        shader.setVec3("material.specular",  specular);
        shader.setFloat("material.shininess", shininess);
        shader.setMat4("model", model);
        shader.setBool("useTexture", textureMap != 0);
        if (textureMap) {
            shader.setInt("texUnit", 0);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, textureMap);
        }
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

    // Re-generate with new material colors (no geometry change)
    void setColor(glm::vec3 amb, glm::vec3 diff, glm::vec3 spec, float shine = 32.0f) {
        ambient   = amb;
        diffuse   = diff;
        specular  = spec;
        shininess = shine;
    }

private:
    vector<glm::vec2> ctrlPoints;
    unsigned int VAO, VBO, EBO;
    int indexCount;

    // ---- Bernstein basis / Bezier evaluation ----
    // Returns binomial coefficient C(n,k) as float (control points <= 10, no overflow)
    float binomial(int n, int k) {
        if (k > n - k) k = n - k;
        float r = 1.0f;
        for (int i = 0; i < k; ++i) {
            r *= (float)(n - i);
            r /= (float)(i + 1);
        }
        return r;
    }

    // Point on Bezier curve at parameter t
    glm::vec2 bezierPoint(float t) {
        int n = (int)ctrlPoints.size() - 1;
        glm::vec2 p(0.0f);
        for (int i = 0; i <= n; ++i) {
            float b = binomial(n, i) * powf(t, (float)i) * powf(1.0f - t, (float)(n - i));
            p += b * ctrlPoints[i];
        }
        return p;
    }

    // Tangent of Bezier curve at parameter t (derivative)
    glm::vec2 bezierTangent(float t) {
        int n = (int)ctrlPoints.size() - 1;
        if (n < 1) return glm::vec2(0.0f, 1.0f);
        glm::vec2 d(0.0f);
        for (int i = 0; i < n; ++i) {
            float b = binomial(n - 1, i) * powf(t, (float)i) * powf(1.0f - t, (float)(n - 1 - i));
            d += b * (float)n * (ctrlPoints[i + 1] - ctrlPoints[i]);
        }
        return d;
    }

    // uSteps = resolution along the curve, vSteps = slices around Y axis
    void generateSurface(int uSteps, int vSteps) {
        vector<float> verts;
        vector<unsigned int> inds;

        for (int i = 0; i <= uSteps; ++i) {
            float t   = (float)i / (float)uSteps;
            glm::vec2 pt  = bezierPoint(t);
            glm::vec2 tang = bezierTangent(t);
            float r   = pt.x;
            float y   = pt.y;
            float dr  = tang.x;
            float dy  = tang.y;

            for (int j = 0; j <= vSteps; ++j) {
                float theta = (float)j / (float)vSteps * 2.0f * MY_PI;
                float cosT  = cosf(theta);
                float sinT  = sinf(theta);

                // Position: rotate profile point around Y axis
                float px = r * cosT;
                float py = y;
                float pz = r * sinT;

                // Surface tangents
                glm::vec3 dT(dr * cosT, dy,      dr * sinT);    // along t
                glm::vec3 dA(-r * sinT, 0.0f,   r * cosT);     // around axis (theta)

                glm::vec3 normal = glm::normalize(glm::cross(dT, dA));
                // Flip if pointing inward
                if (glm::dot(normal, glm::vec3(cosT, 0.0f, sinT)) < 0.0f)
                    normal = -normal;

                float u = (float)j / (float)vSteps;
                float v = t;

                verts.push_back(px); verts.push_back(py); verts.push_back(pz);
                verts.push_back(normal.x); verts.push_back(normal.y); verts.push_back(normal.z);
                verts.push_back(u); verts.push_back(v);
            }
        }

        for (int i = 0; i < uSteps; ++i) {
            for (int j = 0; j < vSteps; ++j) {
                unsigned int a = i * (vSteps + 1) + j;
                unsigned int b = a + (vSteps + 1);
                inds.push_back(a);     inds.push_back(b);     inds.push_back(a + 1);
                inds.push_back(a + 1); inds.push_back(b);     inds.push_back(b + 1);
            }
        }

        indexCount = (int)inds.size();

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, inds.size() * sizeof(unsigned int), inds.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(2);

        glBindVertexArray(0);
    }
};

#endif // CYLINDER_H
