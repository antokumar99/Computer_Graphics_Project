// sphere.h – UV sphere for Phong-shaded rendering
// Matches the interface of cube.h and cylinder.h

#ifndef SPHERE_H
#define SPHERE_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <cmath>
#include "shader.h"

#ifndef SPH_PI
#define SPH_PI 3.14159265358979f
#endif

class Sphere {
public:
    glm::vec3 ambient, diffuse, specular;
    float     shininess;
    unsigned int textureMap;

    Sphere(int stacks = 24, int sectors = 36,
           glm::vec3 amb  = glm::vec3(0.5f),
           glm::vec3 diff = glm::vec3(0.8f),
           glm::vec3 spec = glm::vec3(0.5f),
           float     shine = 32.0f,
           unsigned int tex = 0)
        : ambient(amb), diffuse(diff), specular(spec),
          shininess(shine), textureMap(tex)
    {
        build(stacks, sectors);
    }

    ~Sphere() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
    }

    // Phong-lit draw
    void draw(Shader& sh, glm::mat4 model) {
        sh.setVec3("material.ambient",    ambient);
        sh.setVec3("material.diffuse",    diffuse);
        sh.setVec3("material.specular",   specular);
        sh.setFloat("material.shininess", shininess);
        sh.setMat4("model", model);
        sh.setBool("emissive", false);
        sh.setBool("useTexture", textureMap != 0);
        if (textureMap) {
            sh.setInt("texUnit", 0);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, textureMap);
        }
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

    // Solid-colour shorthand
    void drawColor(Shader& sh, glm::mat4 model, glm::vec3 col, float shine = 32.0f) {
        ambient   = col * 0.35f;
        diffuse   = col;
        specular  = col * 0.5f;
        shininess = shine;
        textureMap = 0;
        draw(sh, model);
    }

    // Emissive (bypass lighting, pure colour)
    void drawEmissive(Shader& sh, glm::mat4 model, glm::vec3 color) {
        sh.setMat4("model", model);
        sh.setBool("emissive", true);
        sh.setVec3("emissiveColor", color);
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        sh.setBool("emissive", false);
    }

private:
    unsigned int VAO = 0, VBO = 0, EBO = 0;
    int indexCount = 0;

    void build(int stacks, int sectors) {
        std::vector<float>        verts;
        std::vector<unsigned int> idxs;

        float sStep = 2.0f * SPH_PI / sectors;
        float tStep = SPH_PI / stacks;

        for (int i = 0; i <= stacks; ++i) {
            float phi = SPH_PI / 2.0f - i * tStep;   // pi/2 … -pi/2
            float xz  = cosf(phi);
            float y   = sinf(phi);

            for (int j = 0; j <= sectors; ++j) {
                float theta = j * sStep;
                float x = xz * cosf(theta);
                float z = xz * sinf(theta);

                // position
                verts.push_back(x);
                verts.push_back(y);
                verts.push_back(z);
                // normal (= position on unit sphere)
                verts.push_back(x);
                verts.push_back(y);
                verts.push_back(z);
                // UV
                verts.push_back((float)j / sectors);
                verts.push_back((float)i / stacks);
            }
        }

        for (int i = 0; i < stacks; ++i) {
            int k1 = i * (sectors + 1);
            int k2 = k1 + sectors + 1;
            for (int j = 0; j < sectors; ++j, ++k1, ++k2) {
                if (i != 0) {
                    idxs.push_back(k1);
                    idxs.push_back(k2);
                    idxs.push_back(k1 + 1);
                }
                if (i != stacks - 1) {
                    idxs.push_back(k1 + 1);
                    idxs.push_back(k2);
                    idxs.push_back(k2 + 1);
                }
            }
        }

        indexCount = (int)idxs.size();

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER,
            verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
            idxs.size() * sizeof(unsigned int), idxs.data(), GL_STATIC_DRAW);

        // attrib 0: position
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        // attrib 1: normal
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float),
            (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        // attrib 2: UV
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float),
            (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(2);

        glBindVertexArray(0);
    }
};

#endif // SPHERE_H
