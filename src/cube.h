//
//  cube.h
//  test
//
//  Created by Nazirul Hasan on 4/10/23.
//

#ifndef cube_h
#define cube_h

#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "shader.h"

using namespace std;

class Cube {
public:

    // materialistic property
    glm::vec3 ambient;
    glm::vec3 diffuse;
    glm::vec3 specular;

    // texture property
    unsigned int textureMap;

    // common property
    float shininess;

    // constructors
    // Single constructor with all-default params acts as the default constructor
    Cube(glm::vec3 amb = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 diff = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 spec = glm::vec3(0.5f, 0.5f, 0.5f),
        float shiny = 32.0f)
    {
        this->ambient = amb;
        this->diffuse = diff;
        this->specular = spec;
        this->shininess = shiny;

        setUpCubeVertexDataAndConfigureVertexAttribute();
    }

    Cube(unsigned int tMap, glm::vec3 amb = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 diff = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 spec = glm::vec3(0.5f, 0.5f, 0.5f),
        float shiny = 32.0f)
    {
        this->textureMap = tMap;
        this->ambient = amb;
        this->diffuse = diff;
        this->specular = spec;
        this->shininess = shiny;

        setUpCubeVertexDataAndConfigureVertexAttribute();
    }

    // destructor
    ~Cube()
    {
        glDeleteVertexArrays(1, &cubeVAO);
        glDeleteVertexArrays(1, &lightCubeVAO);
        glDeleteVertexArrays(1, &lightTexCubeVAO);
        glDeleteBuffers(1, &cubeVBO);
        glDeleteBuffers(1, &cubeEBO);
    }

    void draw(Shader& lightingShaderWithTexture, glm::mat4 model = glm::mat4(1.0f))
    {
        lightingShaderWithTexture.use();
        lightingShaderWithTexture.setVec3("material.ambient",   this->ambient);
        lightingShaderWithTexture.setVec3("material.diffuse",   this->diffuse);
        lightingShaderWithTexture.setVec3("material.specular",  this->specular);
        lightingShaderWithTexture.setFloat("material.shininess", this->shininess);
        lightingShaderWithTexture.setBool("emissive", false);
        lightingShaderWithTexture.setBool("useTexture", this->textureMap != 0);
        if (this->textureMap) {
            lightingShaderWithTexture.setInt("texUnit", 0);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, this->textureMap);
        }
        lightingShaderWithTexture.setMat4("model", model);
        glBindVertexArray(lightTexCubeVAO);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
    }

    // Draw with emissive color (for active lights)
    void drawEmissive(Shader& shader, glm::mat4 model, glm::vec3 color) {
        shader.use();
        shader.setBool("emissive", true);
        shader.setVec3("emissiveColor", color);
        shader.setMat4("model", model);
        glBindVertexArray(lightTexCubeVAO);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
        shader.setBool("emissive", false);
    }

    void drawColor(Shader& lightingShaderWithTexture, glm::mat4 model = glm::mat4(1.0f))
    {
        lightingShaderWithTexture.use();

        lightingShaderWithTexture.setVec3("color", this->ambient);

        // bind diffuse map
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, this->textureMap);

        lightingShaderWithTexture.setMat4("model", model);

        glBindVertexArray(lightTexCubeVAO);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
    }

private:
    unsigned int cubeVAO;
    unsigned int lightCubeVAO;
    unsigned int lightTexCubeVAO;
    unsigned int cubeVBO;
    unsigned int cubeEBO;

    void setUpCubeVertexDataAndConfigureVertexAttribute()
    {
        // set up vertex data (and buffer(s)) and configure vertex attributes
        // ------------------------------------------------------------------

        float cube_vertices[] = {
            // positions      // normals         // texture
            // back
            -0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 0.75, 0,
            0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 0.5, 0,
            0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 0.5, 1,
            -0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 0.75, 1,

            // right
            0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f, 0.5, 0,
            0.5f, 0.5f, -0.5f, 1.0f, 0.0f, 0.0f, 0.5, 1,
            0.5f, -0.5f, 0.5f, 1.0f, 0.0f, 0.0f, 0.25, 0,
            0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f, 0.25, 1,

            // front
            -0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 0, 0,
            0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 0.25, 0,
            0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 0.25, 0.25,
            -0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 0, 0.25,

            // left
            -0.5f, -0.5f, 0.5f, -1.0f, 0.0f, 0.0f, 1, 0,
            -0.5f, 0.5f, 0.5f, -1.0f, 0.0f, 0.0f, 1, 1,
            -0.5f, 0.5f, -0.5f, -1.0f, 0.0f, 0.0f, 0.75, 1,
            -0.5f, -0.5f, -0.5f, -1.0f, 0.0f, 0.0f, 0.75, 0,

            // top
            0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 1, 0,
            0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f, 1, 2,
            -0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f, 0, 2,
            -0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 0, 0,

            // bottom
            -0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f, 0, 0,
            0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f, 1, 0,
            0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f, 1, 1,
            -0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f, 0, 1
        };
        unsigned int cube_indices[] = {
            0, 3, 2,
            2, 1, 0,

            4, 5, 7,
            7, 6, 4,

            8, 9, 10,
            10, 11, 8,

            12, 13, 14,
            14, 15, 12,

            16, 17, 18,
            18, 19, 16,

            20, 21, 22,
            22, 23, 20
        };

        glGenVertexArrays(1, &cubeVAO);
        glGenVertexArrays(1, &lightCubeVAO);
        glGenVertexArrays(1, &lightTexCubeVAO);
        glGenBuffers(1, &cubeVBO);
        glGenBuffers(1, &cubeEBO);


        glBindVertexArray(lightTexCubeVAO);

        glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(cube_vertices), cube_vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cube_indices), cube_indices, GL_STATIC_DRAW);

        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // vertex normal attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)12);
        glEnableVertexAttribArray(1);

        // texture coordinate attribute
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)24);
        glEnableVertexAttribArray(2);


        glBindVertexArray(lightCubeVAO);

        glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)12);
        glEnableVertexAttribArray(1);


        glBindVertexArray(cubeVAO);

        glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    }

};


#endif /* cube_h */
