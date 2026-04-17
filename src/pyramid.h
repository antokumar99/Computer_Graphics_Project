//
//  pyra.h
//  test
//
//  Created by Badiuzzaman on 4/12/24.
//

#ifndef pyra_h
#define pyra_h

#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "shader.h"

using namespace std;

class Pyramid {
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
    Pyramid()
    {
        setUpPyramidVertexDataAndConfigureVertexAttribute();
    }

    Pyramid(glm::vec3 amb = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 diff = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 spec = glm::vec3(0.5f, 0.5f, 0.5f),
        float shiny = 32.0f)
    {
        this->ambient = amb;
        this->diffuse = diff;
        this->specular = spec;
        this->shininess = shiny;

        setUpPyramidVertexDataAndConfigureVertexAttribute();
    }

    Pyramid(unsigned int tMap, glm::vec3 amb = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 diff = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 spec = glm::vec3(0.5f, 0.5f, 0.5f),
        float shiny = 32.0f)
    {
        this->textureMap = tMap;
        this->ambient = amb;
        this->diffuse = diff;
        this->specular = spec;
        this->shininess = shiny;

        setUpPyramidVertexDataAndConfigureVertexAttribute();
    }

    // destructor
    ~Pyramid()
    {
        glDeleteVertexArrays(1, &pyraVAO);
        glDeleteVertexArrays(1, &lightPyramidVAO);
        glDeleteVertexArrays(1, &lightTexPyramidVAO);
        glDeleteBuffers(1, &pyraVBO);
        glDeleteBuffers(1, &pyraEBO);
    }

    void draw(Shader& lightingShaderWithTexture, glm::mat4 model = glm::mat4(1.0f))
    {
        lightingShaderWithTexture.use();

        lightingShaderWithTexture.setInt("texUnit", 0);
        lightingShaderWithTexture.setVec3("material.ambient", this->ambient);
        lightingShaderWithTexture.setVec3("material.diffuse", this->diffuse);
        lightingShaderWithTexture.setVec3("material.specular", this->specular);
        lightingShaderWithTexture.setFloat("material.shininess", this->shininess);


        // bind diffuse map
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, this->textureMap);

        lightingShaderWithTexture.setMat4("model", model);

        glBindVertexArray(lightTexPyramidVAO);
        glDrawElements(GL_TRIANGLES, 60, GL_UNSIGNED_INT, 0);
    }

private:
    unsigned int pyraVAO;
    unsigned int lightPyramidVAO;
    unsigned int lightTexPyramidVAO;
    unsigned int pyraVBO;
    unsigned int pyraEBO;

    void setUpPyramidVertexDataAndConfigureVertexAttribute()
    {
        // set up vertex data (and buffer(s)) and configure vertex attributes
        // ------------------------------------------------------------------

        float pyra_vertices[] = {
            // positions      // normals         // texture
            // back
            -0.5f, -0.5f, -0.5f, 0.0f, 1.0f, -1.0f, 1, 0,
            0.5f, -0.5f, -0.5f, 0.0f, 1.0f, -1.0f, 0, 0,
            0.0f, 0.5f, 0.0f, 0.0f, 1.0f, -1.0f, 0.5, 1,

            // right
            0.5f, -0.5f, -0.5f, 1.0f,1.0f, 0.0f, 1, 0,
            0.5f, -0.5f, 0.5f, 1.0f, 1.0f, 0.0f, 0, 0,
            0.0f, 0.5f, 0.0f, 1.0f, 1.0f, 0.0f, 0.5, 1,

            // front
            -0.5f, -0.5f, 0.5f, 0.0f, 1.0f, 1.0f, 0, 0,
            0.5f, -0.5f, 0.5f, 0.0f, 1.0f, 1.0f, 1, 0,
            0.0f, 0.5f, 0.0f, 0.0f, 1.0f, 1.0f, 0.5, 1,

            // left
            -0.5f, -0.5f, -0.5f, -1.0f, 1.0f, 0.0f, 0, 0,
            -0.5f, -0.5f, 0.5f, -1.0f, 1.0f, 0.0f, 1, 0,
            0.0f, 0.5f, 0.0f, -1.0f, 1.0f, 0.0f, 0.5, 1,

            // bottom
            -0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f, 0, 0,
            0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f, 1, 0,
            0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f, 1, 1,
            -0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f, 0, 1
        };
        unsigned int pyra_indices[] = {
            1, 0, 2,

            4, 3, 5,

            6, 7, 8,

            9, 10, 11,

            12, 13, 14,
            14, 15, 12
        };

        glGenVertexArrays(1, &pyraVAO);
        glGenVertexArrays(1, &lightPyramidVAO);
        glGenVertexArrays(1, &lightTexPyramidVAO);
        glGenBuffers(1, &pyraVBO);
        glGenBuffers(1, &pyraEBO);


        glBindVertexArray(lightTexPyramidVAO);

        glBindBuffer(GL_ARRAY_BUFFER, pyraVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(pyra_vertices), pyra_vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pyraEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(pyra_indices), pyra_indices, GL_STATIC_DRAW);

        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // vertex normal attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)12);
        glEnableVertexAttribArray(1);

        // texture coordinate attribute
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)24);
        glEnableVertexAttribArray(2);


        glBindVertexArray(lightPyramidVAO);

        glBindBuffer(GL_ARRAY_BUFFER, pyraVBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pyraEBO);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)12);
        glEnableVertexAttribArray(1);


        glBindVertexArray(pyraVAO);

        glBindBuffer(GL_ARRAY_BUFFER, pyraVBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pyraEBO);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    }

};


#endif /* pyra_h */
