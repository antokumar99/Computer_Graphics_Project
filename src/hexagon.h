//
//  hexa.h
//  test
//
//  Created by Nazirul Hasan on 4/10/23.
//

#ifndef hexa_h
#define hexa_h

#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "shader.h"

using namespace std;

class Hexagon {
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
    Hexagon()
    {
        setUpHexagonVertexDataAndConfigureVertexAttribute();
    }

    Hexagon(glm::vec3 amb = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 diff = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 spec = glm::vec3(0.5f, 0.5f, 0.5f),
        float shiny = 32.0f)
    {
        this->ambient = amb;
        this->diffuse = diff;
        this->specular = spec;
        this->shininess = shiny;

        setUpHexagonVertexDataAndConfigureVertexAttribute();
    }

    Hexagon(unsigned int tMap, glm::vec3 amb = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 diff = glm::vec3(1.0f, 0.5f, 0.3f),
        glm::vec3 spec = glm::vec3(0.5f, 0.5f, 0.5f),
        float shiny = 32.0f)
    {
        this->textureMap = tMap;
        this->ambient = amb;
        this->diffuse = diff;
        this->specular = spec;
        this->shininess = shiny;

        setUpHexagonVertexDataAndConfigureVertexAttribute();
    }

    // destructor
    ~Hexagon()
    {
        glDeleteVertexArrays(1, &hexaVAO);
        glDeleteVertexArrays(1, &lightHexagonVAO);
        glDeleteVertexArrays(1, &lightTexHexagonVAO);
        glDeleteBuffers(1, &hexaVBO);
        glDeleteBuffers(1, &hexaEBO);
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

        glBindVertexArray(lightTexHexagonVAO);
        glDrawElements(GL_TRIANGLES, 60, GL_UNSIGNED_INT, 0);
    }

private:
    unsigned int hexaVAO;
    unsigned int lightHexagonVAO;
    unsigned int lightTexHexagonVAO;
    unsigned int hexaVBO;
    unsigned int hexaEBO;

    void setUpHexagonVertexDataAndConfigureVertexAttribute()
    {
        // set up vertex data (and buffer(s)) and configure vertex attributes
        // ------------------------------------------------------------------

        float hexa_vertices[] = {
            // positions      // normals         // texture
            // back
            -0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 1, 0,
            0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 0, 0,
            0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 0, 1,
            -0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 1, 1,

            // front
            -0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 0, 0,
            0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 1, 0,
            0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 1, 1,
            -0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 0, 1,

            // front-right
            0.5f, -0.5f, 0.5f, 1.0f, 0.0f, 1.0f, 0, 0,
            1.0f, -0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 1, 0,
            1.0f, 0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 1, 1,
            0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 1.0f, 0, 1,

            // back-right
            0.5f, -0.5f, -0.5f, 1.0f, 0.0f, -1.0f, 1, 0,
            1.0f, -0.5f, 0.0f, 1.0f, 0.0f, -1.0f, 0, 0,
            1.0f, 0.5f, 0.0f, 1.0f, 0.0f, -1.0f, 0, 1,
            0.5f, 0.5f, -0.5f, 1.0f, 0.0f, -1.0f, 1, 1,

            // front-left
            -0.5f, -0.5f, 0.5f, -1.0f, 0.0f, 1.0f, 1, 0,
            -1.0f, -0.5f, 0.0f, -1.0f, 0.0f, 1.0f, 0, 0,
            -1.0f, 0.5f, 0.0f, -1.0f, 0.0f, 1.0f, 0, 1,
            -0.5f, 0.5f, 0.5f, -1.0f, 0.0f, 1.0f, 1, 1,

            // back-left
            -0.5f, -0.5f, -0.5f, -1.0f, 0.0f, -1.0f, 0, 0,
            -1.0f, -0.5f, 0.0f, -1.0f, 0.0f, -1.0f, 1, 0,
            -1.0f, 0.5f, 0.0f, -1.0f, 0.0f, -1.0f, 1, 1,
            -0.5f, 0.5f, -0.5f, -1.0f, 0.0f, -1.0f, 0, 1,

            // top
            0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 0.75, 0,
            0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f, 0.75, 1,
            -0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f, 0.25, 1,
            -0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 0.25, 0,

            -1.0f, 0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 0, 0.5,
            1.0f, 0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 1, 0.5,

            // bottom
            -0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f, 0.25, 0,
            0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f, 0.75, 0,
            0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f, 0.75, 1,
            -0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f, 0.25, 1,

            -1.0f, -0.5f, 0.0f, 0.0f, -1.0f, 0.0f, 0, 0.5,
            1.0f, -0.5f, 0.0f, 0.0f, -1.0f, 0.0f, 1, 0.5
        };
        unsigned int hexa_indices[] = {
            0, 3, 2,
            2, 1, 0,

            4, 5, 6,
            4, 6, 7,

            8, 9, 10,
            10, 11, 8,

            12, 13, 14,
            14, 15, 12,

            16, 17, 18,
            18, 19, 16,

            20, 21, 22,
            22, 23, 20,

            24, 25, 26,
            26, 27, 24,
            27, 26, 28,
            25, 24, 29,

            30, 31, 32,
            32, 33, 30,
            30, 33, 34,
            32, 31, 35
        };

        glGenVertexArrays(1, &hexaVAO);
        glGenVertexArrays(1, &lightHexagonVAO);
        glGenVertexArrays(1, &lightTexHexagonVAO);
        glGenBuffers(1, &hexaVBO);
        glGenBuffers(1, &hexaEBO);


        glBindVertexArray(lightTexHexagonVAO);

        glBindBuffer(GL_ARRAY_BUFFER, hexaVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(hexa_vertices), hexa_vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, hexaEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(hexa_indices), hexa_indices, GL_STATIC_DRAW);

        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // vertex normal attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)12);
        glEnableVertexAttribArray(1);

        // texture coordinate attribute
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)24);
        glEnableVertexAttribArray(2);


        glBindVertexArray(lightHexagonVAO);

        glBindBuffer(GL_ARRAY_BUFFER, hexaVBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, hexaEBO);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)12);
        glEnableVertexAttribArray(1);


        glBindVertexArray(hexaVAO);

        glBindBuffer(GL_ARRAY_BUFFER, hexaVBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, hexaEBO);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    }

};


#endif /* hexa_h */
