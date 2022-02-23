//
//  Cube.hpp
//  hw1
//
//  Created by Baichuan Wu on 1/5/22.
//

#pragma once

#include "Mesh.hpp"

class Cube : public Mesh {
public:
    explicit Cube(const std::string& name, const glm::vec3& cubeMin, const glm::vec3& cubeMax) : Mesh(name) {
        verts = {
            new Vertex(glm::vec4(cubeMax.x, cubeMax.y, cubeMin.z, 1)),
            new Vertex(glm::vec4(cubeMax.x, cubeMin.y, cubeMin.z, 1)),
            new Vertex(glm::vec4(cubeMax.x, cubeMax.y, cubeMax.z, 1)),
            new Vertex(glm::vec4(cubeMax.x, cubeMin.y, cubeMax.z, 1)),
            new Vertex(glm::vec4(cubeMin.x, cubeMax.y, cubeMin.z, 1)),
            new Vertex(glm::vec4(cubeMin.x, cubeMin.y, cubeMin.z, 1)),
            new Vertex(glm::vec4(cubeMin.x, cubeMax.y, cubeMax.z, 1)),
            new Vertex(glm::vec4(cubeMin.x, cubeMin.y, cubeMax.z, 1))
        };
        GLuints indices = {
            5, 3, 1,    3, 8, 4,
            7, 6, 8,    2, 8, 6,
            1, 4, 2,    5, 2, 6,
            5, 7, 3,    3, 7, 8,
            7, 5, 6,    2, 4, 8,
            1, 3, 4,    5, 1, 2
        };
        for (size_t i = 0; i < indices.size(); i += 3) {
            faces.push_back(new Face(verts[indices[i] - 1], verts[indices[i + 1] - 1], verts[indices[i + 2] - 1]));
        }
        vec4s _verts;
        vec4s _normals;
        GLuints _indices;
        for (size_t i = 0; i < faces.size(); ++i) {
            _verts.push_back(faces[i]->v1->position);
            _verts.push_back(faces[i]->v2->position);
            _verts.push_back(faces[i]->v3->position);
            for (size_t j = 0; j < 3; ++j) {
                _normals.push_back(faces[i]->fnormal);
                _indices.push_back(static_cast<GLuint>(i * 3 + j));
            }
        }

        glGenVertexArrays(1, &VAO);
        buffers.resize(3);  // v, n, vt
        glGenBuffers(3, buffers.data());
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
        glBufferData(GL_ARRAY_BUFFER, _verts.size() * sizeof(glm::vec4), _verts.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

        glBindBuffer(GL_ARRAY_BUFFER, buffers[1]);
        glBufferData(GL_ARRAY_BUFFER, _normals.size() * sizeof(glm::vec4), _normals.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers[2]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, _indices.size() * sizeof(GLuint), _indices.data(), GL_STATIC_DRAW);

        glBindVertexArray(0);

    }
    
    void update() final {
        matrix_world = glm::rotate(glm::mat4(1), glm::radians(1.f), glm::vec3(0.0f, 1.0f, 0.0f)) * matrix_world;
    }

};
