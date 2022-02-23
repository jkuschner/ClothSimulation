//
//  EmptyGrid.h
//  hw1
//
//  Created by Baichuan Wu on 1/4/22.
//

#pragma once

#include "Object.hpp"

class Grid : public Object {
private:
    GLuint VAO;
    GLuints buffers;
    
public:
    explicit Grid(const std::string& name) : Object(name) {
        vec4s verts = {
            glm::vec4(1, 1, 0, 1), glm::vec4(-1, -1, 0, 1), glm::vec4(-1, 1, 0, 1),
            glm::vec4(-1, -1, 0, 1), glm::vec4(1, 1, 0, 1), glm::vec4(1, -1, 0, 1)
        };
        GLuints indices = {
            0, 1, 2,
            3, 4, 5
        };
        glGenVertexArrays(1, &VAO);
        buffers.resize(2);
        glGenBuffers(2, buffers.data());
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(glm::vec4), verts.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers[1]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);
        glBindVertexArray(0);
    }
    void draw(Shader* shader) final {
        GLuint program = shader->program;
        glUseProgram(program);
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }
    void update() final {}
    viewer::ObjectType getType() final { return viewer::GRID; }
};
