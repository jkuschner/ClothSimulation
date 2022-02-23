//
//  Scene.cpp
//  hw1
//
//  Created by Baichuan Wu on 12/21/21.
//

#include "Scene.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

/// Constructor
Scene::Scene(const std::string& name) : name(name) {
    grid = new Grid("Grid");
    camera = new Camera("Camera");
}

/// Destructor
Scene::~Scene() {
    for (auto object : objects) {
        if (object.second) { delete object.second; }
    }
    objects.clear();
}

void Scene::drawGrid(Shader *shader) {
    GLuint program = shader->program;
    glUseProgram(program);
    glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(camera->view));
    glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, glm::value_ptr(camera->projection));
    glUniformMatrix4fv(glGetUniformLocation(program, "viewInv"), 1, GL_FALSE, glm::value_ptr(camera->getViewInv()));
    glUniformMatrix4fv(glGetUniformLocation(program, "projInv"), 1, GL_FALSE, glm::value_ptr(camera->getProjInv()));
    glUniform1f(glGetUniformLocation(program, "zNear"), camera->getNear());
    glUniform1f(glGetUniformLocation(program, "zFar"), camera->getFar());
    grid->draw(shader);
}

void Scene::draw(Shader *shader) {
    GLuint program = shader->program;
    glUseProgram(program);
    glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(camera->view));
    glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, glm::value_ptr(camera->projection));

    // MARK: Ask each object to draw.
    for (auto& object : objects) {
        object.second->draw(shader);
    }
}
