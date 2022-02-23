//
//  Object.hpp
//  hw1
//
//  Created by Baichuan Wu on 12/21/21.
//

#pragma once

#include "core.hpp"
#include "Shader.hpp"

class Object {
public:
    std::string name;
    glm::vec4 pivot;
    glm::mat4 matrix_world;
    
    explicit Object(const std::string& name) : name(name) {}
    virtual ~Object() {}
    
    virtual void draw(Shader* shader) = 0;
    virtual void update() = 0;
    
    virtual viewer::ObjectType getType() = 0;
    
}; 
