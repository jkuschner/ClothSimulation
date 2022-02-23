//
//  Scene.h
//  hw1
//
//  Created by Baichuan Wu on 12/20/21.
//

#pragma once

#include "Grid.hpp"
#include "Camera.hpp"
#include "Mesh.hpp"

class Scene {
public:
    std::string name;
    std::map<std::string, Object*> objects;
    Camera* camera;
    Grid* grid;
    
    explicit Scene(const std::string& name);
    void drawGrid(Shader *shader);
    void draw(Shader *shader);
    ~Scene();
};
