//
//  core.h
//  hw1
//
//  Created by Xuezheng Wang on 1/4/22.
//

#pragma once

#ifdef __APPLE__
#define GL_DO_NOT_WARN_IF_MULTI_GL_VERSION_HEADERS_INCLUDED
#endif

#include <OpenGL/gl3.h>
#include <OpenGL/glext.h>
#include <GLUT/glut.h>


#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <map>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

#define vec4s std::vector<glm::vec4>
#define mat4s std::vector<glm::mat4>
#define GLuints std::vector<GLuint>

namespace viewer {
    enum ObjectType {
        MESH,
        LIGHT,
        CAMERA,
        GRID
    };
}
