#pragma once

#include "Object.hpp"

// The Camera class provides a simple means to controlling the 3D camera. It could
// be extended to support more interactive controls. Ultimately. the camera sets the
// GL projection and viewing matrices.

class Camera : public Object {
public:
    glm::mat4 view;
    glm::mat4 projection;
    glm::mat4 projInv;
    
    explicit Camera(const std::string& name) : Object(name) {
        reset();
    }

    void update() final {
        // Compute camera world matrix
        matrix_world = glm::mat4(1);
        matrix_world[3][2] = distance;
        matrix_world = glm::translate(glm::mat4(1), pivot) * glm::eulerAngleY(glm::radians(-azimuth)) * glm::eulerAngleX(glm::radians(-incline)) * matrix_world;

        // Compute view matrix (inverse of world matrix)
        view = glm::inverse(matrix_world);

    }
    
    void reset() {
        fov=45.0;
        aspect=1.33f;
        near=0.1f;
        far=100.0f;
        distance=10.0f;
        azimuth=0.0f;
        incline=20.0f;
        pivot = glm::vec3(0);
        // Compute perspective projection matrix
        projection = glm::perspective(glm::radians(fov), aspect, near, far);
        projInv = glm::inverse(projection);
        update();
    }

    // Access functions
    void setAspect(float a)                 { aspect=a; }
    void setDistance(float d)               { distance=d; }
    void setAzimuth(float a)                { azimuth=a; }
    void setIncline(float i)                { incline=i; }
    void offsetPivot(const glm::vec2& dir) {
        pivot += glm::vec3(matrix_world * glm::vec4(dir, 0, 0));
    }

    float getNear()                         { return near; }
    float getFar()                          { return far; }
    float getDistance()                     { return distance; }
    float getAzimuth()                      { return azimuth; }
    float getIncline()                      { return incline; }
    glm::mat4 getViewInv() {
        return matrix_world;
    }
    glm::mat4 getProjInv() {
        return projInv;
    }

    void draw(Shader* shader) final {}

    viewer::ObjectType getType() final { return viewer::CAMERA; }

private:
    // Perspective controls
    float fov;            // Field of View Angle (degrees)
    float aspect;        // Aspect Ratio
    float near;        // Near clipping plane distance
    float far;        // Far clipping plane distance

    // Polar controls
    float distance;        // Distance of the camera eye position to the origin (meters)
    float azimuth;        // Rotation of the camera eye position around the Y axis (degrees)
    float incline;        // Angle of the camera eye position over the XZ plane (degrees)
    
    glm::vec3 pivot = glm::vec3(0);

};
