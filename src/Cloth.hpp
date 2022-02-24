#pragma once
#define Cloth_hpp

#include "core.hpp"
#include "Mesh.hpp"

#define DEFAULT_SPRING_CONSTANT 1.0f
#define DEFAULT_DAMPING_CONSTANT 1.0f
#define DEFAULT_NORMAL glm::vec3(0, 1, 0)
#define PARTICLE_SPACING 0.1f
#define INITIAL_HEIGHT 5.0f
#define SQRT2 1.41421356237f

struct Particle {
    glm::vec3 position;
    glm::vec3 position_prev;
    glm::vec3 velocity;
    glm::vec3 force;
    glm::vec3 normal;
    float     mass;
    bool      isFixed;
    GLuint particleID;

    public:
        glm::vec3 acceleration() { return (1 / mass) * force; }
        glm::vec3 momentum()     { return mass * velocity; }

        Particle(glm::vec3 pos, float m, bool fixed, GLuint id) {
            position = pos;
            position_prev = pos;
            velocity = glm::vec3(0);
            force = glm::vec3(0);
            normal = DEFAULT_NORMAL;
            mass = m;
            isFixed = fixed;
            particleID = id;
        }

        void updatePosition(float timestep) {
            // Verlet w/ no collision detection and no oversampling
            glm::vec3 position_new = 2.0f * position - position_prev;
            position_new += acceleration() * timestep * timestep;
            position_prev = position;
            position = position_new;
        }
};

struct SpringDamper {
    float springConstant;
    float dampingConstant;
    float restLength;
    Particle *p1, *p2;

public:
    SpringDamper(Particle* particle1,
                 Particle* particle2,
                 bool diagonal) {
        springConstant = DEFAULT_SPRING_CONSTANT;
        dampingConstant = DEFAULT_DAMPING_CONSTANT;
        if(diagonal)
            restLength = SQRT2 * PARTICLE_SPACING;
        else
            restLength = PARTICLE_SPACING;

        p1 = particle1;
        p2 = particle2;
    }

    void ComputeForce() {
        glm::vec3 e = p2->position - p1->position;
        float length = glm::length(e);
        e = glm::normalize(e);

        float v_close = glm::dot(p1->velocity - p2->velocity, e);
        float force = -1 * springConstant * (restLength - length);
        force -= dampingConstant * v_close;

        p1->force += force * e;
        p2->force += -1 * force * e;
    }
};

struct Triangle {
    Particle *p1, *p2, *p3;
    glm::vec3 normal;
    glm::vec3 velocity;

public:
    Triangle(Particle* particle1,
             Particle* particle2,
             Particle* particle3) {
        p1 = particle1;
        p2 = particle2;
        p3 = particle3;
        normal = DEFAULT_NORMAL;
    }

    void calcVelocity() {
        velocity = (p1->velocity + p2->velocity + p3->velocity) / 3.0f;
    }

    void calcNormal() {
        normal = glm::cross(p2->position - p1->position, p3->position - p1->position);
        normal = glm::normalize(normal);
    }

    float getArea() { // cross-sectional
        float area = 0.5f * glm::length(glm::cross(p2->position - p1->position, p3->position - p1->position));

        area *= glm::dot(glm::normalize(velocity), normal);
        return area;
    }
};

class Cloth : public Mesh {
public:
    std::vector<std::vector<Particle*>> particles;
    std::vector<SpringDamper*> springDampers;
    std::vector<Triangle*> triangles;

    // constructor for square shaped grid of particles
    explicit Cloth(const std::string& name, int size, float mass);
    
    void update();
};
