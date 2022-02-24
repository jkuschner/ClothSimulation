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
    explicit Cloth(const std::string& name, 
                   int size,
                   float mass) : Mesh(name) {

        matrix_world = glm::mat4(1);

        particles.reserve(size);
        glm::vec3 tmpPos;
        tmpPos.y = INITIAL_HEIGHT;
        bool fixed = false;
        unsigned int particleCount = 0;
        for(int i = 0; i < size; i++) {
            std::vector<Particle*> nextvec;
            particles.push_back(nextvec);
            particles.back().reserve(size);
            for(int j = 0; j < size; j++) {
                tmpPos.x = j * PARTICLE_SPACING;
                tmpPos.z = i * PARTICLE_SPACING;
                // only 2 corners are fixed
                if(i == 0 && (j == 0 || j == size - 1))
                    fixed = true;

                particles.back().push_back(new Particle(tmpPos, mass, fixed, particleCount));
                particleCount++;

                // create and connect SpringDampers
                if(j > 0 && i > 0 && j < size-1) {
                    springDampers.push_back(new SpringDamper(
                        particles[i][j-1], particles[i][j], false));
                    springDampers.push_back(new SpringDamper(
                        particles[i-1][j], particles[i][j], false));
                    springDampers.push_back(new SpringDamper(
                        particles[i-1][j-1], particles[i][j], true));
                    springDampers.push_back(new SpringDamper(
                        particles[i-1][j+1], particles[i][j], true));
                } else if (j > 0 && i > 0) {
                    springDampers.push_back(new SpringDamper(
                        particles[i][j-1], particles[i][j], false));
                    springDampers.push_back(new SpringDamper(
                        particles[i-1][j], particles[i][j], false));
                    springDampers.push_back(new SpringDamper(
                        particles[i-1][j-1], particles[i][j], true));
                } else if (i > 0 && j < size-1) {
                    springDampers.push_back(new SpringDamper(
                        particles[i-1][j], particles[i][j], false));
                    springDampers.push_back(new SpringDamper(
                        particles[i-1][j+1], particles[i][j], true));
                } else if (i > 0) {
                    //this block might not be neccessary...
                    springDampers.push_back(new SpringDamper(
                        particles[i-1][j], particles[i][j], false));
                } else if (j > 0) {
                    springDampers.push_back(new SpringDamper(
                        particles[i][j-1], particles[i][j], false));
                }

                // create and connect Triangles
                if(i > 0 && j > 0) {
                    triangles.push_back(new Triangle(
                        particles[i][j],particles[i][j-1],particles[i-1][j-1]));
                    triangles.push_back(new Triangle(
                        particles[i][j],particles[i-1][j-1],particles[i-1][j]));
                }
            }
        }

        // set up VAO
        vec4s positions;
        vec4s normals;
        GLuints indices;

        for(int i = 0; i < triangles.size(); i++) {
            indices.push_back(triangles[i]->p1->particleID);
            indices.push_back(triangles[i]->p2->particleID);
            indices.push_back(triangles[i]->p3->particleID);
        }

        for(int i = 0; i < size; i++) {
            for(int j = 0; j < size; j++) {
                positions.push_back(glm::vec4(particles[i][j]->position, 1));
                normals.push_back(glm::vec4(particles[i][j]->normal, 0));
                verts.push_back(new Vertex(positions.back(), normals.back()));
            }
        }

        for(int i = 0; i < indices.size(); i+=3) {
            faces.push_back(new Face(verts[indices[i]], verts[indices[i+1]], verts[indices[i+2]]));
        }

            glGenVertexArrays(1, &VAO);
            buffers.resize(3);  // v, n, vt
            glGenBuffers(3, buffers.data());
            glBindVertexArray(VAO);

            glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
            glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec4), positions.data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

            glBindBuffer(GL_ARRAY_BUFFER, buffers[1]);
            glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec4), normals.data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

            glBindVertexArray(0);
    }
    
    void update() {
        matrix_world = glm::mat4(1);
    }
};
