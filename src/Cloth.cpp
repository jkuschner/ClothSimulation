#include "Cloth.hpp"

Cloth::Cloth(const std::string& name, int size, float mass) : Mesh(name) {

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
            //if(i == 0 && (j == 0 || j == size - 1))
            // row of vertices are fixed
            if(i == 0)
                fixed = true;
            else
                fixed = false;

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

    positions.reserve(particleCount);
    normals.reserve(particleCount);

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

void Cloth::update() {
    // zero out forces
    for(int i = 0; i < particles.size(); i++) {
        for(int j = 0; j < particles[0].size(); j++) {
            particles[i][j]->force = glm::vec3(0);
        }
    }

    // Apply all forces
    // apply gravity to all particles
    glm::vec3 gravity = glm::vec3(0, GRAVITY, 0);
    for(int i = 0; i < particles.size(); i++) {
        for(int j = 0; j < particles[0].size(); j++) {
            particles[i][j]->force += gravity;
        }
    }

    // apply springdamper force
    for(int i = 0; i < springDampers.size(); i++) {
        springDampers[i]->ComputeForce();
    }
    
    // Integrate motion
    for(int i = 0; i < particles.size(); i++) {
        for(int j = 0; j < particles[0].size(); j++) {
            if(particles[i][j]->isFixed == false)
                particles[i][j]->updatePosition(TIME_STEP);
        }
    }

    // zero out particle normals
    for(int i = 0; i < particles.size(); i++) {
        for(int j = 0; j < particles[0].size(); j++) {
            if(particles[i][j]->isFixed == false)
                particles[i][j]->normal = glm::vec3(0);
        }
    }

    //Loop through all triangles and add the triangle normal to the normal of each of the three particles it connects
    for(int i = 0; i < triangles.size(); i++) {
        triangles[i]->calcNormal();
        triangles[i]->p1->normal += triangles[i]->normal;
        triangles[i]->p2->normal += triangles[i]->normal;
        triangles[i]->p3->normal += triangles[i]->normal;
    }

    //Loop through all the particles again and normalize the normal
    for(int i = 0; i < particles.size(); i++) {
        for(int j = 0; j < particles[0].size(); j++) {
            glm::normalize(particles[i][j]->normal);
        }
    }

    vec4s positions;
    vec4s normals;
    positions.reserve(verts.size());
    normals.reserve(verts.size());

    for(int i = 0; i < particles.size(); i++) {
        for(int j = 0; j < particles[0].size(); j++) {
            positions.push_back(glm::vec4(particles[i][j]->position, 1));
            normals.push_back(glm::vec4(particles[i][j]->normal, 0));
        }
    }

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
    glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec4), positions.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, buffers[1]);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec4), normals.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindVertexArray(0);

}