//
//  Mesh.hpp
//  hw1
//
//  Created by Baichuan Wu on 12/19/21.
//

#pragma once

#include "Object.hpp"
#include <tiny_obj_loader.h>

struct Vertex {
    glm::vec4 position;
    glm::vec4 normal;
    Vertex(const glm::vec4& _position, const glm::vec4& _normal = glm::vec4(0)) : position(_position), normal(_normal) {}
};

struct Edge {
    Vertex* v1, *v2;
    Edge(Vertex* const _v1, Vertex* const _v2) : v1(_v1), v2(_v2) {}
};

struct Face {
    Vertex* v1, *v2, *v3;
    glm::vec4 vnormal;
    glm::vec4 fnormal;
    Face(Vertex* const _v1, Vertex* const _v2, Vertex* const _v3) : v1(_v1), v2(_v2), v3(_v3) {
        vnormal = glm::normalize(v1->normal + v2->normal + v3->normal);
        fnormal = glm::vec4(glm::normalize(glm::cross(glm::vec3(v2->position - v3->position), glm::vec3(v1->position - v3->position))), 0.f);   // RHR
    }
};

class Mesh : public Object {
public:
    GLuint VAO;
    GLuints buffers;
    
    std::vector<Vertex*> verts;
    std::vector<Edge*> edges;
    std::vector<Face*> faces;

    /// Dummy constructor
    explicit Mesh(const std::string& name) : Object(name) {
        matrix_world = glm::mat4(1);
    }

    /// Construct a mesh from tinyobjloader values.
    explicit Mesh(const std::string& name,
                  const tinyobj::attrib_t& attrib,
                  const tinyobj::mesh_t& mesh,
                  const tinyobj::material_t& material
                  ) : Object(name) {

        matrix_world = glm::mat4(1);

        vec4s _verts;
        vec4s _normals;
        GLuints _indices;

        for (size_t v = 0; v < attrib.vertices.size(); v += 3) {
            const GLfloat vx = attrib.vertices[v];
            const GLfloat vy = attrib.vertices[v + 1];
            const GLfloat vz = attrib.vertices[v + 2];
            _verts.push_back(glm::vec4(vx, vy, vz, 1));
            verts.push_back(new Vertex(_verts.back()));
        }

        // Loop-add, triangulated
        size_t index_offset = 0;
        for (auto& f : mesh.num_face_vertices) {
            size_t fv = size_t(f);  assert(fv == 3);
            for (size_t i = 0; i < fv; ++i) {
                tinyobj::index_t idx = mesh.indices[index_offset + i];
                GLfloat nx = 0, ny = 0, nz = 0;
                if (idx.normal_index >= 0) {
                    nx = attrib.normals[3 * size_t(idx.normal_index) + 0];
                    ny = attrib.normals[3 * size_t(idx.normal_index) + 1];
                    nz = attrib.normals[3 * size_t(idx.normal_index) + 2];
                }
                verts[idx.vertex_index]->normal = glm::vec4(nx, ny, nz, 0);
                // MARK: Add vt & vcol support here if needed
                _indices.push_back(idx.vertex_index);
            }
            auto v1 = verts[*(_indices.end() - 1)];
            auto v2 = verts[*(_indices.end() - 2)];
            auto v3 = verts[*(_indices.end() - 3)];
            edges.push_back(new Edge(v1, v2));
            edges.push_back(new Edge(v2, v3));
            edges.push_back(new Edge(v3, v1));
            faces.push_back(new Face(v1, v2, v3));
            index_offset += fv;
        }

        for (auto& v: verts) {
            _normals.push_back(v->normal);
        }

        std::cout << "Loaded " << name << "; ";
        std::cout << verts.size() << " vertices, " << faces.size() << " faces. " << std::endl;
        
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
    
    ~Mesh() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(3, buffers.data());
        verts.clear();
        edges.clear();
        faces.clear();
    }
    
    void draw(Shader* shader) final {
        glm::mat4 invT = glm::inverse(glm::transpose(matrix_world));

        GLuint program = shader->program;
        glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(matrix_world));
        glUniformMatrix4fv(glGetUniformLocation(program, "invT"), 1, GL_FALSE, glm::value_ptr(invT));

        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, static_cast<int>(faces.size()) * 3, GL_UNSIGNED_INT, 0);
    }
    
    virtual void update() {}

    viewer::ObjectType getType() final { return viewer::MESH; }
};

