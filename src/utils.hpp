//
//  utils.h
//  hw1
//
//  Created by Xuezheng Wang on 1/4/22.
//

#pragma once

inline void print_mat4(const glm::mat4 &mtx, const std::string name) {
    std::cout << name << ": " << std::endl;
    std::cout << "[" << mtx[0][0] << "\t" << mtx[1][0] << "\t" << mtx[2][0] << "\t" << mtx[3][0] << "]" << std::endl;
    std::cout << "[" << mtx[0][1] << "\t" << mtx[1][1] << "\t" << mtx[2][1] << "\t" << mtx[3][1] << "]" << std::endl;
    std::cout << "[" << mtx[0][2] << "\t" << mtx[1][2] << "\t" << mtx[2][2] << "\t" << mtx[3][2] << "]" << std::endl;
    std::cout << "[" << mtx[0][3] << "\t" << mtx[1][3] << "\t" << mtx[2][3] << "\t" << mtx[3][3] << "]" << std::endl;
}

Scene* loadFromObj(const std::string &name, const std::string &filepath) {
    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(filepath)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
            throw std::runtime_error("loadFromObj failed");
        }
    }
    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();

    // MARK: Create new scene object, add mesh objects
    Scene* scene = new Scene(name);
    size_t numVerts = 0, numFaces = 0;
    for (const auto& shape : shapes) {
        assert(!shape.name.empty());
        Mesh* newMesh = new Mesh(shape.name, attrib, shape.mesh, tinyobj::material_t());
        scene->objects.insert(std::make_pair(shape.name, newMesh));
        numVerts += static_cast<Mesh*>(scene->objects[shape.name])->verts.size();
        numFaces += static_cast<Mesh*>(scene->objects[shape.name])->faces.size();
    }

    std::cout << "Summary: " << numVerts << " vertices, " << numFaces << " faces. " << std::endl;

    return scene;
}
