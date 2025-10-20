#pragma once

#include "vector3.h"
#include <vector>
#include <fstream>
#include <sstream>
#include <atomic>

namespace common {

    template <typename Scalar>
    class Mesh {
    protected:
        static std::atomic<uint32_t> _globalIdCounter;

    public:
        struct Vertex {
            Vector3<Scalar> position;
            Vector3<Scalar> normal;
        };
        struct Face {
            std::vector<uint32_t> indices;
            Vector3<Scalar> normal;
        };


        std::vector<Vertex> vertices;
        std::vector<Face> faces;

        Mesh() {}
        Mesh(std::vector<Vertex> vs, std::vector<Face> fs):
            vertices(std::move(vs)), faces(std::move(fs)) {}

        bool empty() const { return vertices.empty() || faces.empty(); }

        static void perFaceNormal(Mesh<Scalar>& mesh);
        static void perVertexNormal(Mesh<Scalar>& mesh);
        static void loadFromObj(const std::string &filename, Mesh<Scalar> &mesh, const Vector3<Scalar>& size);
        static void saveToObj(const std::string &filename, const Mesh<Scalar> &mesh, const Vector3<Scalar>& size);
    };

    #include "mesh.inl"

} // namespace common