#pragma once

#include "phys/phys_general.h"

namespace pe_phys_fracture {

    pe::Matrix3 from_two_vectors(const pe::Vector3& a, const pe::Vector3& b);

    void calc_tet_bounding_sphere(const pe::Vector3& v1, const pe::Vector3& v2,
                                  const pe::Vector3& v3, const pe::Vector3& v4,
                                  pe::Vector3& center, pe::Real& radius);
    bool calc_line_plane_intersection(const pe::Vector3& p, const pe::Vector3& n,
                                      const pe::Vector3& v1, const pe::Vector3& v2,
                                      pe::Vector3& inter, pe::Real& t);

    bool is_point_upside_plane(const pe::Vector3& p, const pe::Vector3& n, const pe::Vector3& v);
    bool is_point_on_plane(const pe::Vector3& p, const pe::Vector3& n, const pe::Vector3& v);
    bool is_point_inside_sphere(const pe::Vector3& p, const pe::Vector3& center, pe::Real radius);
    bool are_points_collinear(const pe::Vector3& p1, const pe::Vector3& p2, const pe::Vector3& p3);

    template <typename T>
    void sort3(T& a, T& b, T& c) {
        if (a > b) std::swap(a, b);
        if (b > c) std::swap(b, c);
        if (a > b) std::swap(a, b);
    }

} // namespace pe_phys_fracture