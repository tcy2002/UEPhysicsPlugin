#pragma once

#include "phys/fracture/fracture_utils/fracture_data_manager.h"
#include "phys/phys_general.h"
#include "utils/hash_vector.h"
#include "phys/fracture/fracture_utils/fracture_utils.h"
#include <algorithm>

namespace pe_phys_fracture  {

    class VoronoiCalculator {
    protected:
        FractureDataManager _manager;
        pe::Array<pe::Uint32HashList> _adjacency_list;
        void calc_adjacency_list();
        pe::Array<bool> _inside_flags;

    public:
        VoronoiCalculator() {}
        virtual ~VoronoiCalculator() {}

        virtual void triangulate(const pe::Array<pe::KV<pe::Vector3, bool>>& points) = 0;
        uint32_t point_count() const { return _manager.vertex_count(); }
        pe::Vector3 get_point(uint32_t idx, bool& inside) { inside = _inside_flags[idx];  return _manager.get_vertex(idx).pos; }
        pe::Array<uint32_t> get_adjacent_points(uint32_t idx) { return _adjacency_list[idx].to_vector(); }
        void reset() { _adjacency_list.clear(); _manager.clear(); }
    };

} // namespace pe_phys_fracture