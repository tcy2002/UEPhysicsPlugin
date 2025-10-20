#pragma once

#include "voronoi_calculator.h"

namespace pe_phys_fracture  {

    class BowyerWatsonVoronoiCalculator : public VoronoiCalculator {
    protected:
        void add_bounding_box(const pe::Array<pe::KV<pe::Vector3, bool>>& points);
        void remove_bounding_box();
        void generate(const pe::Array<pe::KV<pe::Vector3, bool>>& points);

    public:
        BowyerWatsonVoronoiCalculator() {}
        virtual ~BowyerWatsonVoronoiCalculator() {}

        PE_API virtual void triangulate(const pe::Array<pe::KV<pe::Vector3, bool>>& points) override;
    };

} // namespace pe_phys_fracture