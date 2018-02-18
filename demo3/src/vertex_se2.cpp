//
// Created by ubuntu on 18-2-14.
//

#include "vertex_se2.h"

namespace g2o {
    //
    namespace tutorial {
        //
        VertexSE2::VertexSE2():BaseVertex<3, SE2>(){

        }

        bool VertexSE2::read(std::istream& is)
        {
            Eigen::Vector3d p;
            is >> p[0] >> p[1] >> p[2];
            _estimate.fromVector(p);
            return true;
        }

        bool VertexSE2::write(std::ostream& os) const
        {
            Eigen::Vector3d p = estimate().toVector();
            os << p[0] << " " << p[1] << " " << p[2];
            return os.good();
        }
    }
}
