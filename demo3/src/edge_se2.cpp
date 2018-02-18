//
// Created by ubuntu on 18-2-16.
//

#include "edge_se2.h"

namespace g2o {

    namespace tutorial {

        // construct
        EdgeSE2::EdgeSE2():BaseBinaryEdge<3,SE2,VertexSE2,VertexSE2>() {
            _measurement = SE2();
            _inverseMeasurement = SE2();
        }

        bool EdgeSE2::read(std::istream& is)
        {
            Eigen::Vector3d p;
            is >> p[0] >> p[1] >> p[2];
            _measurement.fromVector(p);
            _inverseMeasurement = measurement().inverse();
            for (int i = 0; i < 3; ++i)
                for (int j = i; j < 3; ++j) {
                    is >> information()(i, j);
                    if (i != j)
                        information()(j, i) = information()(i, j);
                }
            return true;
        }

        bool EdgeSE2::write(std::ostream& os) const
        {
            Eigen::Vector3d p = measurement().toVector();
            os << p.x() << " " << p.y() << " " << p.z();
            for (int i = 0; i < 3; ++i)
                for (int j = i; j < 3; ++j)
                    os << " " << information()(i, j);
            return os.good();
        }
    }
}
