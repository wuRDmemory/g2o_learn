//
// Created by ubuntu on 18-2-14.
//

#include "vertex_point_xy.h"

namespace g2o {

    namespace tutorial {

        VertexPointXY::VertexPointXY():BaseVertex<2, Eigen::Vector2d>()
        {
            _estimate.setZero();
        }

        bool VertexPointXY::read(std::istream& is)
        {
            is >> _estimate[0] >> _estimate[1];
            return true;
        }

        bool VertexPointXY::write(std::ostream& os) const
        {
            os << estimate()(0) << " " << estimate()(1);
            return os.good();
        }

    } // end namespace
} // end namespace
