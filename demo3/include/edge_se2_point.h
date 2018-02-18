//
// Created by ubuntu on 18-2-16.
//

#ifndef DEMO3_EDGE_SE2_POINT_H
#define DEMO3_EDGE_SE2_POINT_H

#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "g2o_tutorial_slam2d_api.h"
#include "parameter_se2_offset.h"

#include "g2o/core/base_binary_edge.h"

namespace g2o {

    namespace tutorial {

        class ParameterSE2Offset;
        class CacheSE2Offset;

        // new edge type
        class G2O_TUTORIAL_SLAM2D_API EdgeSE2PointXY:public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXY> {
        public:
            // ensure memory aligned
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            // construct
            EdgeSE2PointXY();
            // compute error between pose and point
            virtual void computeError();
            //
            virtual bool read(std::istream& is);
            virtual bool write(std::ostream& os) const;

        protected:
            ParameterSE2Offset* _sensorOffset;
            CacheSE2Offset* _sensorCache;

            virtual bool resolveCaches();
        };
    }
}


#endif //DEMO3_EDGE_SE2_POINT_H
