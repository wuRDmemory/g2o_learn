//
// Created by ubuntu on 18-2-14.
//

#ifndef DEMO3_VERTEX_POINT_XY_H
#define DEMO3_VERTEX_POINT_XY_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o_tutorial_slam2d_api.h"
#include <Eigen/Core>


namespace g2o {
    //
    namespace tutorial {
        //
        class G2O_TUTORIAL_SLAM2D_API VertexPointXY : public BaseVertex<2, Eigen::Vector2d>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            // constrution
            VertexPointXY();
            // set the initial status
            virtual void setToOriginImpl() {
                _estimate.setZero();
            }
            // iterate status
            virtual void oplusImpl(const double* update)
            {
                _estimate[0] += update[0];
                _estimate[1] += update[1];
            }

            virtual bool read(std::istream& is);
            virtual bool write(std::ostream& os) const;

        };
    }
}


#endif //DEMO3_VERTEX_POINT_XY_H
