//
// Created by ubuntu on 18-2-14.
//

#ifndef DEMO3_VERTEX_SE2_H
#define DEMO3_VERTEX_SE2_H

#include <Eigen/StdVector>
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.h"
#include "g2o_tutorial_slam2d_api.h"

namespace g2o {

    namespace tutorial {
        // new class
        class G2O_TUTORIAL_SLAM2D_API VertexSE2: public BaseVertex<3, SE2> {
        public:
            // ensure the memory align
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            // construction
            VertexSE2();

            // set status variables to initial status
            void setToOriginImpl() {
                _estimate = SE2();
            }
            // update estimate
            void oplusImpl(const double* update) {
                SE2 up(update[0], update[1], update[2]);
                _estimate *= up;
            }

            virtual bool read(std::istream& is);
            virtual bool write(std::ostream& os) const;

        };
    }
}


#endif //DEMO3_VERTEX_SE2_H
