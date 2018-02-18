//
// Created by ubuntu on 18-2-16.
//

#ifndef DEMO3_EDGE_SE2_H
#define DEMO3_EDGE_SE2_H

#include "vertex_se2.h"
#include "g2o_tutorial_slam2d_api.h"
#include "g2o/core/base_binary_edge.h"


namespace g2o {

    namespace tutorial {
        //
        class G2O_TUTORIAL_SLAM2D_API EdgeSE2:public g2o::BaseBinaryEdge<3,SE2,g2o::tutorial::VertexSE2, g2o::tutorial::VertexSE2> {
        public:
            // ensure memory align
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            // construct
            EdgeSE2();

            // compute error
            virtual void computeError() {
                // get two vertex
                const VertexSE2* v1 = dynamic_cast<VertexSE2*>(_vertices[0]);
                const VertexSE2* v2 = dynamic_cast<VertexSE2*>(_vertices[1]);
                // compute error between measurement and estimate
                SE2 deltaSE2 = _inverseMeasurement*(v1->estimate().inverse()*v2->estimate());
                _error = deltaSE2.toVector();
            }

            // set measurement
            virtual void setMeasurement(const SE2& m) {
                _measurement = m;
                _inverseMeasurement = m.inverse();
            }

            // read and write
            virtual bool read(std::istream& is);
            virtual bool write(std::ostream& os) const;

        private:
            // SE2's inverse
            SE2 _inverseMeasurement;
        };
    }
}


#endif //DEMO3_EDGE_SE2_H
