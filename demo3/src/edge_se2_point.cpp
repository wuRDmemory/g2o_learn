//
// Created by ubuntu on 18-2-16.
//

#include "edge_se2_point.h"


namespace g2o {

    namespace tutorial {
        // new class
        EdgeSE2PointXY::EdgeSE2PointXY():BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXY>(), _sensorOffset(0), _sensorCache(0) {
            //
            resizeParameters(1);
            //
            installParameter(_sensorOffset, 0);
        }

        bool EdgeSE2PointXY::read(std::istream& is) {
            int paramId;
            is >> paramId;
            if (! setParameterId(0,paramId))
                return false;
            is >> _measurement[0] >> _measurement[1];
            is >> information()(0,0) >> information()(0,1) >> information()(1,1);
            information()(1,0) = information()(0,1);
            return true;
        }

        bool EdgeSE2PointXY::write(std::ostream& os) const {
            os << _sensorOffset->id() << " ";
            os << measurement()[0] << " " << measurement()[1] << " ";
            os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
            return os.good();
        }

        void EdgeSE2PointXY::computeError() {
            // landmark
            const VertexPointXY* l2 = dynamic_cast<const VertexPointXY*>(_vertices[1]);
            //
            _error = (_sensorCache->w2n()*l2->estimate()) - _measurement;
        }

        bool EdgeSE2PointXY::resolveCaches() {
            ParameterVector pv(1);
            pv[0] = _sensorOffset;
            resolveCache(_sensorCache, static_cast<OptimizableGraph::Vertex*>(_vertices[0]), "TUTORIAL_CACHE_SE2_OFFSET", pv);
            return _sensorCache != 0;
        }
    }
}