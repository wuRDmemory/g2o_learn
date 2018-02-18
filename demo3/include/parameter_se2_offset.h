//
// Created by ubuntu on 18-2-16.
//

#ifndef DEMO3_PARAMETER_SE2_OFFSET_H
#define DEMO3_PARAMETER_SE2_OFFSET_H

#include "g2o/core/cache.h"
#include "se2.h"
#include "g2o_tutorial_slam2d_api.h"

namespace g2o {

    namespace tutorial {

        // new class
        class G2O_TUTORIAL_SLAM2D_API ParameterSE2Offset:public Parameter {
        public:
            // ensure memory aligned
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            // construct
            ParameterSE2Offset();
            // set offset
            void setOffset(const SE2& offset = SE2());
            // return offset
            const SE2& offset() const {return _offset;}
            // inverse of offset
            const SE2& inverseOffset() const { return _inverseOffset;}
            // read and write
            virtual bool read(std::istream& is);
            virtual bool write(std::ostream& os) const;

        private:
            SE2 _offset;
            SE2 _inverseOffset;
        };


        // new class
        class G2O_TUTORIAL_SLAM2D_API CacheSE2Offset:public Cache {
        public:
            // ensure memory aligned
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            // return w2n n2w
            const SE2& w2n() const {return _w2n;}
            const SE2& n2w() const {return _n2w;}

        protected:
            virtual void updateImpl();
            virtual bool resolveDependancies();

            ParameterSE2Offset* _offsetParam;
            SE2 _w2n,_n2w;
        };
    }
}


#endif //DEMO3_PARAMETER_SE2_OFFSET_H
