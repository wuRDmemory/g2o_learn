//
// Created by ubuntu on 18-2-14.
//

#ifndef DEMO3_SE2_H
#define DEMO3_SE2_H

#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h"
#include "g2o_tutorial_slam2d_api.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {
    // new namespace distinguish to default
    namespace tutorial {
        // SE2的类型，注意这个不是边也不是顶点
        class G2O_TUTORIAL_SLAM2D_API SE2 {
        public:
            // ensure the memory align
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            // construction
            SE2():_R(0),_t(0,0){}
            // construction
            SE2(double x, double y, double theta):_R(theta),_t(x,y) {}
            // return _t
            const Eigen::Vector2d& translation() const {return _t;}
            // return _t
            Eigen::Vector2d& translation() {return _t;}
            // return _R
            const Eigen::Rotation2Dd& rotation() const {return _R;}
            // return _R
            Eigen::Rotation2Dd& rotation() {return _R;}
            // multiplication
            SE2 operator*(const SE2& tr2) const {
                SE2 result(*this);
                result._t += _R*tr2._t;
                result._R.angle() += tr2._R.angle();
                result._R.angle() = normalize_theta(result._R.angle());
                return result;
            }
            // multiplication
            SE2& operator*=(const SE2& tr2) {
                _t += _R*tr2._t;
                _R.angle() += tr2._R.angle();
                _R.angle() = normalize_theta(_R.angle());
                return *this;
            }
            // multiplicate with vector
            Eigen::Vector2d operator*(const Eigen::Vector2d& v) const {
                return _t + _R*v;
            }
            // 求逆
            SE2 inverse() const {
                SE2 ret;
                ret._R = _R.inverse();
                ret._R.angle() = normalize_theta(ret._R.angle());
                ret._t = ret._R*(Eigen::Vector2d(-1*_t));
                return ret;
            }
            // 右值
            double operator[](int i) const {
                assert(i>=0 && i<3);
                if(i<2)
                    return _t(i);
                return _R.angle();
            }
            // 左值
            double& operator[](int i) {
                assert(i>=0 && i<3);
                if(i<2)
                    return _t(i);
                return _R.angle();
            }
            // construct from vector
            void fromVector(const Eigen::Vector3d& v) {
                *this = SE2(v[0],v[1],v[2]);
            }
            // convert SE2 to vector
            Eigen::Vector3d toVector() const {
                Eigen::Vector3d ret;
                for (int i=0; i<3; i++){
                    ret(i)=(*this)[i];
                }
                return ret;
            }

        private:
            // 旋转
            Eigen::Rotation2Dd _R;
            // 平移
            Eigen::Vector2d _t;

        };
    }
}

#endif //DEMO3_SE2_