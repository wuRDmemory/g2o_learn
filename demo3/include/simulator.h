//
// Created by ubuntu on 18-2-17.
//

#ifndef DEMO3_SIMULATOR_H
#define DEMO3_SIMULATOR_H

#include "se2.h"
#include "g2o_tutorial_slam2d_api.h"

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <vector>
#include <map>

namespace g2o {

    namespace tutorial {
        // new class
        class G2O_TUTORIAL_SLAM2D_API Simulator {
        public:
            // enum of motion
            enum G2O_TUTORIAL_SLAM2D_API MotionType {
                MO_LEFT,
                MO_RIGHT,
                MO_NUM_ELEMS,
            };// end enum

            /**
             * \bref simulated landmark structure
             */
            struct G2O_TUTORIAL_SLAM2D_API Landmark {
                // ensure memory aligned
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                // landmark's id
                int id;
                // the true pose of landmark
                Eigen::Vector2d truePose;
                // the pose with noise
                Eigen::Vector2d simulatedPose;
                // record the id of pose which has seen this landmark
                std::vector<int> seenBy;
                // construct
                Landmark():id(-1){}
            };// end of Landmark
            typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> > LandmarkVec;
            typedef std::vector<Landmark*> LandmarkPtrVec;

            /**
             * simulated pose of the robot
             * */
            struct G2O_TUTORIAL_SLAM2D_API GridPose {
                // ensure memory aligned
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                // id of pose
                int id;
                // true pose
                SE2 truePose;
                // pose with noise
                SE2 simulatedPose;
                // landmarks saw by this pose
                LandmarkPtrVec landmarks;
            };// end of Grid Pose
            typedef std::vector<GridPose, Eigen::aligned_allocator<GridPose> > PoseVec;

            /**
             * simulated the data of odometry
             * */
            struct G2O_TUTORIAL_SLAM2D_API GridEdge {
                // ensure memory aligned
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                // the last pose id
                int from;
                // now pose id
                int to;
                // true transform
                SE2 trueTransform;
                // simulated transform
                SE2 simulatedTransform;
                // information matrix
                Eigen::Matrix3d information;
            };// end of Grid edge
            typedef std::vector<GridEdge, Eigen::aligned_allocator<GridEdge> > GridEdgeVec;

            /*
             * \brief landmark constraint
             * */
            struct G2O_TUTORIAL_SLAM2D_API LandmarkEdge {
                // ensure memory aligned
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                // pose id
                int from;
                // landmark id
                int to;
                // true measurement
                Eigen::Vector2d trueMeas;
                // simulated measurement
                Eigen::Vector2d simulatedMeas;
                // information matrix
                Eigen::Matrix2d information;

            };// end of LandmarkEdge
            typedef std::vector<LandmarkEdge, Eigen::aligned_allocator<LandmarkEdge> > LandmarkEdgeVec;

        public:
            // construct
            Simulator();
            ~Simulator();

            void simulate(int numPoses, const SE2& sensorOffset = SE2());

            const PoseVec& poses() const { return _poses; }
            const LandmarkVec& landmarks() const { return _landmarks; }
            const GridEdgeVec& odometry() const { return _odometry; }
            const LandmarkEdgeVec& landmarkObs() const { return _landmarkObs; }

        private:
            PoseVec _poses;
            LandmarkVec _landmarks;
            GridEdgeVec _odometry;
            LandmarkEdgeVec _landmarkObs;

            GridPose generateNewPose(const GridPose& prev, const SE2& trueMotion, const Eigen::Vector2d& transformNoise, double rotNoise);
            SE2 getMotion(int motionDirection, double stepLen);
            SE2 sampleTransformation(const SE2& trueMotion_, const Eigen::Vector2d& transNoise, double rotNoise);
        };// end of Simulator

    }// end tutorial namespace
}// end g2o namespace


class simulator {

};


#endif //DEMO3_SIMULATOR_H
