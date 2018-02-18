//
// Created by ubuntu on 18-2-17.
//

#include "simulator.h"

#include "rand.h"

#include <map>
#include <iostream>
#include <cmath>

using namespace std;

namespace g2o {

    namespace tutorial {
        //
        using namespace Eigen;

        // define the landmark map
        typedef map<int, map<int, Simulator::LandmarkPtrVec> > LandmarkGrid;

        // construct
        Simulator::Simulator() {
            time_t seed = time(0);
            //
            Rand::seed_rand(static_cast<unsigned int>(seed));
        }

        // destroy
        Simulator::~Simulator() {

        }

        void Simulator::simulate(int numPoses, const SE2 &sensorOffset) {
            // simulate a robot observe landmark from map
            int step = 5;
            double stepLen = 1.0;
            int boundArea = 50;

            double maxSensorRangeLandmarks = 2.5*stepLen;

            int landMarksPerSquareMeter = 1;
            double observationProb = 0.8f;

            int landmarksRange = 2;

            Vector2d transNoise(0.05, 0.01);
            double rotNoise = DEG2RAD(2);
            Vector2d landmarkNoise(0.05, 0.05);

            Vector2d bound(boundArea, boundArea);

            VectorXd probLimits;
            probLimits.resize(MO_NUM_ELEMS);
            for (int i = 0; i < probLimits.size(); ++i)
                probLimits[i] = (i + 1) / (double) MO_NUM_ELEMS;

            // 协方差矩阵
            Matrix3d covariance;
            covariance.fill(0);
            covariance(0,0) = transNoise[0]*transNoise[0];
            covariance(1,1) = transNoise[1]*transNoise[1];
            covariance(2,2) = rotNoise*rotNoise;
            // generate the information matrix
            Matrix3d information = covariance.inverse();

            //
            SE2 maxStepTransf(stepLen*step, 0, 0);
            Simulator::PoseVec& poses = _poses;
            poses.clear();
            LandmarkVec& landmarks = _landmarks;
            landmarks.clear();
            // the first pose
            GridPose firstPose;
            firstPose.id = 0;
            firstPose.truePose = SE2(0,0,0);
            firstPose.simulatedPose = SE2(0,0,0);
            poses.push_back(firstPose);
            cerr<<"[Simulator] --> sampling nodes..."<<endl;

            // loops
            while ((int)poses.size() < numPoses) {
                // add straight motions
                for (int i = 1; i < step && poses.size() < numPoses; ++i) {
                    // generate next pose
                    Simulator::GridPose next = generateNewPose(poses.back(), SE2(stepLen, 0, 0), transNoise, rotNoise);
                    // push new pose
                    poses.push_back(next);
                }

                // check numbers of poses
                if (poses.size() == numPoses)
                    break;

                // sample a new motion direction
                double sampleMove = Rand::uniform_rand(0, 1);
                int motionDirection = 0;
                // if the probability great than the random number, set this direction as next motion's direction
                while (probLimits[motionDirection] < sampleMove && motionDirection + 1 < MO_NUM_ELEMS) {
                    motionDirection++;
                }

                //
                SE2 nextMotion = getMotion(motionDirection, stepLen);
                // next Pose
                Simulator::GridPose nextPose = generateNewPose(poses.back(), nextMotion, transNoise, rotNoise);

                // check wether we will walk outside of the boundaries in the next walk
                SE2 nextStepFinalPose = nextPose.truePose * maxStepTransf;
                //
                if (fabs(nextStepFinalPose.toVector()[0] >= bound[0]) ||
                    fabs(nextStepFinalPose.toVector()[1] >= bound[1])) {
                    //
                    for (int i = 0; i < MO_NUM_ELEMS; ++i) {
                        // get next motion direction
                        nextMotion = getMotion(i, stepLen);
                        // get next pose
                        nextPose = generateNewPose(poses.back(), nextMotion, transNoise, rotNoise);
                        // check
                        nextStepFinalPose = nextPose.truePose * maxStepTransf;
                        // check
                        if (fabs(nextStepFinalPose.toVector()[0]) < bound[0] ||
                            fabs(nextStepFinalPose.toVector()[1]) < bound[1]) {
                            // find a direction
                            break;
                        }
                    }

                    poses.push_back(nextPose);
                }
            }

            cerr<<"[Simulate] poses generate done"<<endl;

            cout<<"[Simulate] create landmarks..."<<endl;
            LandmarkGrid grid;
            for (auto it = poses.begin(), end = poses.end(); it!=end; it++) {
                // 虽然不能当作机器人当前的位置，但是如果前后文都是这样的表示就没有问题
                int ccx = (int)round(it->truePose.toVector()[0]);
                int ccy = (int)round(it->truePose.toVector()[1]);
                // 以landmarkRange为半径的圆
                for (int a=-landmarksRange; a<=landmarksRange; a++) {
                    for (int b = -landmarksRange; b <= landmarksRange; b++) {
                        int cx = ccx + a;
                        int cy = ccy + b;
                        // 找出在cx cy处的珊格地图
                        LandmarkPtrVec &landmarksForCell = grid[cx][cy];
                        // 如果是空的，则添加珊格
                        if (landmarksForCell.size() == 0) {
                            Landmark *l = new Landmark();
                            double offx, offy;
                            do {
                                offx = Rand::uniform_rand(-0.5 * stepLen, 0.5 * stepLen);
                                offy = Rand::uniform_rand(-0.5 * stepLen, 0.5 * stepLen);
                            } while (hypot_sqr(offx, offy) < 0.25 * 0.25);
                            l->truePose[0] = cx + offx;
                            l->truePose[1] = cy + offy;
                            landmarksForCell.push_back(l);
                        }
                    }
                }
            }
            // end
            cerr<<"[simulate] create landmark done..."<<endl;

            cout << "[Simulator]  Simulating landmark observations for the poses ... ";
            double maxSensorSqr = maxSensorRangeLandmarks * maxSensorRangeLandmarks;
            int globalId = 0;
            for (PoseVec::iterator it = poses.begin(); it != poses.end(); ++it) {
                Simulator::GridPose& pv = *it;
                int cx = (int)round(it->truePose.translation().x());
                int cy = (int)round(it->truePose.translation().y());
                int numGridCells = (int)(maxSensorRangeLandmarks) + 1;

                pv.id = globalId++;
                SE2 trueInv = pv.truePose.inverse();

                for (int xx = cx - numGridCells; xx <= cx + numGridCells; ++xx) {
                    for (int yy = cy - numGridCells; yy <= cy + numGridCells; ++yy) {
                        LandmarkPtrVec &landmarksForCell = grid[xx][yy];
                        if (landmarksForCell.size() == 0)
                            continue;
                        for (size_t i = 0; i < landmarksForCell.size(); ++i) {
                            Landmark *l = landmarksForCell[i];
                            double dSqr = hypot_sqr(pv.truePose.translation().x() - l->truePose.x(),
                                                    pv.truePose.translation().y() - l->truePose.y());
                            if (dSqr > maxSensorSqr)
                                continue;
                            double obs = Rand::uniform_rand(0.0, 1.0);
                            if (obs > observationProb) // we do not see this one...
                                continue;
                            if (l->id < 0)
                                l->id = globalId++;
                            if (l->seenBy.size() == 0) {
                                // project
                                Vector2d trueObservation = trueInv * l->truePose;
                                Vector2d observation = trueObservation;
                                observation[0] += Rand::gauss_rand(0., landmarkNoise[0]);
                                observation[1] += Rand::gauss_rand(0., landmarkNoise[1]);
                                l->simulatedPose = pv.simulatedPose * observation;
                            }
                            l->seenBy.push_back(pv.id);
                            pv.landmarks.push_back(l);
                        }
                    }
                }
            }
            cerr << "[Simulator] landmark obserse done." << endl;

            // add the odometry measurements
            _odometry.clear();
            cout << "[Simulator]: Adding odometry measurements ... "<<endl;
            for (size_t i = 1; i < poses.size(); ++i) {
                // two pose
                const GridPose& prev = poses[i-1];
                const GridPose& p = poses[i];
                //
                _odometry.push_back(GridEdge());
                GridEdge& edge = _odometry.back();

                edge.from = prev.id;
                edge.to = p.id;
                edge.trueTransform = prev.truePose.inverse() * p.truePose;
                edge.simulatedTransform = prev.simulatedPose.inverse() * p.simulatedPose;
                edge.information = information;
            }
            cerr << "[Simulator] create odometry done." << endl;

            _landmarks.clear();
            _landmarkObs.clear();
            // add the landmark observations
            {
                cout << "[Simulator]  add landmark observations ... ";
                Matrix2d covariance; covariance.fill(0.);
                covariance(0, 0) = landmarkNoise[0]*landmarkNoise[0];
                covariance(1, 1) = landmarkNoise[1]*landmarkNoise[1];
                Matrix2d information = covariance.inverse();

                for (size_t i = 0; i < poses.size(); ++i) {
                    const GridPose& p = poses[i];
                    for (size_t j = 0; j < p.landmarks.size(); ++j) {
                        Landmark* l = p.landmarks[j];
                        if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) {
                            landmarks.push_back(*l);
                        }
                    }
                }

                for (size_t i = 0; i < poses.size(); ++i) {
                    const GridPose& p = poses[i];
                    SE2 trueInv = (p.truePose * sensorOffset).inverse();
                    for (size_t j = 0; j < p.landmarks.size(); ++j) {
                        Landmark* l = p.landmarks[j];
                        Vector2d observation;
                        Vector2d trueObservation = trueInv * l->truePose;
                        observation = trueObservation;
                        if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) { // write the initial position of the landmark
                            observation = (p.simulatedPose * sensorOffset).inverse() * l->simulatedPose;
                        } else {
                            // create observation for the LANDMARK using the true positions
                            observation[0] += Rand::gauss_rand(0., landmarkNoise[0]);
                            observation[1] += Rand::gauss_rand(0., landmarkNoise[1]);
                        }

                        _landmarkObs.push_back(LandmarkEdge());
                        LandmarkEdge& le = _landmarkObs.back();

                        le.from = p.id;
                        le.to = l->id;
                        le.trueMeas = trueObservation;
                        le.simulatedMeas = observation;
                        le.information = information;
                    }
                }
                cerr << "done." << endl;
            }


            // cleaning up
            for (LandmarkGrid::iterator it = grid.begin(); it != grid.end(); ++it) {
                for (std::map<int, Simulator::LandmarkPtrVec>::iterator itt = it->second.begin(); itt != it->second.end(); ++itt) {
                    Simulator::LandmarkPtrVec& landmarks = itt->second;
                    for (size_t i = 0; i < landmarks.size(); ++i)
                        delete landmarks[i];
                }
            }

        }// end of simulate


        /*
         * \brief get next pose from last pose
         * */
        Simulator::GridPose Simulator::generateNewPose(const GridPose &prev, const SE2 &trueMotion,
                                            const Eigen::Vector2d &transformNoise, double rotNoise) {
            // return variable
            Simulator::GridPose nextPose;
            // id plus one
            nextPose.id = prev.id + 1;
            // motion
            nextPose.truePose = prev.truePose * trueMotion;
            // add noise
            SE2 noiseMotion = sampleTransformation(nextPose.truePose, transformNoise, rotNoise);
            // give noise motion attribution to next pose
            nextPose.simulatedPose = noiseMotion;
            //
            return nextPose;
        }// end generateNextPose

        /*
         * \brief generate motion direction and rotation
         * */
        SE2 Simulator::getMotion(int motionDirection, double stepLen) {
            //
            switch (motionDirection) {
                case MO_LEFT:
                    return SE2(stepLen, 0, 0.5*M_PI);
                case MO_RIGHT:
                    return SE2(stepLen, 0, -0.5*M_PI);
                default:
                    cerr<<"[Simulate] --> unknown direction"<<endl;
                    // default direction is right
                    return SE2(stepLen, 0, -0.5*M_PI);
            }
        }


        /*
         * get sample transform from pure transform
         * */
        SE2 Simulator::sampleTransformation(const SE2 &trueMotion_, const Eigen::Vector2d &transNoise,
                                            double rotNoise) {
            // first get the vector
            Vector3d trueMotion = trueMotion_.toVector();
            // add noise to trueMotion
            SE2 noiseMotion(trueMotion[0] + Rand::gauss_rand(0.0, transNoise[0]),
                            trueMotion[1] + Rand::gauss_rand(0.0, transNoise[1]),
                            trueMotion[2] + Rand::gauss_rand(0.0, rotNoise));
            return noiseMotion;
        }

    }// end of tutorial
}// end of g2o
