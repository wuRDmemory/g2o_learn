//
// Created by ubuntu on 18-2-17.
//

#ifndef DEMO3_RAND_H
#define DEMO3_RAND_H

#include <cstdlib>
#include <cmath>
#include <ctime>

#include "g2o_tutorial_slam2d_api.h"

namespace g2o {

    namespace tutorial {

        /**
         * \brief generate random numbers
         */
        class G2O_TUTORIAL_SLAM2D_API Rand
        {
        public:
            /**
             * Gaussian random with a mean and standard deviation. Uses the
             * Polar method of Marsaglia.
             */
            static double gauss_rand(double mean, double sigma)
            {
                double x, y, r2;
                do {
                    x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
                    y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
                    r2 = x * x + y * y;
                } while (r2 > 1.0 || r2 == 0.0);
                return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
            }

            /**
             * sample a number from a uniform distribution
             */
            static double uniform_rand(double lowerBndr, double upperBndr)
            {
                return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
            }

            /**
             * default seed function using the current time in seconds
             */
            static void seed_rand()
            {
                seed_rand(static_cast<unsigned int>(std::time(NULL)));
            }

            /** seed the random number generator */
            static void seed_rand(unsigned int seed)
            {
                std::srand(seed);
            }
        };

    } // end namespace
} // end namespace

#endif //DEMO3_RAND_H
