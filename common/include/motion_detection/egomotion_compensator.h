/* egomotion_compensator.h
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */
#ifndef EGOMOTION_COMPENSATOR_H_
#define EGOMOTION_COMPENSATOR_H_

class EgomotionCompensator
{
    public:
        EgomotionCompensator();
        virtual ~EgomotionCompensator();

        void calculateCompensatedVectors(const cv::Mat &optical_flow_vectors, const std::vector<double> &odom, cv::Mat &compensated_vectors, int pixel_step);            

    private:
        void getFOE(const std::vector<double> &odom, double &foe_x, double &foe_y);
        double getExpectedOrientation(double x, double y, double foe_x, double foe_y);

};

#endif
