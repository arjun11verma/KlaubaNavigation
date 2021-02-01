/*
 * Created by Mihir Kasmalkar
 * Date: 2/1/2021
 */

#include "ChanHoApproximator.h"
#include <iostream>

#ifndef KLAUBANAVIGATIONFILES_HARBIAPPROXIMATOR_H
#define KLAUBANAVIGATIONFILES_HARBIAPPROXIMATOR_H

class HarbiApproximator {
public:
    constexpr static const double c = 0.299792458;

    double anchorOneX, anchorOneY;
    double anchorTwoX, anchorTwoY;
    double anchorThreeX, anchorThreeY;

    HarbiApproximator(double anchorOneX, double anchorOneY,
        double anchorTwoX, double anchorTwoY,
        double anchorThreeX, double anchorThreeY) {

        this->anchorOneX = anchorOneX;
        this->anchorOneY = anchorOneY;
        this->anchorTwoX = anchorTwoX;
        this->anchorTwoY = anchorTwoY;
        this->anchorThreeX = anchorThreeX;
        this->anchorThreeY = anchorThreeY;
    }

    double* calculateLocation(double TDoATwo, double TDoAThree) {
        // Anchor 1: TDOA = 0
        // Anchor 2: TDOA = TDoATwo
        // Anchor 3: TDOA = TDoAThree
        
        // Base anchor: Anchor 1
        // Anchor 2: Anchor 2
        // Anchor 3: Anchor 3

        ChanHoApproximator chanHoOne(anchorOneX, anchorOneY, anchorTwoX, anchorTwoY, anchorThreeX, anchorThreeY);
        double* locationEstimateOne = chanHoOne.calculateLocation(TDoATwo, TDoAThree);
        double R1 = chanHoOne.R1;
        
        // Anchor 1: TDOA = -TDoATwo
        // Anchor 2: TDOA = 0
        // Anchor 3: TDOA = TDoAThree - TDoATwo

        // Base anchor: Anchor 2
        // Anchor 2: Anchor 1
        // Anchor 3: Anchor 3

        double adjustedTDoAOne = 0 - TDoATwo;
        double adjustedTDoAThree = TDoAThree - TDoATwo;

        ChanHoApproximator chanHoTwo(anchorTwoX, anchorTwoY, anchorOneX, anchorOneY, anchorThreeX, anchorThreeY);
        double* locationEstimateTwo = chanHoTwo.calculateLocation(adjustedTDoAOne, adjustedTDoAThree);
        double R2 = chanHoTwo.R1;
        
        double x1 = anchorOneX;
        double y1 = anchorOneY;
        double x2 = anchorTwoX;
        double y2 = anchorTwoY;

        
        double Q[2][2] = {
            {(x1 - locationEstimateOne[0]) / R1, (x2 - locationEstimateTwo[0]) / R2},
            {(y1 - locationEstimateOne[1]) / R1, (y2 - locationEstimateTwo[1]) / R2}
        };

        double Qt[2][2] = {
            {Q[0][0], Q[1][0]},
            {Q[0][1], Q[1][1]}
        };

        double traceQQt = Qt[0][0] * Q[0][0] + Qt[0][1] * Q[1][0]
            + Qt[1][0] * Q[0][1] + Qt[1][1] * Q[1][1];
        
        double omega = sqrt(traceQQt / 2);
        
        double** anchorCoordinateDifferences = chanHoOne.anchorCoordinateDifferences;
        double* ranges = chanHoOne.ranges;
        double* rangeKMatrix = chanHoOne.rangeKMatrix;

        double productMatrix[2];

        for (int i = 0; i < 2; i++) {
            productMatrix[i] = ranges[i] * omega * R1 + rangeKMatrix[i];
        }

        double x = anchorCoordinateDifferences[0][0] * productMatrix[0] + anchorCoordinateDifferences[1][0] * productMatrix[1];
        double y = anchorCoordinateDifferences[1][0] * productMatrix[0] + anchorCoordinateDifferences[1][1] * productMatrix[1];
        
        double* location = new double[2];
        location[0] = x;
        location[1] = y;

        return location;
    }
};

#endif