//
// Created by arjun on 1/31/2021.
//
#include <iostream>
#include <cmath>

#ifndef KLAUBANAVIGATIONFILES_CHANHOAPRROXIMATOR_H
#define KLAUBANAVIGATIONFILES_CHANHOAPRROXIMATOR_H

class ChanHoApproximator {
public:
    constexpr static const double c = 0.299792458;
    double Xbase, Ybase, Xone, Yone, Xtwo, Ytwo;
    double* KMatrix;
    double** anchorCoordinateDifferences;
    ChanHoApproximator(double Xbase, double Ybase, double Xone, double Yone, double Xtwo, double Ytwo) {
        this->Xbase = Xbase;
        this->Ybase = Ybase;
        this->Xone = Xone;
        this->Xtwo = Xtwo;
        this->Yone = Yone;
        this->Ytwo = Ytwo;

        generateKMatrix();
        generateAnchorCoordinateDifferenceMatrix();
    }
    double* calculateLocation(double TDoAone, double TDoAtwo) {
        double** coordinatesInTermsOfR = coordinatesInTermsOfRange(TDoAone, TDoAtwo);

        double a = coordinatesInTermsOfR[0][0];
        double b = coordinatesInTermsOfR[1][0];
        double c = coordinatesInTermsOfR[0][1];
        double d = coordinatesInTermsOfR[1][1];

        double range_squared_coefficient = a*a + c*c - 1;
        double range_coefficient = -2*Xbase*a - 2*Ybase*c + 2*a*b + 2*c*d;
        double range_constant = KMatrix[0] - 2*Xbase*b - 2*Ybase*d + b*b + d*d;

        double discriminant = range_coefficient*range_coefficient - 4*range_squared_coefficient*range_constant;

        try {
            if(discriminant < 0) throw 'i';
            discriminant = std::sqrt(discriminant);

            double rangeOne = (-1*range_coefficient + discriminant)/(2*range_squared_coefficient);
            double rangeTwo = (-1*range_coefficient - discriminant)/(2*range_squared_coefficient);

            if(rangeOne > 0 && rangeTwo > 0) {
                rangeOne = std::min(rangeOne, rangeTwo);
            } else rangeOne = std::max(rangeOne, rangeTwo);

            double* locationFromRange = new double[2];

            locationFromRange[0] = coordinatesInTermsOfR[0][0]*rangeOne + coordinatesInTermsOfR[1][0];
            locationFromRange[1] = coordinatesInTermsOfR[0][1]*rangeOne + coordinatesInTermsOfR[1][1];

            return locationFromRange;
        } catch (char error) {
            std::cout << error << std::endl;
        }
    }
private:
    void generateAnchorCoordinateDifferenceMatrix() {
        anchorCoordinateDifferences = new double*[2];

        anchorCoordinateDifferences[0] = new double[2];
        anchorCoordinateDifferences[1] = new double[2];
        anchorCoordinateDifferences[0][0] = Xone - Xbase;
        anchorCoordinateDifferences[0][1] = Yone - Ybase;
        anchorCoordinateDifferences[1][0] = Xtwo - Xbase;
        anchorCoordinateDifferences[1][1] = Ytwo - Ybase;

        double determinant = 1/((anchorCoordinateDifferences[0][0]*anchorCoordinateDifferences[1][1]) - (anchorCoordinateDifferences[0][1]*anchorCoordinateDifferences[1][0]));
        double d = anchorCoordinateDifferences[1][1];

        anchorCoordinateDifferences[0][1] *= determinant;
        anchorCoordinateDifferences[1][0] *= determinant;
        anchorCoordinateDifferences[1][1] = anchorCoordinateDifferences[0][0]*determinant*-1;
        anchorCoordinateDifferences[0][0] = d * determinant*-1;
    }

    double* getRanges(double TDoAone, double TDoAtwo) {
        auto* ranges = new double[2];
        ranges[0] = TDoAone * c;
        ranges[1] = TDoAtwo * c;
        return ranges;
    }

    double* generateRangeKMatrix(double* ranges) {
        auto* rangeKMatrix = new double[2];
        rangeKMatrix[0] = 0.5 * (ranges[0]*ranges[0] - KMatrix[1] + KMatrix[0]);
        rangeKMatrix[1] = 0.5 * (ranges[1]*ranges[1] - KMatrix[2] + KMatrix[0]);
        return rangeKMatrix;
    }

    void generateKMatrix() {
        KMatrix = new double[3];
        KMatrix[0] = Xbase*Xbase + Ybase*Ybase;
        KMatrix[1] = Xone*Xone + Yone*Yone;
        KMatrix[2] = Xtwo*Xtwo + Ytwo*Ytwo;
    }

    double** coordinatesInTermsOfRange(double TDoAone, double TDoAtwo) {
        auto** coordinatesInTermsOfRange = new double*[2];

        auto* rangeCoefficient = new double[2];
        auto* rangeConstant = new double[2];

        double* ranges = getRanges(TDoAone, TDoAtwo);
        double* rangeKMatrix = generateRangeKMatrix(ranges);

        rangeCoefficient[0] = anchorCoordinateDifferences[0][0]*ranges[0] + anchorCoordinateDifferences[0][1]*ranges[1];
        rangeCoefficient[1] = anchorCoordinateDifferences[1][0]*ranges[0] + anchorCoordinateDifferences[1][1]*ranges[1];

        rangeConstant[0] = anchorCoordinateDifferences[0][0]*rangeKMatrix[0] + anchorCoordinateDifferences[0][1]*rangeKMatrix[1];
        rangeConstant[1] = anchorCoordinateDifferences[1][0]*rangeKMatrix[0] + anchorCoordinateDifferences[1][1]*rangeKMatrix[1];

        coordinatesInTermsOfRange[0] = rangeCoefficient;
        coordinatesInTermsOfRange[1] = rangeConstant;

        return coordinatesInTermsOfRange;
    }
};

#endif //KLAUBANAVIGATIONFILES_CHANHOAPRROXIMATOR_H
