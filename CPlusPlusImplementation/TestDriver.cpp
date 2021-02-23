#include "HarbiApproximator.h"
#include "ChanHoApproximator.h"
#include <math.h>

#define NOISE_ADDED false

void runTest(double anchorOneX,
	double anchorOneY,
	double anchorTwoX,
	double anchorTwoY,
	double anchorThreeX,
	double anchorThreeY,
	double mobileX,
	double mobileY);

int main() {

	double anchorOneX = 0.0;
	double anchorOneY = 0.0;
	double anchorTwoX = 3.0;
	double anchorTwoY = 0.0;
	double anchorThreeX = 0.0;
	double anchorThreeY = 2.5;

	double mobileX = 3.0;
	double mobileY = 4.0;

	ChanHoApproximator chanHo(anchorOneX, anchorOneY, anchorTwoX, anchorTwoY, anchorThreeX, anchorThreeY);
	double* chanHoEstimate = chanHo.calculateLocation(-0.45, 2.66);

	std::cout << "Chan ho estimate: (" << chanHoEstimate[0] << ", " << chanHoEstimate[1] << ")" << std::endl;
	delete chanHoEstimate;

	//runTest(anchorOneX, anchorOneY, anchorTwoX, anchorTwoY, anchorThreeX, anchorThreeY, mobileX, mobileY);
	
	return 0;
}


void runTest(double anchorOneX,
	double anchorOneY,
	double anchorTwoX,
	double anchorTwoY,
	double anchorThreeX,
	double anchorThreeY,
	double mobileX,
	double mobileY)
{
	constexpr double c = 0.299792458;

	double anchorOneDistToMS = sqrt(pow(anchorOneX - mobileX, 2) + pow(anchorOneY - mobileY, 2));
	
	double anchorTwoDistToMS = sqrt(pow(anchorTwoX - mobileX, 2) + pow(anchorTwoY - mobileY, 2));
	double anchorTwoTDOA = (anchorTwoDistToMS - anchorOneDistToMS) / c;

	double anchorThreeDistToMS = sqrt(pow(anchorThreeX - mobileX, 2) + pow(anchorThreeY - mobileY, 2));
	double anchorThreeTDOA = (anchorThreeDistToMS - anchorOneDistToMS) / c;


	if (NOISE_ADDED) {
		// TODO: add noise
	}

	ChanHoApproximator chanHo(anchorOneX, anchorOneY, anchorTwoX, anchorTwoY, anchorThreeX, anchorThreeY);
	double* chanHoEstimate = chanHo.calculateLocation(anchorTwoTDOA, anchorThreeTDOA);

	std::cout << "Chan ho estimate: (" << chanHoEstimate[0] << ", " << chanHoEstimate[1] << ")" << std::endl;
	delete chanHoEstimate;

	HarbiApproximator harbi(anchorOneX, anchorOneY, anchorTwoX, anchorTwoY, anchorThreeX, anchorThreeY);
	double* locationEstimate = harbi.calculateLocation(anchorTwoTDOA, anchorThreeTDOA);
	std::cout << "Harbi Loc estimate: (" << locationEstimate[0] << ", " << locationEstimate[1] << ")" << std::endl;
	delete locationEstimate;
}