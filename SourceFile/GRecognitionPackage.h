/*
	This is header file of the class of "GRecognitionPackage" that calculates variables related to inter-vehicular distance.
*/

#ifndef GRECOGNITIONPACKAGE_H
#define GRECOGNITIONPACKAGE_H
#include <algorithm>
#include "CarStruct.h"
#include "PedalChangePackage.h"

class GRecognitionPackage {
public:
	GRecognitionPackage(const double& deltaT, const double& L, const PedalChangePackage* const PedalChange);	//constructor
	~GRecognitionPackage();	//destructor

	void CalculateGSerise(const CarStruct* const car) const;	//Calculated by Eq.(3-5) to (3-7).
	double Calculate_Zg(const CarStruct* const car) const;	//Calculated by Eq.(4-6).
	double Calculate_fg(const CarStruct* const car) const;	//Calculated by Eq.(3-8).
private:
	const double deltaT;
	const double L;
	const PedalChangePackage* const PedalChange;

	double GetNg(const CarElements::MomentValuesElements::GapSerise* const g) const;	//Calculate Ng(delta g(t)) of Eq.(3-8).
	double GetTMargin(const CarStruct* const car) const;	//Calculate t_margin(v(t)) of Eq.(3-7).
};

#endif // !GRECOGNITIONPACKAGE_H
