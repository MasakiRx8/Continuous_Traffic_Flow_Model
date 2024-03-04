/*
	This is header file of the class of "VRecognitionPackage" that calculates variables related to speed recognition.
*/

#ifndef VRECOGNITIONPACKAGE_H
#define VRECOGNITIONPACKAGE_H
#include <algorithm>
#include <cmath>
#include "CarStruct.h"

class VRecognitionPackage {
public:
	VRecognitionPackage();	//constructor
	~VRecognitionPackage();	//destructor

	void CalculateVSerise(const double& fg, const CarStruct* const car) const;	//Calculate v_target of Eq.(4-11). In addtion to calculate delta v of Eq.(3-1) in both cases regardless of the magnitude relationship between v_target(t) and vStattisticsElements(t).
	double Calculate_Zv(const CarStruct* const car) const;	//Calculated by Eq.(4-6).
	double Calculate_fv(const CarStruct* const car) const;	//Calculated by Eq.(3-1).
private:
	double GetNv(const CarStruct* const car, const double& v) const;	//Calculate Nv(delta vStattisticsElements(t)) of Eq.(3-1). Be careful the argument is the current velocity, not delta v.
};

#endif // !VRECOGNITIONPACKAGE_H
