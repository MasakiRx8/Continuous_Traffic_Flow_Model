/*
	This is header file of the class of "PedalChangePackage" that performs calculations related to pedal changes.
*/

#ifndef PEDALCHANGEPACKAGE_H
#define PEDALCHANGEPACKAGE_H

#include "Common.h"
#include "CarStruct.h"

class PedalChangePackage {
public:
	PedalChangePackage(const double& deltaT);	//constructor
	~PedalChangePackage();	//destructor

	double GetAccelToBrakeTime(const CarStruct* const car, const double& v) const;	//Calculated by Eq.(3-4).
	double GetBrakeToAccelTime(const CarStruct* const car, const double& v) const;	//Calculated by Eq.(3-4).
	void UpdatePedalChangingInformations(const CarStruct* const car, const double& nextA) const;	//Update information such as the time required to switch a pedal.
	PedalChangedStateType DoPedalChange(DriverElements::MomentValuesElements::PedalInformations* const pedal, const bool& recognitionHit) const;	//Pedal switching execution.
private:
	const double deltaT;
};

#endif // !PEDALCHANGEPACKAGE_H
