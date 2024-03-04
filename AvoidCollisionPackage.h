/*
	This is header file of the class of "AvoidCollisionPackage" that is for avoiding collisions.
*/

#ifndef AVOIDCOLLISIONPACKAGE_H
#define AVOIDCOLLISIONPACKAGE_H
#include "CarStruct.h"
#include "PedalChangePackage.h"

class AvoidCollisionPackage {
public:
	AvoidCollisionPackage(const double& deltaT);	//constructor
	~AvoidCollisionPackage();	//destructor

	bool IsEmergency(const CarStruct* const car) const;	//Judged by Eq.(4-9).
	double GetEmergencyAcceleration(const CarStruct* const car) const;	//Calculate a_emergency of Eq.(4-12).
private:
	const double deltaT;
	const PedalChangePackage* PedalChange;
};

#endif // !AVOIDCOLLISIONPACKAGE_H
