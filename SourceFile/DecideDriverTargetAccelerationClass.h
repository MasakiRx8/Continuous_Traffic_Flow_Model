/*
	This is header file of the class of "DecideDriverTargetAccelerationClass" that determine the next timestep acceleration for each driver.
	This inherits from "ModelBaseClass".
*/

#ifndef DECIDEDRIVERTARGETACCELERATIONCLASS_H
#define DECIDEDRIVERTARGETACCELERATIONCLASS_H
#include "ModelBaseClass.h"
#include "VRecognitionPackage.h"
#include "GRecognitionPackage.h"
#include "AvoidCollisionPackage.h"

class DecideDriverTargetAccelerationClass : public ModelBaseClass {
public:
	DecideDriverTargetAccelerationClass(const ModelBaseClass* const baseClass);	//constructor
	~DecideDriverTargetAccelerationClass();	//destructor

	void DecideDriverTargetAcceleration(const CarStruct* const car);	//Determine the target acceleration of the next time step.
private:
	const VRecognitionPackage* VRecognition;
	const GRecognitionPackage* GRecognition;
	const PedalChangePackage* PedalChange;
	const AvoidCollisionPackage* AvoidCollision;

	double CalculateNextA(const CarStruct* const car);	//Calculate the target acceleration of the next time step using Eq.(4-12). 
};

#endif // !DECIDEDRIVERTARGETACCELERATIONCLASS_H
