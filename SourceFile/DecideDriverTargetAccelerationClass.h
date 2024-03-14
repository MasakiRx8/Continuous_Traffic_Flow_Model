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
	DecideDriverTargetAccelerationClass(const PedalChangePackage* const PedalChange,const ModelBaseClass* const baseClass);	//constructor
	~DecideDriverTargetAccelerationClass();	//destructor

	void DecideDriverTargetAcceleration(const CarStruct* const car);	//Determine the target acceleration of the next time step.
private:
	const PedalChangePackage* const PedalChange;
	const VRecognitionPackage* const VRecognition;
	const GRecognitionPackage* const GRecognition;
	const AvoidCollisionPackage* const AvoidCollision;

	double CalculateNextA(const CarStruct* const car);	//Calculate the target acceleration of the next time step using Eq.(4-12). 

	bool deletedVRecognition;
	bool deletedGRecognition;
	bool deletedAvoidCollision;
};

#endif // !DECIDEDRIVERTARGETACCELERATIONCLASS_H
