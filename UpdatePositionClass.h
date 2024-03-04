/*
	This is header file of the class of "UpdatePositionClass" that move the car position by one time step.
	This inherits from "ModelBaseClass".
*/

#ifndef UPDATEPOSITIONCLASS_H
#define UPDATEPOSITIONCLASS_H
#include "ReadOnlyPropertyClass.h"
#include "PedalChangePackage.h"
#include "ModelBaseClass.h"

class UpdatePositionClass : public ModelBaseClass {
public:
	UpdatePositionClass(StatisticsClass* const statistics, const ModelBaseClass* const baseClass);	//constructor
	~UpdatePositionClass();	//destructor
	
	void UpdateCarPosition();	//Move the car position by one time step.
private:
	StatisticsClass* const statistics;
	const PedalChangePackage* PedalChange;
	double _global_dX;

	void DecideNextCarAcceleration(const CarStruct* const car) const;	//Determine the car's actual acceleration for the next timestep.
	double GetElapsedTime(const CarStruct* const car, const double& x0, const double& x1) const;

	void InitializeProperties(UpdatePositionClass* const thisPtr);

	double Get_Global_dX() const;
public:
	ReadOnlyPropertyClass<double> Global_dX;
};

#endif // !UPDATEPOSITIONCLASS_H