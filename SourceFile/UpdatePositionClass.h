/*
	This is header file of the class of "UpdatePositionClass" that move the car position by one time step.
	This inherits from "ModelBaseClass".
*/

#ifndef UPDATEPOSITIONCLASS_H
#define UPDATEPOSITIONCLASS_H
#include <fstream>
#include <sstream>
#include "ReadOnlyPropertyClass.h"
#include "PedalChangePackage.h"
#include "ModelBaseClass.h"

class UpdatePositionClass : public ModelBaseClass {
public:
	UpdatePositionClass(StatisticsClass* const statistics, const PedalChangePackage* const PedalChange, const ModelBaseClass* const baseClass);	//constructor
	~UpdatePositionClass();	//destructor
	
	void UpdateCarPosition(const CarStruct* const car);	//Move the car position by one time step.
private:
	StatisticsClass* const statistics;
	const PedalChangePackage* const PedalChange;
	double _dX;
	double _position;

	void DecideNextCarAcceleration(const CarStruct* const car) const;	//Determine the car's actual acceleration for the next timestep.
	double GetElapsedTime(const CarStruct* const car, const double& x0, const double& x1) const;

	void InitializeProperties(UpdatePositionClass* const thisPtr);

	const double& Get_dX() const;
	const double& Get_Position() const;
public:
	ReadOnlyPropertyClass<const double&> dX;
	ReadOnlyPropertyClass<const double&> Position;
};

#endif // !UPDATEPOSITIONCLASS_H
