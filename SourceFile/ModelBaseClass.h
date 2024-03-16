/*
	This is header file of the class of "ModelBaseClass" that is a base class that is the basis of various classes.
	This class manages all the information necessary for simulation, such as model parameters, statistical information, and information about each vehicle.
	This class is inherited by "InitializerClass", "DecideDriverTargetAccelerationClass", "UpdatePositionClass" and "AdvanceTimeAndMeasureClass".
*/

#ifndef MODELBASECLASS_H
#define MODELBASECLASS_H
#include <iostream>
#include <string>
#include <vector>
#include "random.h"
#include "Common.h"
#include "ModelParametersClass.h"
#include "StatisticsParametersClass.h"
#include "CarStruct.h"
#include "DriverStruct.h"
#include "StatisticsClass.h"

class ModelBaseClass {
public:
	ModelBaseClass(const int& RunNumber, const int& N, const ModelParametersClass& ModelParameters, const StatisticsParametersClass& StatisticsParameters);	//This constructor is only called by "AdvanceTimeAndMeasureClass".
	ModelBaseClass(const ModelBaseClass* const baseClass);	//This copy constructor is called from anything other than "AdvanceTimeAndMeasureClass".
	~ModelBaseClass();	//destructor
protected:
	const int N;
	const ModelParametersClass& ModelParameters;
	const StatisticsParametersClass& StatisticsParameters;
	std::vector<CarStruct*>* const cars;
	const Random* const random;
private:
	enum CalledBy {
		Constructor
		, Others
	};
	CalledBy calledBy;

	bool deletedCars;
	bool deletedRandom;
};

#endif // !MODELBASECLASS_H
