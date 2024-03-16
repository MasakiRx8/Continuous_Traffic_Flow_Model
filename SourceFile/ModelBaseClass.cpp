/*
	This is header file of the class of "ModelBaseClass" that is a base class that is the basis of various classes.
	This class manages all the information necessary for simulation, such as model parameters, statistical information, and information about each vehicle.
	This class is inherited by "InitializerClass", "DecideDriverTargetAccelerationClass", "UpdatePositionClass" and "AdvanceTimeAndMeasureClass".
*/

#include "ModelBaseClass.h"

/*
	This constructor is only called by "AdvanceTimeAndMeasureClass".
*/
ModelBaseClass::ModelBaseClass(const int& RunNumber, const int& N, const ModelParametersClass& ModelParameters, const StatisticsParametersClass& StatisticsParameters)
	: N(N), ModelParameters(ModelParameters), StatisticsParameters(StatisticsParameters)
	, cars(new std::vector<CarStruct*>(N))
	, random(new Random(RunNumber)) {
	calledBy = CalledBy::Constructor;
	deletedCars = false;
	deletedRandom = false;
}

/*
	This copy constructor is called from anything other than "AdvanceTimeAndMeasureClass".
*/
ModelBaseClass::ModelBaseClass(const ModelBaseClass* const baseClass)
	: N(baseClass->N), ModelParameters(baseClass->ModelParameters), StatisticsParameters(baseClass->StatisticsParameters)
	, cars(baseClass->cars)
	, random(baseClass->random) {
	calledBy = CalledBy::Others;
	deletedCars = true;
	deletedRandom = true;
}

//destructor
ModelBaseClass::~ModelBaseClass() {
	switch (calledBy) {
	case ModelBaseClass::Constructor:
		for (std::size_t i = 0; i < cars->size(); i++) {
			SafeDelete((*cars)[i]);	//delete CarStruct
		}
		if (!deletedCars) {
			delete cars;	//delete vector
			deletedCars = true;
		}
		if (!deletedRandom) {
			delete random;	//delete Random
			deletedRandom = true;
		}
		break;
	case ModelBaseClass::Others:
		break;
	default:
		break;
	}
}
