/*
	This is header file of the class of "ModelBaseClass" that is a base class that is the basis of various classes.
	This class manages all the information necessary for simulation, such as model parameters, statistical information, and information about each vehicle.
	This class is inherited by "InitializerClass", "DecideDriverTargetAccelerationClass", "UpdatePositionClass" and "AdvanceTimeAndMeasureClass".
*/

#include "ModelBaseClass.h"

/*
	This constructor is only called by "AdvanceTimeAndMeasureClass".
*/
ModelBaseClass::ModelBaseClass(const int& N, const ModelParametersClass& ModelParameters, const StatisticsParametersClass& statisticsParameters)
	: N(N), ModelParameters(ModelParameters), statisticsParameters(statisticsParameters) {
	calledBy = CalledBy::Constructor;
	cars = new std::vector<CarStruct*>(N);
	random = new Random(N);
}

/*
	This copy constructor is called from anything other than "AdvanceTimeAndMeasureClass".
*/
ModelBaseClass::ModelBaseClass(const ModelBaseClass* baseClass) : N(baseClass->N), ModelParameters(baseClass->ModelParameters), statisticsParameters(baseClass->statisticsParameters) {
	calledBy = CalledBy::Others;
	this->cars = baseClass->cars;
	this->random = baseClass->random;
}

//destructor
ModelBaseClass::~ModelBaseClass() {
	switch (calledBy) {
	case ModelBaseClass::Constructor:
		for (int i = 0; i < N; i++) {
			SafeDelete((*cars)[i]);	//delete CarStruct
		}
		SafeDelete(cars);	//delete vector
		SafeDelete(random);	//delete Random
		break;
	case ModelBaseClass::Others:
		break;
	default:
		break;
	}
}
