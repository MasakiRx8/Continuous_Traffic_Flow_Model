/*
	This is cpp file of the class of "StatisticsElementsClass" that stores statistical information.
	This class's private parameters are opened to class "StatisticsClass".
*/

#include "StatisticsElementsClass.h"

//constructor
StatisticsElementsClass::StatisticsElementsClass() {
	InitializeProperties(this);
	_counter = 0;
	_k = 0;
	_averageVelocity = 0;
}

void StatisticsElementsClass::InitializeProperties(StatisticsElementsClass* const thisPtr) {
	Counter(std::bind(&StatisticsElementsClass::Get_Counter, thisPtr));
	K(std::bind(&StatisticsElementsClass::Get_K, thisPtr));
	AverageVelocity(std::bind(&StatisticsElementsClass::Get_AverageVelocity, thisPtr));
}

const int& StatisticsElementsClass::Get_Counter() const {
	return _counter;
}

const double& StatisticsElementsClass::Get_K() const {
	return _k;
}

const double& StatisticsElementsClass::Get_AverageVelocity() const {
	return _averageVelocity;
}
