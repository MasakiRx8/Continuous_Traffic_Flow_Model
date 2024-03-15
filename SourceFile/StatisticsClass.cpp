/*
	This is cpp file of the class of "StatisticsClass" that manages local and global statistical information.
	This class can be access to private parameters of class "StatisticsElementsClass" and "StatisticsElementsArray".
*/

#include "StatisticsClass.h"

//constructor
StatisticsClass::StatisticsClass(const int& N, const double& globalK, const StatisticsParametersClass& StatisticsParameters)
	: N(N), globalK(globalK), StatisticsParameters(StatisticsParameters)
	, Global(new StatisticsElementsClass()), Local(new StatisticsElementsArray(std::size_t(StatisticsParameters.NumberOfMeasurements))) {
	addingNumber = 0;
	sumGlobal_dX = 0;
	Reset();
}

//destructor
StatisticsClass::~StatisticsClass() {
	if (Global != nullptr) {
		delete Global;
	}
	if (Local != nullptr) {
		delete Local;
	}
}

void StatisticsClass::Reset() {
	counter = 0;
	sumMeasurementSectionTransitTime = 0;
}

void StatisticsClass::IncrementCounter() {
	counter++;
}

void StatisticsClass::AddMeasurementSectionTransitTime(const double& dT) {
	sumMeasurementSectionTransitTime += dT;
}

void StatisticsClass::AddGlobal_dX(const double& gloval_dX) {
	sumGlobal_dX += gloval_dX;
}

void StatisticsClass::CalculateAndAddLocalStatistics() {
	double av;
	double k;
	if (counter == 0) {
		av = 0;
		k = 0;
	}
	else {
		av = StatisticsParameters.MeasurementLength * counter / sumMeasurementSectionTransitTime;
		k = 1000 / StatisticsParameters.UnitMeasurementTime * counter / av;
	}
	StatisticsElementsClass* const local = (*Local->vStattisticsElements)[addingNumber];
	local->_counter = counter;
	local->_k = k;
	local->_averageVelocity = av;
	addingNumber++;
}

void StatisticsClass::CalculateAndSetGlobalStatistics() {
	Global->_counter = 0;
	Global->_k = globalK;
	Global->_averageVelocity = sumGlobal_dX / (StatisticsParameters.NumberOfMeasurements * StatisticsParameters.UnitMeasurementTime * N);
}
