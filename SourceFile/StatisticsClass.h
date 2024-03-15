/*
	This is header file of the class of "StatisticsClass" that manages local and global statistical information.
	This class can be access to private parameters of class "StatisticsElementsClass" and "StatisticsElementsArray".
*/

#ifndef STATISTICSCLASS_H
#define STATISTICSCLASS_H
#include "Common.h"
#include "StatisticsElementsClass.h"
#include "StatisticsElementsArray.h"
#include "StatisticsParametersClass.h"

class StatisticsClass {
public:
	StatisticsClass(const int& N, const double& globalK, const StatisticsParametersClass& StatisticsParameters);	//constructor
	~StatisticsClass();	//destructor

	StatisticsElementsClass* const Global;
	StatisticsElementsArray* const Local;

	void Reset();
	void IncrementCounter();
	void AddMeasurementSectionTransitTime(const double& dT);
	void AddGlobal_dX(const double& gloval_dX);
	void CalculateAndAddLocalStatistics();
	void CalculateAndSetGlobalStatistics();
private:
	int counter;
	double sumMeasurementSectionTransitTime;
	double sumGlobal_dX;
	const int N;
	const StatisticsParametersClass& StatisticsParameters;

	int addingNumber;
	double localK;
	const double globalK;
	double localAverageVelocity;
	double globallAverageVelocity;
};

#endif // !STATISTICSCLASS_H
