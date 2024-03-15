/*
	This is header file of the class of "StatisticsParametersClass" that reads model parameters related to statistical information acquisition from a ".ini" file and provides them.
*/

#ifndef STATISTICSPARAMETERSCLASS_H
#define STATISTICSPARAMETERSCLASS_H
#include "ReadIniFilePackage.h"
#include "ReadOnlyPropertyClass.h"

class StatisticsParametersClass {
public:
	StatisticsParametersClass(const std::string& iniFilePath);	//This constructor reads model parameters related to statistical information acquisition from a ".ini" file and initialize them.
private:
	int _unitMeasurementTime;
	int _numberOfMeasurements;
	double _measurementLength;
	double _measurementStartX;
	double _measurementEndX;

	void InitializeProperties(StatisticsParametersClass* const thisPtr);

	const int& Get_UnitMeasurementTime() const;
	const int& Get_NumberOfMeasurements() const;
	const double& Get_MeasurementLength() const;
	const double& Get_MeasurementStartX() const;
	const double& Get_MeasurementEndX() const;
public:
	ReadOnlyPropertyClass<const int&> UnitMeasurementTime;
	ReadOnlyPropertyClass<const int&> NumberOfMeasurements;
	ReadOnlyPropertyClass<const double&> MeasurementLength;
	ReadOnlyPropertyClass<const double&> MeasurementStartX;
	ReadOnlyPropertyClass<const double&> MeasurementEndX;
};

#endif // !STATISTICSPARAMETERSCLASS_H

