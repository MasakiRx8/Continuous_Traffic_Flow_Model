/*
	This is cpp file of the class of "StatisticsParametersClass" that reads model parameters related to statistical information acquisition from a ".ini" file and provides them.
*/

#include "StatisticsParametersClass.h"

/*
	This constructor reads model parameters related to statistical information acquisition from a ".ini" file and initialize them.
*/
StatisticsParametersClass::StatisticsParametersClass(const std::string& iniFilePath) {
	InitializeProperties(this);
	ReadIniFilePackage ReadIniFile = ReadIniFilePackage(iniFilePath);
	ReadIniFile.ReadIni("Statistics Parameters", "Unit Measurement Time", _unitMeasurementTime);
	ReadIniFile.ReadIni("Statistics Parameters", "Number Of Measurements", _numberOfMeasurements);
	ReadIniFile.ReadIni("Statistics Parameters", "Measurement Length", _measurementLength);
	ReadIniFile.ReadIni("Statistics Parameters", "Measurement Start X", _measurementStartX);
	_measurementEndX = _measurementStartX + _measurementLength;
}

void StatisticsParametersClass::InitializeProperties(StatisticsParametersClass* const thisPtr) {
	UnitMeasurementTime(std::bind(&StatisticsParametersClass::Get_UnitMeasurementTime, thisPtr));
	NumberOfMeasurements(std::bind(&StatisticsParametersClass::Get_NumberOfMeasurements, thisPtr));
	MeasurementStartX(std::bind(&StatisticsParametersClass::Get_MeasurementStartX, thisPtr));
	MeasurementLength(std::bind(&StatisticsParametersClass::Get_MeasurementLength, thisPtr));
	MeasurementEndX(std::bind(&StatisticsParametersClass::Get_MeasurementEndX, thisPtr));
}

const int& StatisticsParametersClass::Get_UnitMeasurementTime() const {
	return _unitMeasurementTime;
}

const int& StatisticsParametersClass::Get_NumberOfMeasurements() const {
	return _numberOfMeasurements;
}

const double& StatisticsParametersClass::Get_MeasurementLength() const {
	return _measurementLength;
}

const double& StatisticsParametersClass::Get_MeasurementStartX() const {
	return _measurementStartX;
}

const double& StatisticsParametersClass::Get_MeasurementEndX() const {
	return _measurementEndX;
}
