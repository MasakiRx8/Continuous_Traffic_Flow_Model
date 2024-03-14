/*
	This is cpp file of the class of "ModelParametersClass" that is structure of the model parameters such as road length.
*/

#include "ModelParametersClass.h"

/*
	Initialize parameters reading ".ini" file.
*/
ModelParametersClass::ModelParametersClass(const std::string& iniFilePath) {
	InitializeProperties(this);
	ReadIniFilePackage ReadIniFile = ReadIniFilePackage(iniFilePath);
	ReadIniFile.ReadIni("Model Parameters", "NMax", _NMax);
	ReadIniFile.ReadIni("Model Parameters", "deltaT", _deltaT);
	ReadIniFile.ReadIni("Model Parameters", "L", _L);
	ReadIniFile.ReadIni("Model Parameters", "Run-Up Time", _RunUpTime);
	std::string sMode;
	ReadIniFile.ReadIni("Model Parameters", "InitialPositionMode", sMode, ReadIniFilePackage::TransformModeType::Lower);
	if (sMode == "random") {
		_InitialPositionMode = InitialPositionModeType::Random;
	}
	else {
		_InitialPositionMode = InitialPositionModeType::Equal;
	}	
}

void ModelParametersClass::InitializeProperties(ModelParametersClass* const thisPtr) {
	NMax(std::bind(&ModelParametersClass::Get_NMax, thisPtr));
	L(std::bind(&ModelParametersClass::Get_L, thisPtr));
	deltaT(std::bind(&ModelParametersClass::Get_deltaT, thisPtr));
	RunUpTime(std::bind(&ModelParametersClass::Get_RunUpTime, thisPtr));
	InitialPositionMode(std::bind(&ModelParametersClass::Get_InitialPositionMode, thisPtr));
}

const int& ModelParametersClass::Get_NMax() const {
	return _NMax;
}

const double& ModelParametersClass::Get_L() const {
	return _L;
}

const double& ModelParametersClass::Get_deltaT() const {
	return _deltaT;
}

const double& ModelParametersClass::Get_RunUpTime() const {
	return _RunUpTime;
}

const InitialPositionModeType& ModelParametersClass::Get_InitialPositionMode() const {
	return _InitialPositionMode;
}
