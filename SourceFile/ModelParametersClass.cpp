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
}

void ModelParametersClass::InitializeProperties(ModelParametersClass* const thisPtr) {
	NMax(std::bind(&ModelParametersClass::Get_NMax, thisPtr));
	L(std::bind(&ModelParametersClass::Get_L, thisPtr));
	deltaT(std::bind(&ModelParametersClass::Get_deltaT, thisPtr));
	RunUpTime(std::bind(&ModelParametersClass::Get_RunUpTime, thisPtr));
}

int ModelParametersClass::Get_NMax() const {
	return _NMax;
}

double ModelParametersClass::Get_L() const {
	return _L;
}

double ModelParametersClass::Get_deltaT() const {
	return _deltaT;
}

double ModelParametersClass::Get_RunUpTime() const {
	return _RunUpTime;
}
