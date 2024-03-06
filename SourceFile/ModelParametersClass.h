/*
	This is header file of the class of "ModelParametersClass" that is structure of the model parameters such as road length.
*/

#ifndef MODELPARAMETERSCLASS_H
#define MODELPARAMETERSCLASS_H
#include "ReadIniFilePackage.h"
#include "ReadOnlyPropertyClass.h"
#include "Common.h"

class ModelParametersClass {
public:
	ModelParametersClass(const std::string& iniFilePath);	//Initialize parameters reading ".ini" file.
private:
	int _NMax;
	double _L;
	double _deltaT;
	double _RunUpTime;
	InitialPositionMode _InitialPositionMode;
	void InitializeProperties(ModelParametersClass* const thisPtr);

	int Get_NMax() const;
	double Get_L() const;
	double Get_deltaT() const;
	double Get_RunUpTime() const;
	InitialPositionMode Get_InitialPositionMode() const;
public:
	ReadOnlyPropertyClass<int> NMax;
	ReadOnlyPropertyClass<double> L;
	ReadOnlyPropertyClass<double> deltaT;
	ReadOnlyPropertyClass<double> RunUpTime;
	ReadOnlyPropertyClass<InitialPositionMode> InitialPositionMode;
};

#endif // !MODELPARAMETERSCLASS_H
