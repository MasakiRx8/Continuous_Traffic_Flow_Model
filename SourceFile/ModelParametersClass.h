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
	InitialPositionModeType _InitialPositionMode;
	void InitializeProperties(ModelParametersClass* const thisPtr);

	const int& Get_NMax() const;
	const double& Get_L() const;
	const double& Get_deltaT() const;
	const double& Get_RunUpTime() const;
	const InitialPositionModeType& Get_InitialPositionMode() const;
public:
	ReadOnlyPropertyClass<const int&> NMax;
	ReadOnlyPropertyClass<const double&> L;
	ReadOnlyPropertyClass<const double&> deltaT;
	ReadOnlyPropertyClass<const double&> RunUpTime;
	ReadOnlyPropertyClass<const InitialPositionModeType&> InitialPositionMode;
};

#endif // !MODELPARAMETERSCLASS_H
