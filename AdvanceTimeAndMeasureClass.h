/*
	This is header file of the class of "AdvanceTimeAndMeasureClass" that actually performs calculations, advances time, and measures the results.
	This inherits from "ModelBaseClass".
*/

#ifndef ADVANCETIMEANDMEASURECLASS_H
#define ADVANCETIMEANDMEASURECLASS_H
#include "ReadOnlyPropertyClass.h"
#include "ModelBaseClass.h"
#include "DecideDriverTargetAccelerationClass.h"
#include "UpdatePositionClass.h"
#include "InitializerClass.h"

class AdvanceTimeAndMeasureClass : public ModelBaseClass {
public:
	AdvanceTimeAndMeasureClass(const int& N, const ModelParametersClass& ModelParameters, const StatisticsParametersClass& statisticsParameters);	//constructor
	~AdvanceTimeAndMeasureClass();	//destructor

	bool Initialize(const std::string& IniFileFolderPath, const int& IniFileNumber);
	void AdvanceTimeAndMeasure();
	StatisticsClass* const Statistics() const;
private:
	bool _succedMeasure;
	DecideDriverTargetAccelerationClass* DecideDriverTargetAcceleration;
	UpdatePositionClass* UpdatePosition;
	StatisticsClass* statistics;
	bool isFirstInitalize;

	void RunUp();
	void Measure();
	void AdvaceTime();

	void InitializeProperties(AdvanceTimeAndMeasureClass* const thisPtr);

	bool Get_SuccedMeasure() const;
public:
	ReadOnlyPropertyClass<bool> SuccedMeasure;
};

#endif // !ADVANCETIMEANDMEASURECLASS_H