/*
	This is header file of the class of "AdvanceTimeAndMeasureClass" that actually performs calculations, advances time, and measures the results.
	This inherits from "ModelBaseClass".
*/

#ifndef ADVANCETIMEANDMEASURECLASS_H
#define ADVANCETIMEANDMEASURECLASS_H
#include <fstream>
#include <sstream>
#include "ReadOnlyPropertyClass.h"
#include "ModelBaseClass.h"
#include "InitializerClass.h"
#include "DecideDriverTargetAccelerationClass.h"
#include "UpdatePositionClass.h"

class AdvanceTimeAndMeasureClass : public ModelBaseClass {
public:
	AdvanceTimeAndMeasureClass(const std::string& IniFileFolderPath, const int& IniFileNumber, const int& N, const ModelParametersClass& ModelParameters, const StatisticsParametersClass& StatisticsParameters, const bool& CreateSnapShot, const int& RunNumber, const std::string& SnapShotFolderPath);	//constructor
	~AdvanceTimeAndMeasureClass();	//destructor

	void AdvanceTimeAndMeasure();
	const StatisticsClass* const Statistics() const;
private:
	const bool CreateSnapShot;
	std::string SnapShotFileNameBase;

	bool _initializeSuccess;
	bool _succedMeasure;

	const PedalChangePackage* const PedalChnage;
	DecideDriverTargetAccelerationClass* DecideDriverTargetAcceleration;
	UpdatePositionClass* UpdatePosition;
	StatisticsClass* statistics;
	double global_dX;
	bool deletedPedalChnage;

	void Initialize(const std::string& IniFileFolderPath, const int& IniFileNumber);
	void RunUp();
	void Measure();
	std::string AdvaceTime();
	std::string GetSnapShotCSVName(const int& MeasureNumber);

	void InitializeProperties(AdvanceTimeAndMeasureClass* const thisPtr);

	const bool& Get_InitializeSuccess() const;
	const bool& Get_SuccedMeasure() const;
public:
	ReadOnlyPropertyClass<const bool&> InitializeSuccess;
	ReadOnlyPropertyClass<const bool&> SuccedMeasure;
};

#endif // !ADVANCETIMEANDMEASURECLASS_H