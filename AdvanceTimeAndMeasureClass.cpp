/*
	This is cpp file of the class of "AdvanceTimeAndMeasureClass" that actually performs calculations, advances time, and measures the results.
	This inherits from "ModelBaseClass".
*/

#include "AdvanceTimeAndMeasureClass.h"

//constructor
AdvanceTimeAndMeasureClass::AdvanceTimeAndMeasureClass(const int& N, const ModelParametersClass& ModelParameters, const StatisticsParametersClass& statisticsParameters)
	: ModelBaseClass(N, ModelParameters, statisticsParameters) {
	InitializeProperties(this);
	isFirstInitalize = true;
}

//destructor
AdvanceTimeAndMeasureClass::~AdvanceTimeAndMeasureClass() {
	SafeDelete(DecideDriverTargetAcceleration);	//delete DecideDriverTargetAccelerationClass
	SafeDelete(UpdatePosition);	//delete UpdatePositionClass
	SafeDelete(statistics);		//delete StatisticsClass
}

bool AdvanceTimeAndMeasureClass::Initialize(const std::string& IniFileFolderPath, const int& IniFileNumber) {
	//Load the ini file and initialize the model calculation conditions and parameters for each vehicle.
	InitializerClass initializer(this, IniFileFolderPath, IniFileNumber);
	bool initializeSuccess = initializer.Initialize();

	if (!isFirstInitalize) {
		SafeDelete(DecideDriverTargetAcceleration);	//delete DecideDriverTargetAccelerationClass
		SafeDelete(UpdatePosition);	//delete UpdatePositionClass
		SafeDelete(statistics);		//delete StatisticsClass
	}
	statistics = new StatisticsClass(N, initializer.GlobalK, statisticsParameters);
	DecideDriverTargetAcceleration = new DecideDriverTargetAccelerationClass(this);
	UpdatePosition = new UpdatePositionClass(statistics, this);

	return initializeSuccess;
}

void AdvanceTimeAndMeasureClass::AdvanceTimeAndMeasure() {
	RunUp();
	Measure();
}

StatisticsClass* const AdvanceTimeAndMeasureClass::Statistics() const {
	return statistics;
}

void AdvanceTimeAndMeasureClass::RunUp() {
	double elapsed = 0;
	while (elapsed < ModelParameters.RunUpTime) {
		AdvaceTime();
		if (!_succedMeasure) {
			return;
		}
		elapsed += ModelParameters.deltaT;
	}
}

void AdvanceTimeAndMeasureClass::Measure() {
	double elapsed;
	for (int n = 0; n < statisticsParameters.NumberOfMeasurements; n++) {
		elapsed = 0;
		statistics->Reset();
		while (elapsed < statisticsParameters.UnitMeasurementTime) {
			AdvaceTime();
			if (!_succedMeasure) {
				return;
			}
			elapsed += ModelParameters.deltaT;
			statistics->AddGlobal_dX(UpdatePosition->Global_dX);
		}
		statistics->CalculateAndAddLocalStatistics();
	}
	statistics->CalculateAndSetGlobalStatistics();
}

/*
	Advance the model one time step.
*/
void AdvanceTimeAndMeasureClass::AdvaceTime() {
	DecideDriverTargetAcceleration->DecideDriverTargetAcceleration();	//calculate by Eq.(4-12)
	_succedMeasure = (!DecideDriverTargetAcceleration->Collision);
	if (_succedMeasure) {
		UpdatePosition->UpdateCarPosition();
	}
}

void AdvanceTimeAndMeasureClass::InitializeProperties(AdvanceTimeAndMeasureClass* const thisPtr) {
	SuccedMeasure(std::bind(&AdvanceTimeAndMeasureClass::Get_SuccedMeasure, thisPtr));
}

bool AdvanceTimeAndMeasureClass::Get_SuccedMeasure() const {
	return _succedMeasure;
}
