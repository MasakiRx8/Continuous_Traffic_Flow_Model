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
			statistics->AddGlobal_dX(global_dX);
		}
		statistics->CalculateAndAddLocalStatistics();
	}
	statistics->CalculateAndSetGlobalStatistics();
}

/*
	Advance the model one time step.
*/
void AdvanceTimeAndMeasureClass::AdvaceTime() {
	global_dX = 0;
	int countMinusGap = 0;
	std::size_t frontID;
	std::size_t rearID;
	double rearX;
	int checked = 0;
	for (std::size_t i = 0; i < std::size_t(N); i++) {
		const CarStruct* const car = (*cars)[i];
		DecideDriverTargetAcceleration->DecideDriverTargetAcceleration(car);	//calculate by Eq.(4-12)
		UpdatePosition->UpdateCarPosition(car);
		global_dX += UpdatePosition->dX;

		//Check Collision
		frontID = car->Moment->arround->front->ID;
		rearID = car->Moment->arround->rear->ID;
		if (frontID <= i) {
			const CarStruct* const front = (*cars)[frontID];
			rearX = front->Moment->x - front->Eigen->Length;
			if (rearX < 0) {
				rearX += ModelParameters.L;
			}
			if (rearX < car->Moment->x) {
				countMinusGap++;
			}
			checked++;
		}
		if (rearID <= i && frontID != i) {
			const CarStruct* const rear = (*cars)[rearID];
			rearX = car->Moment->x - car->Eigen->Length;
			if (rearX < 0) {
				rearX += ModelParameters.L;
			}
			if (rearX < rear->Moment->x) {
				countMinusGap++;
			}
			checked++;
		}
	}
	//All cars update reference informations.
	for (std::size_t i = 0; i < std::size_t(N); i++) {
		(*cars)[i]->Moment->UpdateReferences();
	}
	if (checked != N) {
		_succedMeasure = false;
	}
	else {
		if (countMinusGap < 2) {
			_succedMeasure = true;
		}
		else {
			_succedMeasure = false;
		}
	}
}

void AdvanceTimeAndMeasureClass::InitializeProperties(AdvanceTimeAndMeasureClass* const thisPtr) {
	SuccedMeasure(std::bind(&AdvanceTimeAndMeasureClass::Get_SuccedMeasure, thisPtr));
}

bool AdvanceTimeAndMeasureClass::Get_SuccedMeasure() const {
	return _succedMeasure;
}
