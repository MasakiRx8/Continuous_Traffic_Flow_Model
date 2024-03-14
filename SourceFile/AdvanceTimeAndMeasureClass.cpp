/*
	This is cpp file of the class of "AdvanceTimeAndMeasureClass" that actually performs calculations, advances time, and measures the results.
	This inherits from "ModelBaseClass".
*/

#include "AdvanceTimeAndMeasureClass.h"

//constructor
AdvanceTimeAndMeasureClass::AdvanceTimeAndMeasureClass(const std::string& IniFileFolderPath, const int& IniFileNumber, const int& N, const ModelParametersClass& ModelParameters, const StatisticsParametersClass& StatisticsParameters, const bool& CreateSnapShot, const int& RunNumber, const std::string& SnapShotFolderPath)
	: ModelBaseClass(RunNumber, N, ModelParameters, StatisticsParameters)
	, PedalChnage(new PedalChangePackage(ModelParameters.deltaT))
	, CreateSnapShot(CreateSnapShot) {
	deletedPedalChnage = false;
	InitializeProperties(this);
	_initializeSuccess = false;
	if (RunNumber == 0) {
		SnapShotFileNameBase = SnapShotFolderPath + R"(/SnapShot)" + "_N" + std::to_string(N);
	}
	else {
		SnapShotFileNameBase = SnapShotFolderPath + R"(/SnapShot)" + "_RunN" + std::to_string(RunNumber) + "_N" + std::to_string(N);
	}

	Initialize(IniFileFolderPath, IniFileNumber);
}

//destructor
AdvanceTimeAndMeasureClass::~AdvanceTimeAndMeasureClass() {
	SafeDelete(DecideDriverTargetAcceleration);	//delete DecideDriverTargetAccelerationClass
	SafeDelete(UpdatePosition);	//delete UpdatePositionClass
	SafeDelete(statistics);		//delete StatisticsClass
	if (!deletedPedalChnage) {
		delete PedalChnage;		//delete PedalChangePackage
		deletedPedalChnage = true;
	}
}

void AdvanceTimeAndMeasureClass::AdvanceTimeAndMeasure() {
	RunUp();
	Measure();
}

const StatisticsClass* const AdvanceTimeAndMeasureClass::Statistics() const {
	return statistics;
}

void AdvanceTimeAndMeasureClass::Initialize(const std::string& IniFileFolderPath, const int& IniFileNumber) {
	//Load the ini file and initialize the model calculation conditions and parameters for each vehicle.
	InitializerClass initializer(IniFileFolderPath, IniFileNumber, this);
	_initializeSuccess = initializer.Initialize();
	if (_initializeSuccess) {
		statistics = new StatisticsClass(N, initializer.GlobalK, StatisticsParameters);
		DecideDriverTargetAcceleration = new DecideDriverTargetAccelerationClass(PedalChnage, this);
		UpdatePosition = new UpdatePositionClass(statistics, PedalChnage, this);
	}
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
	if (CreateSnapShot) {
		for (int i = 0; i < StatisticsParameters.NumberOfMeasurements; i++) {
			elapsed = 0;
			statistics->Reset();

			std::ofstream ofs(GetSnapShotCSVName(i + 1), std::ios::app);
			ofs << "time";
			for (int j = 1; j <= N; j++) {
				ofs << ",N" << std::to_string(j);
			}
			ofs << std::endl;

			ofs << elapsed;
			for (std::size_t j = 0; j < cars->size(); j++) {
				ofs << "," << (*cars)[j]->Moment->x;
			}
			ofs << std::endl;
			while (elapsed < StatisticsParameters.UnitMeasurementTime) {
				const std::string&& snapShot = AdvaceTime();
				if (!_succedMeasure) {
					return;
				}
				elapsed += ModelParameters.deltaT;
				statistics->AddGlobal_dX(global_dX);
				ofs << elapsed << "," << snapShot << std::endl;
			}
			ofs.close();
			statistics->CalculateAndAddLocalStatistics();
		}
		statistics->CalculateAndSetGlobalStatistics();
	}
	else {
		for (int i = 0; i < StatisticsParameters.NumberOfMeasurements; i++) {
			elapsed = 0;
			statistics->Reset();
			while (elapsed < StatisticsParameters.UnitMeasurementTime) {
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
}

/*
	Advance the model one time step.
*/
std::string AdvanceTimeAndMeasureClass::AdvaceTime() {
	global_dX = 0;
	int countMinusGap = 0;
	double rearX;
	int checked = 0;
	int updated = 0;
	std::stringstream snapShot;
	for (std::size_t i = 0; i < std::size_t(N); i++) {
		const CarStruct* const car = (*cars)[i];
		DecideDriverTargetAcceleration->DecideDriverTargetAcceleration(car);	//calculate by Eq.(4-12)
		UpdatePosition->UpdateCarPosition(car);
		global_dX += UpdatePosition->dX;
		if (CreateSnapShot) {
			if (i > 0) {
				snapShot << ",";
			}
			snapShot << UpdatePosition->Position;
		}

		//Check Collision and Update reference informations
		CarElements::MomentValues* const carMoment = car->Moment;
		const CarElements::MomentValuesElements::Arround* const arround = carMoment->arround;
		const std::size_t& frontID = arround->front->ID;
		const std::size_t& rearID = arround->rear->ID;
		if (frontID <= i) {
			const CarStruct* const front = (*cars)[frontID];
			rearX = front->Moment->x - front->Eigen->Length;
			if (rearX < 0) {
				rearX += ModelParameters.L;
			}
			if (rearX < carMoment->x) {
				countMinusGap++;
			}
			checked++;

			//Front car updates reference informations.
			const std::size_t& frontFrontID = front->Moment->arround->front->ID;
			if (frontFrontID < i) {
				front->Moment->UpdateReferences();
				updated++;
			}
		}
		if (rearID <= i && frontID != i) {
			const CarStruct* const rear = (*cars)[rearID];
			rearX = carMoment->x - car->Eigen->Length;
			if (rearX < 0) {
				rearX += ModelParameters.L;
			}
			if (rearX < rear->Moment->x) {
				countMinusGap++;
			}
			checked++;

			//Rear car updates reference informations.
			const std::size_t& rearRearID = rear->Moment->arround->rear->ID;
			if (rearRearID <= i && N != 1) {
				rear->Moment->UpdateReferences();
				updated++;
			}
		}
		//Update reference informations
		if (frontID <= i && rearID <= i) {
			carMoment->UpdateReferences();
			updated++;
		}
	}
	if (checked != N || updated != N) {
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

	return snapShot.str();
}

std::string AdvanceTimeAndMeasureClass::GetSnapShotCSVName(const int& MeasureNumber) {
	return SnapShotFileNameBase + "_MeasureN" + std::to_string(MeasureNumber) + ".csv";
}

void AdvanceTimeAndMeasureClass::InitializeProperties(AdvanceTimeAndMeasureClass* const thisPtr) {
	InitializeSuccess(std::bind(&AdvanceTimeAndMeasureClass::Get_InitializeSuccess, thisPtr));
	SuccedMeasure(std::bind(&AdvanceTimeAndMeasureClass::Get_SuccedMeasure, thisPtr));
}

const bool& AdvanceTimeAndMeasureClass::Get_InitializeSuccess() const {
	return _initializeSuccess;
}

const bool& AdvanceTimeAndMeasureClass::Get_SuccedMeasure() const {
	return _succedMeasure;
}
