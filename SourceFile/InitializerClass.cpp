/*
	This is header file of the class of "InitializerClass" that reads car and driver-specific information from the ".ini" file and determines the initial position of the car.
	This inherits from "ModelBaseClass".
*/

#include "InitializerClass.h"

//constructor
InitializerClass::InitializerClass(const std::string& IniFileFolderPath, const int& IniFileNumber, const ModelBaseClass* myBase) : ModelBaseClass(myBase) {
	InitializeProperties(this);
	ReadIniFile = new ReadIniFilePackage(IniFileFolderPath + R"(/Ini)" + std::to_string(IniFileNumber) + ".ini");
}

//destructor
InitializerClass::~InitializerClass() {
	SafeDelete(ReadIniFile);	//delete ReadIniFilePackage
}

/*
	Initialize all parameters of car and driver, in addition, initializes the set positions of all cars.
*/
bool InitializerClass::Initialize() {
	InitializeCarsAndDrivers();
	bool success = InitializePosition();
	for (std::size_t i = 0; i < cars->size(); i++) {
		(*cars)[i]->Moment->UpdateReferences();
	}

	return success;
}

/*
	Initialize all parameters of car and driver reading ".ini" file.
*/
void InitializerClass::InitializeCarsAndDrivers() {
	double pVal;
	double mVal;
	std::string sModeType;
	allDclosest = 0;
	allCarLength = 0;
	for (std::size_t i = 0; i < std::size_t(N); i++) {
		(*cars)[i] = new CarStruct();
		//Car
		CarStruct* const car = (*cars)[i];
		car->ID = i;
		CarElements::EigenValues* const carEigen = car->Eigen;
		CarElements::MomentValues* const carMoment = car->Moment;

		carEigen->Vmax = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Car Informations", "Vmax"));
		carEigen->Amax->Plus = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Car Informations", "A^+_max_V")) / ReadIniFile->ReadIni("Car Informations", "A^+_max_s");
		carEigen->Amax->Minus = std::pow(Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Car Informations", "A^-_max_V")), 2) / 2 / ReadIniFile->ReadIni("Car Informations", "A^-_max_D");
		carEigen->AResistance = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Car Informations", "A^-_resistence"));
		carEigen->Length = ReadIniFile->ReadIni("Car Informations", "Length");
		ReadIniFile->ReadIni("Car Informations", "Driver", sModeType, ReadIniFilePackage::TransformModeType::Lower);
		if (sModeType == "auto") {
			carEigen->DriverMode = DriverModeType::Auto;
		}
		else {
			carEigen->DriverMode = DriverModeType::Human;
		}

		carMoment->a = 0;
		carMoment->v = 0;

		//Driver
		DriverStruct* const driver = car->Driver;
		DriverElements::EigenValues* const driverEigen = driver->Eigen;
		DriverElements::MomentValues* const driverMoment = driver->Moment;
		DriverElements::EigenValuesElements::AccelerationPackage* const driverEigenA = driverEigen->A;
		DriverElements::EigenValuesElements::AccelerationSeries* const Acceleration = driverEigenA->Acceleration;
		DriverElements::EigenValuesElements::AccelerationSeries* const Deceleration = driverEigenA->Deceleration;
		DriverElements::EigenValuesElements::AccelerationSeries* const FrontDeceleration = driverEigenA->FrontDeceleration;
		DriverElements::EigenValuesElements::VSerise* const V = driverEigen->V;
		DriverElements::EigenValuesElements::PedalChangingTimeInformations* const PedalChange = driverEigen->PedalChange;
		DriverElements::EigenValuesElements::TMargin* const TMargin = driverEigen->TMargin;
		Common::EigenValuesElements::GSerise* const G = driverEigen->G;

		Deceleration->Acceptable = ReadIniFile->ReadIni("Driver Informations::A", "A^-_acceptable");

		ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_mode", sModeType, ReadIniFilePackage::TransformModeType::Lower);
		if (sModeType == "equal") {
			Acceleration->Acceptable = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_V")) / ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_s");
		}
		else {
			pVal = ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_s^+");
			mVal = ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_s^-");
			Acceleration->Acceptable = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_V")) / (*random)(mVal, pVal);
		}

		ReadIniFile->ReadIni("Driver Informations::A", "A^-_strong_mode", sModeType, ReadIniFilePackage::TransformModeType::Lower);
		if (sModeType == "equal") {
			Deceleration->Strong = ReadIniFile->ReadIni("Driver Informations::A", "A^-_strong");
		}
		else {
			pVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_strong^+");
			mVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_strong^-");
			Deceleration->Strong = (*random)(mVal, pVal);
		}

		ReadIniFile->ReadIni("Driver Informations::A", "A^-_normal_mode", sModeType, ReadIniFilePackage::TransformModeType::Lower);
		if (sModeType == "equal") {
			Deceleration->Normal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_normal");
		}
		else {
			pVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_normal^+");
			mVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_normal^-");
			Deceleration->Normal = (*random)(mVal, pVal);
		}

		ReadIniFile->ReadIni("Driver Informations::A", "A^-_Fnormal_mode", sModeType, ReadIniFilePackage::TransformModeType::Lower);
		if (sModeType == "equal") {
			FrontDeceleration->Normal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_Fnormal");
		}
		else {
			pVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_Fnormal^+");
			mVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_Fnormal^-");
			FrontDeceleration->Normal = (*random)(mVal, pVal);
		}
		driverMoment->a = 0;
		driverMoment->R->velocity = 1 - (*random)(1.0);
		driverMoment->R->gap = 1 - (*random)(1.0);

		ReadIniFile->ReadIni("Driver Informations::Fg", "Fg_mode", sModeType, ReadIniFilePackage::TransformModeType::Lower);
		if (sModeType == "equal") {
			driverMoment->g->SetBaseNg(ReadIniFile->ReadIni("Driver Informations::Fg", "randomValue"));
		}
		else {
			pVal = ReadIniFile->ReadIni("Driver Informations::Fg", "randomValue^+");
			mVal = ReadIniFile->ReadIni("Driver Informations::Fg", "randomValue^-");
			driverMoment->g->SetBaseNg((*random)(mVal, pVal));
		}

		Common::EigenValuesElements::PlusMinus* const DeltaAtCruise = V->DeltaAtCruise;
		Common::EigenValuesElements::PlusMinus* const DeltaAt0 = V->DeltaAt0;
		V->Cruise = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::V", "V_cruise"));
		DeltaAtCruise->Plus = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::V", "deltaV^+_cruise"));
		DeltaAtCruise->Minus = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::V", "deltaV^-_cruise"));
		DeltaAt0->Plus = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::V", "deltaV^+_0"));
		DeltaAt0->Minus = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::V", "deltaV^-_0"));

		//Set Vcruise as the target velocity.
		DriverElements::MomentValuesElements::VSerise* const v = driverMoment->v;
		Common::MomentValuesElements::CurrentLast* const deltaV = v->deltaV;
		Common::MomentValuesElements::PlusMinus* const delta = v->delta;
		v->target = V->Cruise;
		deltaV->current = -V->Cruise;
		deltaV->last = -V->Cruise;
		delta->plus = DeltaAtCruise->Plus;
		delta->minus = DeltaAtCruise->Minus;

		DriverElements::EigenValuesElements::PedalChanging* const PedalChangeV = PedalChange->V;
		DriverElements::EigenValuesElements::PedalChanging* const PedalChangeT = PedalChange->T;
		Common::EigenValuesElements::UpperLower* const VAccelToBrake = PedalChangeV->AccelToBrake;
		Common::EigenValuesElements::UpperLower* const VBrakeToAccel = PedalChangeV->BrakeToAccel;
		Common::EigenValuesElements::UpperLower* const TAccelToBrake = PedalChangeT->AccelToBrake;
		Common::EigenValuesElements::UpperLower* const TBrakeToAccel = PedalChangeT->BrakeToAccel;
		VAccelToBrake->Upper = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Pedal Change", "V^+_ab"));
		VAccelToBrake->Lower = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Pedal Change", "V^-_ab"));
		TAccelToBrake->Upper = ReadIniFile->ReadIni("Driver Informations::Pedal Change", "T^+_ab");
		TAccelToBrake->Lower = ReadIniFile->ReadIni("Driver Informations::Pedal Change", "T^-_ab");
		VBrakeToAccel->Upper = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Pedal Change", "V^+_ba"));
		VBrakeToAccel->Lower = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Pedal Change", "V^-_ba"));
		TBrakeToAccel->Upper = ReadIniFile->ReadIni("Driver Informations::Pedal Change", "T^+_ba");
		TBrakeToAccel->Lower = ReadIniFile->ReadIni("Driver Informations::Pedal Change", "T^-_ba");

		Common::EigenValuesElements::UpperLower* const TMarginV = TMargin->V;
		Common::EigenValuesElements::UpperLower* const TMarginT = TMargin->T;
		TMarginV->Upper = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Margin", "V^+_margin"));
		TMarginV->Lower = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Margin", "V^-_margin"));
		TMarginT->Upper = ReadIniFile->ReadIni("Driver Informations::Margin", "T^+_margin");
		TMarginT->Lower = ReadIniFile->ReadIni("Driver Informations::Margin", "T^-_margin");

		G->Closest = ReadIniFile->ReadIni("Driver Informations::G", "G_closest");
		G->Cruise = ReadIniFile->ReadIni("Driver Informations::G", "G_cruise");
		G->Influenced = ReadIniFile->ReadIni("Driver Informations::G", "G_influenced");
		
		allDclosest += driverEigen->G->Closest;
		allCarLength += carEigen->Length;
	}
}

/*
	Initializes the set positions of all cars.
*/
bool InitializerClass::InitializePosition() const {
	//First, arrange them evenly.
	const bool&& success = EqualizeAllGap();
	switch (ModelParameters.InitialPositionMode) {
	case InitialPositionModeType::Equal:
		break;
	case InitialPositionModeType::Random:
		//Next, select any vehicle in a random order and move the vehicle to a random position between the previous and following vehicles.
		ChangePositionFromUniformToRandom();
		break;
	default:
		break;
	}

	//Update all car's gap.
	double frontX;
	for (std::size_t i = 0; i < cars->size(); i++) {
		const CarStruct* const car = (*cars)[i];
		CarElements::MomentValues* const carMoment = car->Moment;
		const CarElements::MomentValuesElements::ArroundCarInformations* const front = carMoment->arround->front;
		const double& x = carMoment->x;
		frontX = front->x;
		if (frontX <= x) {
			frontX += ModelParameters.L;
		}
		double&& gap = frontX - front->Length - x;

		const Common::EigenValuesElements::GSerise* const G = car->Driver->Eigen->G;
		CarElements::MomentValuesElements::GapSerise* const g = carMoment->g;
		g->closest = G->Closest;
		g->cruise = g->closest + G->Cruise;
		g->influenced = g->cruise + G->Influenced;
		g->gap = std::move(gap);
		g->deltaGap->current = g->gap - g->cruise;
	}

	return success;
}

/*
	Set up all cars with an equal distance between them.
*/
bool InitializerClass::EqualizeAllGap() const {
	const double remArea = ModelParameters.L - allCarLength;
	if (remArea <= allDclosest) {
		return false;
	}
	else {
		//Assign vehicle ID randomly.
		//First, set random numbers between 0 and 1 in an array of size N.
		std::vector<double> randomID(N);
		for (std::size_t i = 0; i < randomID.size(); i++) {
			randomID[i] = (*random)(1.0);
		}
		//Next, sort this array in ascending order of random numbers.
		VectorSort IDSort = VectorSort();
		IDSort.AscendingSort(randomID);	//Use the subscript of the original array that corresponds to the rearranged random number array as the ID.

		std::size_t front;
		std::size_t rear;
		double x = ModelParameters.L / 2;
		const double parGap = remArea / N;
		for (std::size_t i = 0; i < cars->size(); i++) {
			const CarStruct* const car = (*cars)[IDSort[i]];
			if (i == 0) {
				front = IDSort[N - 1];
			}
			else {
				front = IDSort[i - 1];
			}
			if (i == std::size_t(N) - 1) {
				rear = IDSort[0];
			}
			else {
				rear = IDSort[i + 1];
			}
			CarElements::MomentValues* const carMoment = car->Moment;
			carMoment->x = x;
			carMoment->UpdateReferences();
			x -= parGap + car->Eigen->Length;
			if (x < 0) {
				x += ModelParameters.L;
			}
			carMoment->arround = new CarElements::MomentValuesElements::Arround((*cars)[rear], (*cars)[front]);	//Set pointers for the front and rear vehicles.
		}
		return true;
	}
}

/*
	Change the position from uniform to random.
*/
void InitializerClass::ChangePositionFromUniformToRandom() const {
	std::vector<double> randomID(N);
	for (std::size_t i = 0; i < randomID.size(); i++) {
		randomID[i] = (*random)(1.0);
	}
	VectorSort IDSort = VectorSort();
	IDSort.AscendingSort(randomID);
	for (std::size_t i = 0; i < IDSort.size(); i++) {
		MoveBetweenFrontAndRearCars(IDSort[i]);
	}
}

/*
	Move the vehicle to a random position between the previous and following vehicles.
*/
void InitializerClass::MoveBetweenFrontAndRearCars(const std::size_t& ID) const {
	//Find the range of movement forward and backward.
	const CarStruct* car = (*cars)[ID];
	CarElements::MomentValues* const carMoment = car->Moment;
	const CarElements::MomentValuesElements::Arround* const arround = carMoment->arround;
	const CarElements::MomentValuesElements::ArroundCarInformations* front = arround->front;
	const CarElements::MomentValuesElements::ArroundCarInformations* rear = arround->rear;
	const double& x = carMoment->x;
	double frontX = front->x;
	double rearX = rear->x;
	if (front->x <= x) {
		frontX += ModelParameters.L;
	}
	frontX -= x;
	if (rearX >= x) {
		rearX -= ModelParameters.L;
	}
	rearX -= x;

	const double&& xMax = frontX - front->Length - car->Driver->Eigen->G->Closest;
	const double&& xMin = rearX + (*cars)[rear->ID]->Driver->Eigen->G->Closest + car->Eigen->Length;
	double&& nextX = (xMax - xMin) * (*random)(1.0) + xMin + x;
	if (nextX >= ModelParameters.L) {
		nextX -= ModelParameters.L;
	}
	else if (nextX < 0) {
		nextX += ModelParameters.L;
	}
	carMoment->x = std::move(nextX);
	carMoment->UpdateReferences();
}

void InitializerClass::InitializeProperties(InitializerClass* const thisPtr) {
	GlobalK(std::bind(&InitializerClass::Get_GlobalK, thisPtr));
}

double InitializerClass::Get_GlobalK() const {
	return allCarLength / ModelParameters.L;
}
