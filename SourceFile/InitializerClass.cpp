/*
	This is header file of the class of "InitializerClass" that reads car and driver-specific information from the ".ini" file and determines the initial position of the car.
	This inherits from "ModelBaseClass".
*/

#include "InitializerClass.h"

//constructor
InitializerClass::InitializerClass(const ModelBaseClass* myBase, const std::string& IniFileFolderPath, const int& IniFileNumber) : ModelBaseClass(myBase) {
	InitializeProperties(this);
	ReadIniFile = new ReadIniFilePackage(IniFileFolderPath + R"(\Ini)" + std::to_string(IniFileNumber) + ".ini");
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
		CarElements::EigenValues* const carE = car->Eigen;

		carE->Vmax = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Car Informations", "Vmax"));
		carE->Amax->Plus = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Car Informations", "A^+_max_V")) / ReadIniFile->ReadIni("Car Informations", "A^+_max_s");
		carE->Amax->Minus = std::pow(Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Car Informations", "A^-_max_V")), 2) / 2 / ReadIniFile->ReadIni("Car Informations", "A^-_max_D");
		carE->AResistance = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Car Informations", "A^-_resistence"));
		carE->Length = ReadIniFile->ReadIni("Car Informations", "Length");
		ReadIniFile->ReadIni("Car Informations", "Driver", sModeType, ReadIniFilePackage::TransformMode::Lower);
		if (sModeType == "auto") {
			carE->DriverMode = DriverMode::Auto;
		}
		else {
			carE->DriverMode = DriverMode::Human;
		}

		CarElements::MomentValues* const carM = car->Moment;
		carM->a = 0;
		carM->v = 0;

		//Driver
		DriverStruct* const driver = car->Driver;
		DriverElements::EigenValues* const driverE = driver->Eigen;
		DriverElements::MomentValues* const driverM = driver->Moment;
		DriverElements::EigenValuesElements::AccelerationSeries* const Acceleration = driver->Eigen->A->Acceleration;
		DriverElements::EigenValuesElements::AccelerationSeries* const Deceleration = driver->Eigen->A->Deceleration;
		DriverElements::EigenValuesElements::AccelerationSeries* const FrontDeceleration = driver->Eigen->A->FrontDeceleration;
		DriverElements::EigenValuesElements::VSerise* const V = driverE->V;
		DriverElements::EigenValuesElements::PedalChangingTimeInformations* const PedalChange = driverE->PedalChange;
		DriverElements::EigenValuesElements::TMargin* const TMargin = driverE->TMargin;
		Common::EigenValuesElements::GSerise* const G = driverE->G;

		Deceleration->Acceptable = ReadIniFile->ReadIni("Driver Informations::A", "A^-_acceptable");

		ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_mode", sModeType, ReadIniFilePackage::TransformMode::Lower);
		if (sModeType == "equal") {
			Acceleration->Acceptable = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_V")) / ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_s");
		}
		else {
			pVal = ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_s^+");
			mVal = ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_s^-");
			Acceleration->Acceptable = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::A", "A^+_acceptable_V")) / (*random)(mVal, pVal);
		}

		ReadIniFile->ReadIni("Driver Informations::A", "A^-_strong_mode", sModeType, ReadIniFilePackage::TransformMode::Lower);
		if (sModeType == "equal") {
			Deceleration->Strong = ReadIniFile->ReadIni("Driver Informations::A", "A^-_strong");
		}
		else {
			pVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_strong^+");
			mVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_strong^-");
			Deceleration->Strong = (*random)(mVal, pVal);
		}

		ReadIniFile->ReadIni("Driver Informations::A", "A^-_normal_mode", sModeType, ReadIniFilePackage::TransformMode::Lower);
		if (sModeType == "equal") {
			Deceleration->Normal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_normal");
		}
		else {
			pVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_normal^+");
			mVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_normal^-");
			Deceleration->Normal = (*random)(mVal, pVal);
		}

		ReadIniFile->ReadIni("Driver Informations::A", "A^-_Fnormal_mode", sModeType, ReadIniFilePackage::TransformMode::Lower);
		if (sModeType == "equal") {
			FrontDeceleration->Normal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_Fnormal");
		}
		else {
			pVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_Fnormal^+");
			mVal = ReadIniFile->ReadIni("Driver Informations::A", "A^-_Fnormal^-");
			FrontDeceleration->Normal = (*random)(mVal, pVal);
		}
		driverM->a = 0;
		driverM->R->velocity = 1 - (*random)(1.0);
		driverM->R->gap = 1 - (*random)(1.0);

		ReadIniFile->ReadIni("Driver Informations::Fg", "Fg_mode", sModeType, ReadIniFilePackage::TransformMode::Lower);
		if (sModeType == "equal") {
			driverM->g->SetBaseNg(ReadIniFile->ReadIni("Driver Informations::Fg", "randomValue"));
		}
		else {
			pVal = ReadIniFile->ReadIni("Driver Informations::Fg", "randomValue^+");
			mVal = ReadIniFile->ReadIni("Driver Informations::Fg", "randomValue^-");
			driverM->g->SetBaseNg((*random)(mVal, pVal));
		}

		V->Cruise = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::V", "V_cruise"));
		V->DeltaAtCruise->Plus = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::V", "deltaV^+_cruise"));
		V->DeltaAtCruise->Minus = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::V", "deltaV^-_cruise"));
		V->DeltaAt0->Plus = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::V", "deltaV^+_0"));
		V->DeltaAt0->Minus = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::V", "deltaV^-_0"));

		PedalChange->V->AccelToBrake->Upper = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Pedal Change", "V^+_ab"));
		PedalChange->V->AccelToBrake->Lower = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Pedal Change", "V^-_ab"));
		PedalChange->T->AccelToBrake->Upper = ReadIniFile->ReadIni("Driver Informations::Pedal Change", "T^+_ab");
		PedalChange->T->AccelToBrake->Lower = ReadIniFile->ReadIni("Driver Informations::Pedal Change", "T^-_ab");
		PedalChange->V->BrakeToAccel->Upper = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Pedal Change", "V^+_ba"));
		PedalChange->V->BrakeToAccel->Lower = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Pedal Change", "V^-_ba"));
		PedalChange->T->BrakeToAccel->Upper = ReadIniFile->ReadIni("Driver Informations::Pedal Change", "T^+_ba");
		PedalChange->T->BrakeToAccel->Lower = ReadIniFile->ReadIni("Driver Informations::Pedal Change", "T^-_ba");

		TMargin->V->Upper = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Margin", "V^+_margin"));
		TMargin->V->Lower = Calculate_Km_h_To_m_s(ReadIniFile->ReadIni("Driver Informations::Margin", "V^-_margin"));
		TMargin->T->Upper = ReadIniFile->ReadIni("Driver Informations::Margin", "T^+_margin");
		TMargin->T->Lower = ReadIniFile->ReadIni("Driver Informations::Margin", "T^-_margin");

		G->Closest = ReadIniFile->ReadIni("Driver Informations::G", "G_closest");
		G->Cruise = ReadIniFile->ReadIni("Driver Informations::G", "G_cruise");
		G->Influenced = ReadIniFile->ReadIni("Driver Informations::G", "G_influenced");
		
		allDclosest += driverE->G->Closest;
		allCarLength += carE->Length;
	}
}

/*
	Initializes the set positions of all cars.
*/
bool InitializerClass::InitializePosition() const {
	//First, arrange them evenly.
	bool success = EqualizeAllGap();
	switch (ModelParameters.InitialPosition) {
	case InitialPositionMode::Equal:
		break;
	case InitialPositionMode::Random:
		//Next, select any vehicle in a random order and move the vehicle to a random position between the previous and following vehicles.
		ChangePositionFromUniformToRandom();
		break;
	default:
		break;
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
			car->Moment->x = x;
			car->Moment->UpdateReferences();
			x -= parGap + car->Eigen->Length;
			if (x < 0) {
				x += ModelParameters.L;
			}
			car->Moment->arround = new CarElements::MomentValuesElements::Arround((*cars)[rear], (*cars)[front]);	//Set pointers for the front and rear vehicles.
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

	double frontX;
	double gap;
	for (std::size_t i = 0; i < cars->size(); i++) {
		const CarStruct* const car = (*cars)[i];
		const CarElements::MomentValuesElements::ArroundCarInformations* const front = car->Moment->arround->front;
		frontX = front->x;
		if (frontX <= car->Moment->x) {
			frontX += ModelParameters.L;
		}
		gap = frontX - front->Length - car->Moment->x;

		const Common::EigenValuesElements::GSerise* const G = car->Driver->Eigen->G;
		CarElements::MomentValuesElements::GapSerise* const g = car->Moment->g;
		g->closest = G->Closest;
		g->cruise = g->closest + G->Cruise;
		g->influenced = g->cruise + G->Influenced;
		g->gap = gap;
		g->deltaGap->current = gap - g->cruise;

		const DriverElements::EigenValuesElements::VSerise* const eigenV = car->Driver->Eigen->V;
		DriverElements::MomentValuesElements::VSerise* const v = car->Driver->Moment->v;
		v->deltaV->current = -eigenV->Cruise;
		v->deltaV->last = -eigenV->Cruise;
		v->delta->plus = eigenV->DeltaAtCruise->Plus;
		v->delta->minus = eigenV->DeltaAtCruise->Minus;
		v->target = eigenV->Cruise;
	}
}

/*
	Move the vehicle to a random position between the previous and following vehicles.
*/
void InitializerClass::MoveBetweenFrontAndRearCars(const std::size_t& ID) const {
	//Find the range of movement forward and backward.
	const CarStruct* car = (*cars)[ID];
	const CarElements::MomentValuesElements::ArroundCarInformations* front = car->Moment->arround->front;
	const CarElements::MomentValuesElements::ArroundCarInformations* rear = car->Moment->arround->rear;
	const double x = car->Moment->x;
	double frontX = front->x;
	double rearX = rear->x;
	if (frontX <= x) {
		frontX += ModelParameters.L;
	}
	frontX -= x;
	if (rearX >= x) {
		rearX -= ModelParameters.L;
	}
	rearX -= x;

	const double xMax = frontX - front->Length - car->Driver->Eigen->G->Closest;
	const double xMin = rearX + (*cars)[rear->ID]->Driver->Eigen->G->Closest + car->Eigen->Length;
	double nextX = (xMax - xMin) * (*random)(1.0) + xMin + x;	//>=0
	if (nextX >= ModelParameters.L) {
		nextX -= ModelParameters.L;
	}
	car->Moment->x = nextX;
	car->Moment->UpdateReferences();
}

void InitializerClass::InitializeProperties(InitializerClass* const thisPtr) {
	GlobalK(std::bind(&InitializerClass::Get_GlobalK, thisPtr));
}

double InitializerClass::Get_GlobalK() const {
	return allCarLength / ModelParameters.L;
}
