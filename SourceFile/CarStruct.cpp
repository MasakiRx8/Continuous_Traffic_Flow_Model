#include "CarStruct.h"

CarElements::MomentValuesElements::GapSerise::GapSerise()
	: deltaGap(new Common::MomentValuesElements::CurrentLast()) {
	gap = 0;
	closest = 0;
	cruise = 0;
	influenced = 0;
	deletedDeltaGap = false;
}

CarElements::MomentValuesElements::GapSerise::~GapSerise() {
	if (deltaGap != nullptr && !deletedDeltaGap) {
		delete deltaGap;
		deletedDeltaGap = true;
	}
}

CarElements::MomentValuesElements::Measurement::Measurement() {
	Reset();
}

void CarElements::MomentValuesElements::Measurement::Reset() {
	passed = false;
	elapsedTime = 0;
}

CarElements::MomentValuesElements::ArroundCarInformations::ArroundCarInformations(const std::size_t& ID, const double& a, const double& v, const double& x, const double& Length)
	: ID(ID), a(a), v(v), x(x), Length(Length) { }

CarElements::MomentValuesElements::Arround::Arround(const CarStruct* const rear, const CarStruct* const front)
	: rear(new ArroundCarInformations(rear->ID, rear->Moment->referenceA, rear->Moment->referenceV, rear->Moment->referenceX, rear->Eigen->Length))
	, front(new ArroundCarInformations(front->ID, front->Moment->referenceA, front->Moment->referenceV, front->Moment->referenceX, front->Eigen->Length)) {
	deletedRear = false;
	deletedFront = false;
}

CarElements::MomentValuesElements::Arround::~Arround() {
	if (rear != nullptr && !deletedRear) {
		delete rear;
		deletedRear = true;
	}
	if (front != nullptr && !deletedFront) {
		delete front;
		deletedFront = true;
	}
}

CarElements::EigenValues::EigenValues()
	: Amax(new Common::EigenValuesElements::PlusMinus()) {
	Vmax = 0;
	AResistance = 0;
	Length = 0;
	deletedAmax = false;
	DriverMode = DriverModeType::Human;
}

CarElements::EigenValues::~EigenValues() {
	if (Amax != nullptr && !deletedAmax) {
		delete Amax;
		deletedAmax = true;
	}
}

CarElements::MomentValues::MomentValues()
	: g(new CarElements::MomentValuesElements::GapSerise())
	, measurement(new CarElements::MomentValuesElements::Measurement()) {
	a = 0;
	x = 0;
	v = 0;
	UpdateReferences();
	deletedG = false;
	deletedMeasurement = false;
}

CarElements::MomentValues::~MomentValues() {
	if (g != nullptr && !deletedG) {
		delete g;
		deletedG = true;
	}
	if (measurement != nullptr && !deletedMeasurement) {
		delete measurement;
		deletedMeasurement = true;
	}
	SafeDelete(arround);
}

void CarElements::MomentValues::UpdateReferences() {
	referenceA = a;
	referenceV = v;
	referenceX = x;
}

CarStruct::CarStruct()
	: Eigen(new CarElements::EigenValues())
	, Moment(new CarElements::MomentValues())
	, Driver(new DriverStruct()) { 
	ID = -1;
	deletedEigen = false;
	deletedMoment = false;
	deletedDriver = false;
}

CarStruct::~CarStruct() {
	if (Eigen != nullptr && !deletedEigen) {
		delete Eigen;
		deletedEigen = true;
	}
	if (Moment != nullptr && !deletedMoment) {
		delete Moment;
		deletedMoment = true;
	}
	if (Driver != nullptr && !deletedDriver) {
		delete Driver;
		deletedDriver = true;
	}
}
