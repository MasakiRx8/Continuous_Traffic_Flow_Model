#include "DriverStruct.h"

DriverElements::EigenValuesElements::AccelerationSeries::AccelerationSeries() {
	Acceptable = 0;
	Strong = 0;
	Normal = 0;
}

DriverElements::EigenValuesElements::AccelerationPackage::AccelerationPackage()
	: Acceleration(new AccelerationSeries()) 
	, Deceleration(new AccelerationSeries())
	, FrontDeceleration(new AccelerationSeries()) { 
	deletedAcceleration = false;
	deletedDeceleration = false;
	deletedFrontDeceleration = false;
}

DriverElements::EigenValuesElements::AccelerationPackage::~AccelerationPackage() {
	if (Acceleration != nullptr && !deletedAcceleration) {
		delete Acceleration;
		deletedAcceleration = true;
	}
	if (Deceleration != nullptr && !deletedDeceleration) {
		delete Deceleration;
		deletedDeceleration = true;
	}
	if (FrontDeceleration != nullptr && !deletedFrontDeceleration) {
		delete FrontDeceleration;
		deletedFrontDeceleration = true;
	}
}

DriverElements::EigenValuesElements::PedalChanging::PedalChanging()
	: AccelToBrake(new Common::EigenValuesElements::UpperLower())
	, BrakeToAccel(new Common::EigenValuesElements::UpperLower()) { 
	deletedAccelToBrake = false;
	deletedBrakeToAccel = false;
}

DriverElements::EigenValuesElements::PedalChanging::~PedalChanging() {
	if (AccelToBrake != nullptr && !deletedAccelToBrake) {
		delete AccelToBrake;
		deletedAccelToBrake = true;
	}
	if (BrakeToAccel != nullptr && !deletedBrakeToAccel) {
		delete BrakeToAccel;
		deletedBrakeToAccel = true;
	}
}

DriverElements::EigenValuesElements::PedalChangingTimeInformations::PedalChangingTimeInformations()
	: T(new PedalChanging())
	, V(new PedalChanging()) { 
	deletedT = false;
	deletedV = false;
}

DriverElements::EigenValuesElements::PedalChangingTimeInformations::~PedalChangingTimeInformations() {
	if (T != nullptr && !deletedT) {
		delete T;
		deletedT = true;
	}
	if (V != nullptr && !deletedV) {
		delete V;
		deletedV = true;
	}
}

DriverElements::EigenValuesElements::VSerise::VSerise()
	: DeltaAtCruise(new Common::EigenValuesElements::PlusMinus())
	, DeltaAt0(new Common::EigenValuesElements::PlusMinus()) {
	Cruise = 0;
	deletedDeltaAtCruise = false;
	deletedDeltaAt0 = false;
}

DriverElements::EigenValuesElements::VSerise::~VSerise() {
	if (DeltaAtCruise != nullptr && !deletedDeltaAtCruise) {
		delete DeltaAtCruise;
		deletedDeltaAtCruise = true;
	}
	if (DeltaAt0 != nullptr && !deletedDeltaAt0) {
		delete DeltaAt0;
		deletedDeltaAt0 = true;
	}
}

DriverElements::EigenValuesElements::TMargin::TMargin()
	: V(new Common::EigenValuesElements::UpperLower())
	, T(new Common::EigenValuesElements::UpperLower()) {
	deletedV = false;
	deletedT = false;
}

DriverElements::EigenValuesElements::TMargin::~TMargin() {
	if (V != nullptr && !deletedV) {
		delete V;
		deletedV = true;
	}
	if (T != nullptr && !deletedT) {
		delete T;
		deletedT = true;
	}
}

DriverElements::MomentValuesElements::NeedChangingTime::NeedChangingTime() {
	accelToBrake = 0;
	brakeToAccel = 0;
}

DriverElements::MomentValuesElements::PedalInformations::PedalInformations()
	: t(new NeedChangingTime()) {
	needTime = timeElapsed = 0;
	changing = false;
	footPosition = FootPositionType::Brake;
	targetFootPosition = FootPositionType::Accel;
	deletedT = false;
}

DriverElements::MomentValuesElements::PedalInformations::~PedalInformations() {
	if (t != nullptr && !deletedT) {
		delete t;
		deletedT = true;
	}
}

DriverElements::MomentValuesElements::VSerise::VSerise()
	: delta(new  Common::MomentValuesElements::PlusMinus())
	, deltaV(new Common::MomentValuesElements::CurrentLast()) {
	target = 0;
	deletedDelta = false;
	deletedDeltaV = false;
}

DriverElements::MomentValuesElements::VSerise::~VSerise() {
	if (delta != nullptr && !deletedDelta) {
		delete delta;
		deletedDelta = true;
	}
	if (deltaV != nullptr && !deletedDeltaV) {
		delete deltaV;
		deletedDeltaV = true;
	}
}

DriverElements::MomentValuesElements::GSerise::GSerise() {
	SetBaseNg(0);
	emergency = false;
}

void DriverElements::MomentValuesElements::GSerise::SetBaseNg(const double& randomValue) {
	baseNg = 2 * randomValue - 1;
	baseFg = 1.0 / (1 + exp(-baseNg / kappa));
}

DriverElements::EigenValues::EigenValues()
	: A(new DriverElements::EigenValuesElements::AccelerationPackage())
	, PedalChange(new DriverElements::EigenValuesElements::PedalChangingTimeInformations())
	, TMargin(new DriverElements::EigenValuesElements::TMargin())
	, V(new DriverElements::EigenValuesElements::VSerise())
	, G(new Common::EigenValuesElements::GSerise()) {
	deletedA = false;
	deletedPedalChange = false;
	deletedTMargin = false;
	deletedV = false;
	deletedG = false;
}

DriverElements::EigenValues::~EigenValues() {
	if (A != nullptr && !deletedA) {
		delete A;
		deletedA = true;
	}
	if (PedalChange != nullptr && !deletedPedalChange) {
		delete PedalChange;
		deletedPedalChange = true;
	}
	if (TMargin != nullptr && !deletedTMargin) {
		delete TMargin;
		deletedTMargin = true;
	}
	if (V != nullptr && !deletedV) {
		delete V;
		deletedV = true;
	}
	if (G != nullptr && !deletedG) {
		delete G;
		deletedG = true;
	}
}

DriverElements::MomentValues::MomentValues()
	: R(new Common::MomentValuesElements::VelocityGap())
	, pedal(new DriverElements::MomentValuesElements::PedalInformations())
	, v(new DriverElements::MomentValuesElements::VSerise())
	, g(new DriverElements::MomentValuesElements::GSerise()) {
	a = 0;
	recognitionHit = false;
	deletedR = false;
	deletedPedal = false;
	deletedV = false;
	deletedG = false;
}

DriverElements::MomentValues::~MomentValues() {
	if (R != nullptr && !deletedR) {
		delete R;
		deletedR = true;
	}
	if (pedal != nullptr && !deletedPedal) {
		delete pedal;
		deletedPedal = true;
	}
	if (v != nullptr && !deletedV) {
		delete v;
		deletedV = true;
	}
	if (g != nullptr && !deletedG) {
		delete g;
		deletedG = true;
	}
}

DriverStruct::DriverStruct()
	: Eigen(new DriverElements::EigenValues())
	, Moment(new DriverElements::MomentValues()) { 
	deletedEigen = false;
	deletedMoment = false;
}

DriverStruct::~DriverStruct() {
	if (Eigen != nullptr && !deletedEigen) {
		delete Eigen;
		deletedEigen = true;
	}
	if (Moment != nullptr && !deletedMoment) {
		delete Moment;
		deletedMoment = true;
	}
}
