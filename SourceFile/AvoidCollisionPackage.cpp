/*
	This is cpp file of the class of "AvoidCollisionPackage" that is for avoiding collisions.
*/

#include "AvoidCollisionPackage.h"

AvoidCollisionPackage::AvoidCollisionPackage(const double& deltaT) : deltaT(deltaT) { 
	PedalChange = new PedalChangePackage(deltaT);
}

AvoidCollisionPackage::~AvoidCollisionPackage() {
	SafeDelete(PedalChange);	//delete PedalChangePackage
}

/*
	Judged by Eq.(4-9).
*/
bool AvoidCollisionPackage::IsEmergency(const CarStruct* const car) const {
	const CarElements::MomentValues* const carMoment = car->Moment;
	const DriverElements::EigenValues* const driverEigen = car->Driver->Eigen;
	const DriverElements::EigenValuesElements::AccelerationSeries* const Deceleration = driverEigen->A->Deceleration;
	const double& DAcceptable = Deceleration->Acceptable;
	const double& currentV = carMoment->v;
	const double& currentVf = carMoment->arround->front->v;
	const double& currentA = carMoment->a;
	double dx;
	double dxF;
	double&& v = currentV + currentA * deltaT;
	double&& vf = currentVf - DAcceptable * deltaT;

	if (v < 0) {
		v = 0;
		dx = -0.5 * std::pow(currentV, 2) / currentA;
	}
	else {
		dx = currentV * deltaT + 0.5 * currentA * std::pow(deltaT, 2);
	}
	if (vf < 0) {
		vf = 0;
		dxF = 0.5 * std::pow(currentVf, 2) / DAcceptable;
	}
	else {
		dxF = currentVf * deltaT - 0.5 * DAcceptable * std::pow(deltaT, 2);
	}
	const double& tPedalChange = PedalChange->GetAccelToBrakeTime(car, v);
	const double&& expectedGClosest = v * tPedalChange + 0.5 * std::pow(v, 2) / Deceleration->Strong - 0.5 * std::pow(vf, 2) / DAcceptable + driverEigen->G->Closest;
	const double&& expectedG = carMoment->g->gap + dxF - dx;

	if (expectedG < expectedGClosest) {
		return true;
	}
	else {
		return false;
	}
}

/*
	Calculate a_emergency of Eq.(4-12).
*/
double AvoidCollisionPackage::GetEmergencyAcceleration(const CarStruct* const car) const {
	const CarElements::MomentValues* const carMoment = car->Moment;
	const DriverStruct* const driver = car->Driver;
	const DriverElements::EigenValues* const driverEigen = driver->Eigen;
	const DriverElements::EigenValuesElements::AccelerationSeries* const Deceleration = driverEigen->A->Deceleration;

	double dxFront;
	double nextA;
	switch (driver->Moment->pedal->footPosition) {
	case FootPositionType::Brake:
		dxFront = 0.5 * std::pow(carMoment->arround->front->v, 2) / Deceleration->Acceptable;
		nextA = -0.5 * std::pow(carMoment->v, 2) / (carMoment->g->gap + dxFront - driverEigen->G->Closest);
		break;
	default:
		nextA = -Deceleration->Strong;
		break;
	}
	return nextA;
}