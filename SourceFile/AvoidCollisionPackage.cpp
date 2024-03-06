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
	const DriverElements::EigenValuesElements::AccelerationSeries* const Deceleration = car->Driver->Eigen->A->Deceleration;
	const double currentV = car->Moment->v;
	const double currentVf = car->Moment->arround->front->v;
	const double currentA = car->Moment->a;
	double dx;
	double dxF;
	double v = currentV + currentA * deltaT;
	double vf = currentVf - Deceleration->Acceptable * deltaT;

	if (v < 0) {
		v = 0;
		dx = -0.5 * std::pow(currentV, 2) / currentA;
	}
	else {
		dx = currentV * deltaT + 0.5 * currentA * std::pow(deltaT, 2);
	}
	if (vf < 0) {
		vf = 0;
		dxF = 0.5 * std::pow(currentVf, 2) / Deceleration->Acceptable;
	}
	else {
		dxF = currentVf * deltaT - 0.5 * Deceleration->Acceptable * std::pow(deltaT, 2);
	}
	const double tPedalChange = PedalChange->GetAccelToBrakeTime(car, v);
	const double expectedGClosest = v * tPedalChange + 0.5 * std::pow(v, 2) / Deceleration->Strong - 0.5 * std::pow(vf, 2) / Deceleration->Acceptable + car->Driver->Eigen->G->Closest;
	const double expectedG = car->Moment->g->gap + dxF - dx;

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
	double dxFront;
	double nextA;
	switch (car->Driver->Moment->pedal->foot) {
	case FootPosition::Brake:
		dxFront = 0.5 * std::pow(car->Moment->arround->front->v, 2) / car->Driver->Eigen->A->Deceleration->Acceptable;
		nextA = -0.5 * std::pow(car->Moment->v, 2) / (car->Moment->g->gap + dxFront - car->Driver->Eigen->G->Closest);
		break;
	default:
		nextA = -car->Driver->Eigen->A->Deceleration->Strong;
		break;
	}
	return nextA;
}