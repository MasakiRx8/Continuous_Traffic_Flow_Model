/*
	This is cpp file of the class of "GRecognitionPackage" that calculates variables related to inter-vehicular distance.
*/

#include "GRecognitionPackage.h"

//constructor
GRecognitionPackage::GRecognitionPackage(const double& deltaT, const double& L, const PedalChangePackage* const PedalChange) : deltaT(deltaT), L(L), PedalChange(PedalChange) { }

//destructor
GRecognitionPackage::~GRecognitionPackage() { }

/*
	Calculated by Eq.(3-5) to (3-7).
*/
void GRecognitionPackage::CalculateGSerise(const CarStruct* const car) const {
	const CarElements::MomentValues* const carMoment = car->Moment;
	const CarElements::MomentValuesElements::ArroundCarInformations* const front = car->Moment->arround->front;

	const DriverStruct* const driver = car->Driver;
	const DriverElements::EigenValues* const driverEigen = driver->Eigen;
	const DriverElements::EigenValuesElements::AccelerationPackage* const A = driverEigen->A;
	const DriverElements::EigenValuesElements::AccelerationSeries* const Deceleration = A->Deceleration;
	const Common::EigenValuesElements::GSerise* const G = driverEigen->G;
	const double& v = carMoment->v;
	const double& x = carMoment->x;
	const double&& v2 = 0.5 * std::pow(v, 2);
	const double&& vf2 = 0.5 * std::pow(front->v, 2);

	DriverElements::MomentValuesElements::NeedChangingTime* const pedalT = driver->Moment->pedal->t;
	CarElements::MomentValuesElements::GapSerise* const g = carMoment->g;

	pedalT->accelToBrake = PedalChange->GetAccelToBrakeTime(car, v);	//Calculated by Eq.(3-4).
	pedalT->brakeToAccel = PedalChange->GetBrakeToAccelTime(car, v);	//Calculated by Eq.(3-4).
	const double&& vT = v * pedalT->accelToBrake;

	double frontX = front->x;
	if (frontX <= x) {
		frontX += L;
	}
	g->gap = frontX - front->Length - x;
	g->closest = (std::max)(vT + v2 / Deceleration->Strong - vf2 / Deceleration->Acceptable + G->Closest, G->Closest);	//Calculated by Eq.(3-5).
	g->cruise = (std::max)(vT + v2 / Deceleration->Normal - vf2 / A->FrontDeceleration->Normal, g->closest + G->Cruise);		//Calculated by Eq.(3-6).
	g->influenced = (std::max)(g->cruise + v * GetTMargin(car), g->cruise + G->Influenced);	//Calculated by Eq.(3-7)
	g->deltaGap->CopyCurrentToLast();	//Copy deltaGap of current to last  before updating current it.
	g->deltaGap->current = g->gap - g->cruise;
}

/*
	Calculated by Eq.(4-6).
*/
double GRecognitionPackage::Calculate_Zg(const CarStruct* const car) const {
	double Zg;
	const CarElements::MomentValuesElements::GapSerise* const g = car->Moment->g;
	const Common::MomentValuesElements::CurrentLast* const deltaGap = g->deltaGap;
	const DriverElements::MomentValuesElements::GSerise* const driver_g = car->Driver->Moment->g;
	const double& Ngc = driver_g->baseNg;
	const double&& Ag = (g->influenced - g->closest) * log(1 + exp(-Ngc / kappa)) - (g->cruise - g->influenced) * log(1 + exp(-1 / kappa));
	const double&& Fg = Calculate_fg(car);
	if (g->gap < g->closest) {
		Zg = 0;
	}
	else {
		if (Fg < driver_g->baseFg) {
			Zg = (g->cruise - g->closest) / Ag * (log((1 + exp(-Ngc / kappa)) / (1 + exp(-1 / kappa))));
		}
		else {
			const double&& Ng = GetNg(g);
			if (g->gap <= g->cruise) {
				Zg = (g->cruise - g->closest) / Ag * (log((1 + exp(-Ng / kappa)) / (1 + exp(-1 / kappa))));
			}
			else {
				Zg = 1 - (g->influenced - g->cruise) / Ag * log(1 + exp(-Ng / kappa));
			}
		}
	}
	if (deltaGap->current <= deltaGap->last) {
		Zg = 1 - Zg;
	}
	return Zg;
}

/*
	Calculated by Eq.(3-8).
*/
double GRecognitionPackage::Calculate_fg(const CarStruct* const car) const {
	return 1.0 / (1 + exp(-GetNg(car->Moment->g) / kappa));
}

/*
	Calculate Ng(delta g(t)) of Eq.(3-8).
*/
double GRecognitionPackage::GetNg(const CarElements::MomentValuesElements::GapSerise* const g) const {
	if (g->gap <= g->cruise) {
		return -2.0 * (g->gap - g->cruise) / (g->cruise - g->closest) - 1;
	}
	else {
		return 2.0 * (g->gap - g->cruise) / (g->influenced - g->cruise) - 1;
	}
}

/*
	Calculate t_margin(v(t)) of Eq.(3-7).
*/
double GRecognitionPackage::GetTMargin(const CarStruct* const car) const {
	const double& v = car->Moment->v;
	const DriverElements::EigenValuesElements::TMargin* const TMargin = car->Driver->Eigen->TMargin;
	const Common::EigenValuesElements::UpperLower* const T = TMargin->T;
	const Common::EigenValuesElements::UpperLower* const V = TMargin->V;
	const double& TUpper = T->Upper;
	const double& TLower = T->Lower;
	const double& VUpper = V->Upper;
	const double& VLower = V->Lower;
	if (v > VUpper) {
		return TUpper;
	}
	else if (v < VLower) {
		return TLower;
	}
	else {
		return (TUpper - TLower) / (VUpper - VLower) * (v - VLower) + TLower;
	}
}
