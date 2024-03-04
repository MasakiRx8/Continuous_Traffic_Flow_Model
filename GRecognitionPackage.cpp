/*
	This is cpp file of the class of "GRecognitionPackage" that calculates variables related to inter-vehicular distance.
*/

#include "GRecognitionPackage.h"

//constructor
GRecognitionPackage::GRecognitionPackage(const double& deltaT, const double& L) : deltaT(deltaT), L(L) {
	PedalChange = new PedalChangePackage(deltaT);
}

//destructor
GRecognitionPackage::~GRecognitionPackage() {
	SafeDelete(PedalChange);	//delete PedalChangePackage
}

/*
	Calculated by Eq.(3-5) to (3-7).
*/
void GRecognitionPackage::CalculateGSerise(const CarStruct* const car) const {
	const double* const v = &(car->Moment->v);
	car->Driver->Moment->pedal->t->accelToBrake = PedalChange->GetAccelToBrakeTime(car, *v);	//Calculated by Eq.(3-4).
	car->Driver->Moment->pedal->t->brakeToAccel = PedalChange->GetBrakeToAccelTime(car, *v);	//Calculated by Eq.(3-4).
	const CarElements::MomentValuesElements::ArroundCarInformations* const front = car->Moment->arround->front;
	const DriverElements::EigenValuesElements::AccelerationPackage* const A = car->Driver->Eigen->A;
	const Common::EigenValuesElements::GSerise* const G = car->Driver->Eigen->G;
	const double* const x = &(car->Moment->x);
	const double vT = *v * car->Driver->Moment->pedal->t->accelToBrake;
	const double v2 = 0.5 * pow(*v, 2);
	const double vf2 = 0.5 * pow(front->v, 2);
	double xFrontRear = front->x - front->Length;

	CarElements::MomentValuesElements::GapSerise* const g = car->Moment->g;

	if (xFrontRear < 0) {
		xFrontRear += L;
	}
	g->gap = xFrontRear - *x;
	if (g->gap < 0) {
		g->gap += L;
	}
	g->closest = (std::max)(vT + v2 / A->Deceleration->Strong - vf2 / A->Deceleration->Acceptable + G->Closest, G->Closest);	//Calculated by Eq.(3-5).
	g->cruise = (std::max)(vT + v2 / A->Deceleration->Normal - vf2 / A->FrontDeceleration->Normal, g->closest + G->Cruise);		//Calculated by Eq.(3-6).
	g->influenced = (std::max)(g->cruise + *v * GetTMargin(car), g->cruise + G->Influenced);	//Calculated by Eq.(3-7)
	g->deltaGap->CopyCurrentToLast();	//Copy deltaGap of current to last  before updating current it.
	g->deltaGap->current = g->gap - g->cruise;
}

/*
	Calculated by Eq.(4-6).
*/
double GRecognitionPackage::Calculate_Zg(const CarStruct* const car) const {
	double Zg;
	const CarElements::MomentValuesElements::GapSerise* const g = car->Moment->g;
	const DriverElements::MomentValuesElements::GSerise* const driver_g = car->Driver->Moment->g;
	const double Ngc = driver_g->baseNg;
	const double Ag = (g->influenced - g->closest) * log(1 + exp(-Ngc / kappa)) - (g->cruise - g->influenced) * log(1 + exp(-1 / kappa));
	const double Fg = Calculate_fg(car);
	if (g->gap < g->closest) {
		Zg = 0;
	}
	else {
		if (Fg < driver_g->baseFg) {
			Zg = (g->cruise - g->closest) / Ag * (log((1 + exp(-Ngc / kappa)) / (1 + exp(-1 / kappa))));
		}
		else {
			if (g->gap <= g->cruise) {
				Zg = (g->cruise - g->closest) / Ag * (log((1 + exp(-GetNg(g) / kappa)) / (1 + exp(-1 / kappa))));
			}
			else {
				Zg = 1 - (g->influenced - g->cruise) / Ag * log(1 + exp(-GetNg(g) / kappa));
			}
		}
	}
	if (g->deltaGap->current <= g->deltaGap->last) {
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
	const DriverElements::EigenValuesElements::TMargin* const TMargin = car->Driver->Eigen->TMargin;
	if (car->Moment->v > TMargin->V->Upper) {
		return TMargin->T->Upper;
	}
	else if (car->Moment->v < TMargin->V->Lower) {
		return TMargin->T->Lower;
	}
	else {
		return (TMargin->T->Upper - TMargin->T->Lower) / (TMargin->V->Upper - TMargin->V->Lower) * (car->Moment->v - TMargin->V->Lower) + TMargin->T->Lower;
	}
}
