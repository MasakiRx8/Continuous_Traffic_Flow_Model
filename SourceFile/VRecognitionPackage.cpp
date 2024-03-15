/*
	This is cpp file of the class of "VRecognitionPackage" that calculates variables related to speed recognition.
*/

#include "VRecognitionPackage.h"

//constructor
VRecognitionPackage::VRecognitionPackage() {  }

//destructor
VRecognitionPackage::~VRecognitionPackage() {  }

/*
	Calculate v_target of Eq.(4-11). In addtion to calculate delta v of Eq.(3-1) in both cases regardless of the magnitude relationship between v_target(t) and vStattisticsElements(t).
*/
void VRecognitionPackage::CalculateVSerise(const double& fg, const CarStruct* const car) const {
	//Calculate by Eq.(4-11) and Eq.(3-11).
	const DriverStruct* const driver = car->Driver;
	const DriverElements::EigenValuesElements::VSerise* const driverEigenV = driver->Eigen->V;
	const DriverElements::MomentValues* const driverMoment = driver->Moment;
	const CarElements::MomentValues* const carMoment = car->Moment;
	const CarElements::MomentValuesElements::GapSerise* const g = carMoment->g;
	const Common::EigenValuesElements::PlusMinus* const DeltaAtCruise = driverEigenV->DeltaAtCruise;
	const Common::EigenValuesElements::PlusMinus* const DeltaAt0 = driverEigenV->DeltaAt0;
	const double& VCruise = driverEigenV->Cruise;
	const double& vFront = carMoment->arround->front->v;

	DriverElements::MomentValuesElements::VSerise* const vSerise = driverMoment->v;
	double nextTarget;

	if (driverMoment->g->emergency) {
		nextTarget = vFront;
	}
	else {
		if (g->gap > g->cruise) {
			nextTarget = (VCruise - vFront) * fg + vFront;
		}
		else if (g->gap >= g->closest) {
			nextTarget = vFront * (1 - fg);
		}
		else {
			nextTarget = 0;
		}
	}
	vSerise->target = (std::min)(nextTarget, VCruise);
	vSerise->delta->plus = (DeltaAtCruise->Plus - DeltaAt0->Plus) / VCruise * vSerise->target + DeltaAt0->Plus;
	vSerise->delta->minus = (DeltaAtCruise->Minus - DeltaAt0->Minus) / VCruise * vSerise->target + DeltaAt0->Minus;
}

/*
	Calculated by Eq.(4-6).
*/
double VRecognitionPackage::Calculate_Zv(const CarStruct* const car) const {
	const Common::MomentValuesElements::CurrentLast* const deltaV = car->Driver->Moment->v->deltaV;
	const Common::MomentValuesElements::PlusMinus* const delta = car->Driver->Moment->v->delta;
	const double&& Nvt = GetNv(car, 0);	//The secondly argument of 0 that means delta v equal -v_target(t), because the function GetNv's argument is the current velocity, not delta v.
	const double&& Av = (delta->minus + delta->plus) * log(1 + exp(1 / kappa)) - delta->minus * log(1 + exp(-Nvt / kappa));
	const double& v = car->Moment->v;
	const double& vTarget = car->Driver->Moment->v->target;
	const double&& Nv = GetNv(car, v);
	double Zv;
	if (v <= vTarget) {
		Zv = delta->minus / Av * log((1 + exp(-Nv / kappa)) / (1 + exp(-Nvt / kappa)));
	}
	else {
		Zv = 1 - delta->plus / Av * log(1 + exp(-Nv / kappa));
	}
	if (deltaV->current <= deltaV->last) {
		Zv = 1 - Zv;
	}
	return Zv;
}

/*
	Calculated by Eq.(3-1).
*/
double VRecognitionPackage::Calculate_fv(const CarStruct* const car) const {
	return 1.0 / (1 + exp(-GetNv(car, car->Moment->v) / kappa));
}

/*
	Calculate Nv(delta vStattisticsElements(t)) of Eq.(3-1).
	Be careful the argument is the current velocity, not delta v.
*/
double VRecognitionPackage::GetNv(const CarStruct* const car, const double& v) const {
	//Calculate by Eq.(3-1).
	const DriverElements::MomentValuesElements::VSerise* const vSerise = car->Driver->Moment->v;
	if (v <= vSerise->target) {
		return -1 * 2 / vSerise->delta->minus * (v - vSerise->target) - 1;
	}
	else {
		return 2 / vSerise->delta->plus * (v - vSerise->target) - 1;
	}
}
