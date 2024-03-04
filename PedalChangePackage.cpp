/*
	This is cpp file of the class of "PedalChangePackage" that performs calculations related to pedal changes.
*/

#include "PedalChangePackage.h"

//constructor
PedalChangePackage::PedalChangePackage(const double& deltaT) : deltaT(deltaT) { }

//destructor
PedalChangePackage::~PedalChangePackage() { }

/*
	Calculated by Eq.(3-4).
*/
double PedalChangePackage::GetAccelToBrakeTime(const CarStruct* const car, const double& v) const {
	const Common::EigenValuesElements::UpperLower* const V = car->Driver->Eigen->PedalChange->V->AccelToBrake;
	const Common::EigenValuesElements::UpperLower* const AccelToBrakeTime = car->Driver->Eigen->PedalChange->T->AccelToBrake;
	if (v > V->Upper) {
		return AccelToBrakeTime->Upper;
	}
	else if (v < V->Lower) {
		return AccelToBrakeTime->Lower;
	}
	else {
		return (AccelToBrakeTime->Upper - AccelToBrakeTime->Lower) / (V->Upper - V->Lower) * (v - V->Lower) + AccelToBrakeTime->Lower;
	}
}

/*
	Calculated by Eq.(3-4).
*/
double PedalChangePackage::GetBrakeToAccelTime(const CarStruct* const car, const double& v) const {
	const Common::EigenValuesElements::UpperLower* const V = car->Driver->Eigen->PedalChange->V->BrakeToAccel;
	const Common::EigenValuesElements::UpperLower* const BrakeToAccelTime = car->Driver->Eigen->PedalChange->T->BrakeToAccel;
	if (v > V->Upper) {
		return BrakeToAccelTime->Upper;
	}
	else if (v < V->Lower) {
		return BrakeToAccelTime->Lower;
	}
	else {
		return (BrakeToAccelTime->Upper - BrakeToAccelTime->Lower) / (V->Upper - V->Lower) * (v - V->Lower) + BrakeToAccelTime->Lower;
	}
}

/*
	Update information such as the time required to switch a pedal.
*/
void PedalChangePackage::UpdatePedalChangingInformations(const CarStruct* const car, const double& nextA) const {
	DriverElements::MomentValuesElements::PedalInformations* const pedal = car->Driver->Moment->pedal;
	const DriverElements::MomentValuesElements::NeedChangingTime* const needT = pedal->t;	//The "accelToBrake" and "breakeToAccel" that are parameters of "needT" are already calculated on function "CalculateGSerise" that is defined on GRecognitionpackage class.

	if (pedal->changing) {
		pedal->timeElapsed += deltaT;
	}
	switch (pedal->foot) {
	case FootPosition::Accel:
		if (nextA > -car->Eigen->AResistance) {
			pedal->changing = false;
			pedal->timeElapsed = 0;
			pedal->targetFoot = FootPosition::Accel;
		}
		else {
			pedal->changing = true;
			if (nextA < -car->Eigen->AResistance) {
				pedal->needTime = needT->accelToBrake;
				pedal->targetFoot = FootPosition::Brake;
			}
			else {
				pedal->needTime = 0;
				pedal->targetFoot = FootPosition::Free;
			}
		}
		break;
	case FootPosition::Brake:
		if (nextA < -car->Eigen->AResistance) {
			pedal->changing = false;
			pedal->timeElapsed = 0;
			pedal->targetFoot = FootPosition::Brake;
		}
		else {
			pedal->changing = true;
			if (nextA > -car->Eigen->AResistance) {
				pedal->needTime = needT->brakeToAccel;
				pedal->targetFoot = FootPosition::Accel;
			}
			else {
				pedal->needTime = 0;
				pedal->targetFoot = FootPosition::Free;
			}
		}
		break;
	case FootPosition::Free:
		if (nextA > -car->Eigen->AResistance) {
			if (pedal->targetFoot != FootPosition::Accel) {
				if (pedal->targetFoot == FootPosition::Brake) {
					if (pedal->timeElapsed < needT->brakeToAccel) {
						pedal->needTime = pedal->timeElapsed;
					}
					else {
						pedal->needTime = needT->brakeToAccel;
					}
				}
				else if (pedal->targetFoot == FootPosition::Free) {
					pedal->needTime = needT->brakeToAccel / 2;
				}
				pedal->changing = true;
				pedal->timeElapsed = 0;
				pedal->targetFoot = FootPosition::Accel;
			}
		}
		else if (nextA < -car->Eigen->AResistance) {
			if (pedal->targetFoot != FootPosition::Brake) {
				if (pedal->targetFoot == FootPosition::Accel) {
					if (pedal->timeElapsed < needT->accelToBrake) {
						pedal->needTime = pedal->timeElapsed;
					}
					else {
						pedal->needTime = needT->accelToBrake;
					}
				}
				else if (pedal->targetFoot == FootPosition::Free) {
					pedal->needTime = needT->accelToBrake / 2;
				}
				pedal->changing = true;
				pedal->timeElapsed = 0;
				pedal->targetFoot = FootPosition::Brake;
			}
		}
		else {
			pedal->changing = false;
			pedal->timeElapsed = 0;
			pedal->targetFoot = FootPosition::Free;
		}
		break;
	default:
		break;
	}
}

/*
	Pedal switching execution.
*/
PedalChangedState PedalChangePackage::DoPedalChange(DriverElements::MomentValuesElements::PedalInformations* const pedal, const bool& recognitionHit) const {
	if (pedal->changing) {
		if (pedal->timeElapsed >= pedal->needTime) {
			pedal->changing = false;
			pedal->timeElapsed = 0;
			pedal->foot = pedal->targetFoot;
			return PedalChangedState::Changed;
		}
		else {
			pedal->foot = FootPosition::Free;
			return PedalChangedState::Changing;
		}
	}
	else {
		if (recognitionHit) {
			pedal->changing = false;
			pedal->timeElapsed = 0;
			pedal->foot = pedal->targetFoot;
			return PedalChangedState::ImmediatelyChanged;
		}
		else {
			return PedalChangedState::NoChanged;
		}
	}
}
