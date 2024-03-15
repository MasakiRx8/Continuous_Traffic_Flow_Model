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
	const DriverElements::EigenValuesElements::PedalChangingTimeInformations* const pedalChange = car->Driver->Eigen->PedalChange;
	const Common::EigenValuesElements::UpperLower* const V = pedalChange->V->AccelToBrake;
	const Common::EigenValuesElements::UpperLower* const AccelToBrakeTime = pedalChange->T->AccelToBrake;
	const double& VUpper = V->Upper;
	const double& VLower = V->Lower;
	const double& TUpper = AccelToBrakeTime->Upper;
	const double& TLower = AccelToBrakeTime->Lower;
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

/*
	Calculated by Eq.(3-4).
*/
double PedalChangePackage::GetBrakeToAccelTime(const CarStruct* const car, const double& v) const {
	const DriverElements::EigenValuesElements::PedalChangingTimeInformations* const pedalChange = car->Driver->Eigen->PedalChange;
	const Common::EigenValuesElements::UpperLower* const V = pedalChange->V->BrakeToAccel;
	const Common::EigenValuesElements::UpperLower* const BrakeToAccelTime = pedalChange->T->BrakeToAccel;
	const double& VUpper = V->Upper;
	const double& VLower = V->Lower;
	const double& TUpper = BrakeToAccelTime->Upper;
	const double& TLower = BrakeToAccelTime->Lower;
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

/*
	Update information such as the time required to switch a pedal.
*/
void PedalChangePackage::UpdatePedalChangingInformations(const CarStruct* const car, const double& nextA) const {
	DriverElements::MomentValuesElements::PedalInformations* const pedal = car->Driver->Moment->pedal;
	const DriverElements::MomentValuesElements::NeedChangingTime* const needT = pedal->t;	//The "accelToBrake" and "breakeToAccel" that are parameters of "needT" are already calculated on function "CalculateGSerise" that is defined on GRecognitionpackage class.

	if (pedal->changing) {
		pedal->timeElapsed += deltaT;
	}
	switch (pedal->footPosition) {
	case FootPositionType::Accel:
		if (nextA > -car->Eigen->AResistance) {
			pedal->changing = false;
			pedal->timeElapsed = 0;
			pedal->targetFootPosition = FootPositionType::Accel;
		}
		else {
			pedal->changing = true;
			if (nextA < -car->Eigen->AResistance) {
				pedal->needTime = needT->accelToBrake;
				pedal->targetFootPosition = FootPositionType::Brake;
			}
			else {
				pedal->needTime = 0;
				pedal->targetFootPosition = FootPositionType::Free;
			}
		}
		break;
	case FootPositionType::Brake:
		if (nextA < -car->Eigen->AResistance) {
			pedal->changing = false;
			pedal->timeElapsed = 0;
			pedal->targetFootPosition = FootPositionType::Brake;
		}
		else {
			pedal->changing = true;
			if (nextA > -car->Eigen->AResistance) {
				pedal->needTime = needT->brakeToAccel;
				pedal->targetFootPosition = FootPositionType::Accel;
			}
			else {
				pedal->needTime = 0;
				pedal->targetFootPosition = FootPositionType::Free;
			}
		}
		break;
	case FootPositionType::Free:
		if (nextA > -car->Eigen->AResistance) {
			if (pedal->targetFootPosition != FootPositionType::Accel) {
				if (pedal->targetFootPosition == FootPositionType::Brake) {
					if (pedal->timeElapsed < needT->brakeToAccel) {
						pedal->needTime = pedal->timeElapsed;
					}
					else {
						pedal->needTime = needT->brakeToAccel;
					}
				}
				else if (pedal->targetFootPosition == FootPositionType::Free) {
					pedal->needTime = needT->brakeToAccel / 2;
				}
				pedal->changing = true;
				pedal->timeElapsed = 0;
				pedal->targetFootPosition = FootPositionType::Accel;
			}
		}
		else if (nextA < -car->Eigen->AResistance) {
			if (pedal->targetFootPosition != FootPositionType::Brake) {
				if (pedal->targetFootPosition == FootPositionType::Accel) {
					if (pedal->timeElapsed < needT->accelToBrake) {
						pedal->needTime = pedal->timeElapsed;
					}
					else {
						pedal->needTime = needT->accelToBrake;
					}
				}
				else if (pedal->targetFootPosition == FootPositionType::Free) {
					pedal->needTime = needT->accelToBrake / 2;
				}
				pedal->changing = true;
				pedal->timeElapsed = 0;
				pedal->targetFootPosition = FootPositionType::Brake;
			}
		}
		else {
			pedal->changing = false;
			pedal->timeElapsed = 0;
			pedal->targetFootPosition = FootPositionType::Free;
		}
		break;
	default:
		break;
	}
}

/*
	Pedal switching execution.
*/
PedalChangedStateType PedalChangePackage::DoPedalChange(DriverElements::MomentValuesElements::PedalInformations* const pedal, const bool& recognitionHit) const {
	if (pedal->changing) {
		if (pedal->timeElapsed >= pedal->needTime) {
			pedal->changing = false;
			pedal->timeElapsed = 0;
			pedal->footPosition = pedal->targetFootPosition;
			return PedalChangedStateType::Changed;
		}
		else {
			pedal->footPosition = FootPositionType::Free;
			return PedalChangedStateType::Changing;
		}
	}
	else {
		if (recognitionHit) {
			pedal->changing = false;
			pedal->timeElapsed = 0;
			pedal->footPosition = pedal->targetFootPosition;
			return PedalChangedStateType::ImmediatelyChanged;
		}
		else {
			return PedalChangedStateType::NoChanged;
		}
	}
}
