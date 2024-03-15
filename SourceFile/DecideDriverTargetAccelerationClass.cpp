/*
	This is cpp file of the class of "DecideDriverTargetAccelerationClass" that determine the next timestep acceleration for each driver.
	This inherits from "ModelBaseClass".
*/

#include "DecideDriverTargetAccelerationClass.h"

//constructor
DecideDriverTargetAccelerationClass::DecideDriverTargetAccelerationClass(const PedalChangePackage* const PedalChange, const ModelBaseClass* const baseClass) 
	: ModelBaseClass(baseClass)
	, PedalChange(PedalChange)
	, VRecognition(new VRecognitionPackage())
	, GRecognition(new GRecognitionPackage(ModelParameters.deltaT, ModelParameters.L, PedalChange))
	, AvoidCollision(new AvoidCollisionPackage(ModelParameters.deltaT)) {
	deletedVRecognition = false;
	deletedGRecognition = false;
	deletedAvoidCollision = false;
}

//destructor
DecideDriverTargetAccelerationClass::~DecideDriverTargetAccelerationClass() {
	if (!deletedVRecognition) {
		delete VRecognition;	//delete VRecognitionPackage
		deletedVRecognition = true;
	}
	if (!deletedGRecognition) {
		delete GRecognition;	//delete GRecognitionPackage
		deletedGRecognition = true;
	}
	if (!deletedAvoidCollision) {
		delete AvoidCollision;	//delete AvoidCollisionPackage
		deletedAvoidCollision = true;
	}
}

/*
	Determine the target acceleration of the next time step.
*/
void DecideDriverTargetAccelerationClass::DecideDriverTargetAcceleration(const CarStruct* const car) {
	//Calculate by Eq.(4-12)
	const DriverStruct* const driver = car->Driver;
	DriverElements::MomentValues* const driverMoment = driver->Moment;
	DriverElements::MomentValuesElements::VSerise* const driverMomentV = driverMoment->v;
	Common::MomentValuesElements::VelocityGap* const R = driverMoment->R;
	Common::MomentValuesElements::CurrentLast* const deltaV = driverMomentV->deltaV;
	bool& recognitionHit = driverMoment->recognitionHit;
	bool& emergency = driverMoment->g->emergency;
	recognitionHit = false;
	emergency = false;

	//First, calculate the g series.
	GRecognition->CalculateGSerise(car);
	if (AvoidCollision->IsEmergency(car)) {
		emergency = true;
	}
	else {
		if (car->Eigen->DriverMode == DriverModeType::Human) {
			//Calculate Zg by Eq.(4-6)
			if (R->gap <= GRecognition->Calculate_Zg(car)) {
				recognitionHit = true;
			}
		}
		else {
			//Auto Mode.
			//Everytime Hit.
			recognitionHit = true;
		}
	}
	if (recognitionHit || emergency) {
		//Recalculate v_target by Eq.(4-11)
		if (car->Eigen->DriverMode == DriverModeType::Human) {
			R->gap = 1 - (*random)(1.0);
		}
		VRecognition->CalculateVSerise(GRecognition->Calculate_fg(car), car);
	}
	deltaV->CopyCurrentToLast();	//Copy deltaV of current to last  before updating current it.
	deltaV->current = car->Moment->v - driverMomentV->target;
	if (!recognitionHit) {
		//Calculate Zv by Eq.(4-3)
		if (R->velocity <= VRecognition->Calculate_Zv(car)) {
			R->velocity = 1 - (*random)(1.0);
			recognitionHit = true;
		}
	}
	//Determine the target acceleration of the next time step.
	if (recognitionHit || emergency) {
		double&& nextA = CalculateNextA(car);
		PedalChange->UpdatePedalChangingInformations(car, nextA);
		driverMoment->a = std::move(nextA);
	}
}

/*
	Calculate the target acceleration of the next time step using Eq.(4-12). 
*/
double DecideDriverTargetAccelerationClass::CalculateNextA(const CarStruct* const car) {
	double nextA;
	const double&& fv = VRecognition->Calculate_fv(car);
	const CarElements::MomentValues* const carMoment = car->Moment;
	const CarElements::MomentValuesElements::GapSerise* const g = carMoment->g;
	const DriverStruct* const driver = car->Driver;
	const DriverElements::MomentValues* const driverMoment = driver->Moment;
	const DriverElements::EigenValuesElements::AccelerationPackage* const driverEigenA = driver->Eigen->A;
	const DriverElements::EigenValuesElements::AccelerationSeries* const Acceleration = driverEigenA->Acceleration;
	const DriverElements::EigenValuesElements::AccelerationSeries* const Deceleration = driverEigenA->Deceleration;

	if (driverMoment->g->emergency) {
		//Calculated by Eq.(4-12).
		nextA = AvoidCollision->GetEmergencyAcceleration(car);
	}
	else {
		//Calculated by Eq.(3-13).
		if (g->gap < g->closest) {
			nextA = -Deceleration->Acceptable;
		}
		else {
			double amax;
			const double&& fg = GRecognition->Calculate_fg(car);
			if (driverMoment->v->target >= carMoment->v) {
				const double& frontA = carMoment->arround->front->a;
				if (g->gap <= g->cruise) {
					amax = frontA * (1 - fg);
				}
				else {
					amax = std::abs(Acceleration->Acceptable - frontA) * fg + frontA;
				}
			}
			else {
				if (g->gap <= g->cruise) {
					amax = -1.0 * (Deceleration->Strong - Deceleration->Normal) * fg - Deceleration->Normal;
				}
				else {
					amax = (Deceleration->Normal - car->Eigen->AResistance) * fg - Deceleration->Normal;
				}
			}
			amax = (std::max)(-Deceleration->Acceptable, amax);
			nextA = (std::min)(amax, Acceleration->Acceptable) * fv;
		}
	}
	return nextA;
}
