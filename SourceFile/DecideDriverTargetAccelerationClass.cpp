/*
	This is cpp file of the class of "DecideDriverTargetAccelerationClass" that determine the next timestep acceleration for each driver.
	This inherits from "ModelBaseClass".
*/

#include "DecideDriverTargetAccelerationClass.h"

//constructor
DecideDriverTargetAccelerationClass::DecideDriverTargetAccelerationClass(const ModelBaseClass* const baseClass) : ModelBaseClass(baseClass) {
	VRecognition = new VRecognitionPackage();
	GRecognition = new GRecognitionPackage(ModelParameters.deltaT, ModelParameters.L);
	PedalChange = new PedalChangePackage(ModelParameters.deltaT);
	AvoidCollision = new AvoidCollisionPackage(ModelParameters.deltaT);
}

//destructor
DecideDriverTargetAccelerationClass::~DecideDriverTargetAccelerationClass() {
	SafeDelete(VRecognition);	//delete VRecognitionPackage
	SafeDelete(GRecognition);	//delete GRecognitionPackage
	SafeDelete(PedalChange);	//delete PedalChangePackage
	SafeDelete(AvoidCollision);	//delete AvoidCollisionPackage
}

/*
	Determine the target acceleration of the next time step.
*/
void DecideDriverTargetAccelerationClass::DecideDriverTargetAcceleration(const CarStruct* const car) {
	//Calculate by Eq.(4-12)
	bool recognitionHit;
	double nextA;
	const DriverStruct* const driver = car->Driver;

	Common::MomentValuesElements::VelocityGap* const R = driver->Moment->R;
	recognitionHit = false;

	//First, calculate the g series.
	GRecognition->CalculateGSerise(car);
	if (AvoidCollision->IsEmergency(car)) {
		driver->Moment->g->emergency = true;
		recognitionHit = true;
	}
	else {
		driver->Moment->g->emergency = false;
		if (car->Eigen->DriverMode == DriverMode::Human) {
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
	if (recognitionHit) {
		//Recalculate v_target by Eq.(4-11)
		if (car->Eigen->DriverMode == DriverMode::Human) {
			R->gap = 1 - (*random)(1.0);
		}
		VRecognition->CalculateVSerise(GRecognition->Calculate_fg(car), car);
		driver->Moment->recognitionHit = true;
	}
	driver->Moment->v->deltaV->CopyCurrentToLast();	//Copy deltaV of current to last  before updating current it.
	driver->Moment->v->deltaV->current = car->Moment->v - driver->Moment->v->target;
	if (!recognitionHit) {
		//Calculate Zv by Eq.(4-3)
		if (R->velocity <= VRecognition->Calculate_Zv(car)) {
			R->velocity = 1 - (*random)(1.0);
			driver->Moment->recognitionHit = true;
			recognitionHit = true;
		}
	}
	//Determine the target acceleration of the next time step.
	if (recognitionHit) {
		nextA = CalculateNextA(car);
		PedalChange->UpdatePedalChangingInformations(car, nextA);
		driver->Moment->a = nextA;
	}
}

/*
	Calculate the target acceleration of the next time step using Eq.(4-12). 
*/
double DecideDriverTargetAccelerationClass::CalculateNextA(const CarStruct* const car) {
	double nextA;
	const double fv = VRecognition->Calculate_fv(car);
	const CarElements::MomentValuesElements::GapSerise* const g = car->Moment->g;

	if (car->Driver->Moment->g->emergency) {
		//Calculated by Eq.(4-12).
		nextA = AvoidCollision->GetEmergencyAcceleration(car);
	}
	else {
		//Calculated by Eq.(3-13).
		if (g->gap < g->closest) {
			nextA = -car->Driver->Eigen->A->Deceleration->Acceptable;
		}
		else {
			double amax;
			const double fg = GRecognition->Calculate_fg(car);
			const DriverElements::EigenValuesElements::AccelerationSeries* const Acceleration = car->Driver->Eigen->A->Acceleration;
			const DriverElements::EigenValuesElements::AccelerationSeries* const Deceleration = car->Driver->Eigen->A->Deceleration;
			if (car->Driver->Moment->v->target >= car->Moment->v) {
				const double frontA = car->Moment->arround->front->a;
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
