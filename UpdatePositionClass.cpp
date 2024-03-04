/*
	This is cpp file of the class of "UpdatePositionClass" that move the car position by one time step.
	This inherits from "ModelBaseClass".
*/

#include "UpdatePositionClass.h"

//constructor
UpdatePositionClass::UpdatePositionClass(StatisticsClass* const statistics, const ModelBaseClass* const baseClass) : statistics(statistics), ModelBaseClass(baseClass) {
	InitializeProperties(this);
	PedalChange = new PedalChangePackage(ModelParameters.deltaT);
}

//destructor
UpdatePositionClass::~UpdatePositionClass() {
	SafeDelete(PedalChange);	//delete PedalChangePackage
}

/*
	Move the car position by one time step.
*/
void UpdatePositionClass::UpdateCarPosition() {
	_global_dX = 0;

	double nextX;
	double nextV;
	double transitTime;
	for (int i = 0; i < N; i++) {
		const CarStruct* const car = (*cars)[i];
		const DriverStruct* const driver = car->Driver;

		//Determine the car's actual acceleration for the next timestep.
		DecideNextCarAcceleration(car);

		//Move the car position by one time step.
		const double x = car->Moment->x;
		const double v = car->Moment->v;
		const double a = car->Moment->a;
		nextV = v + a * ModelParameters.deltaT;
		if (nextV > 0) {
			nextX = x + v * ModelParameters.deltaT + 0.5 * a * pow(ModelParameters.deltaT, 2);
		}
		else {
			//Due to the model, go backwards is not allowed.
			if (a < 0) {
				nextX = x - 0.5 * pow(v, 2) / a;
			}
			else {
				nextX = x;
			}
			nextV = 0;
			car->Moment->a = 0;
			if (driver->Moment->a < 0) {
				driver->Moment->recognitionHit = false;
				DriverElements::MomentValuesElements::PedalInformations* const pedal = driver->Moment->pedal;
				pedal->changing = false;
				pedal->timeElapsed = 0;
				pedal->targetFoot = FootPosition::Brake;
				pedal->foot = FootPosition::Brake;
			}
		}
		if (nextX > ModelParameters.L) {
			nextX -= ModelParameters.L;
		}
		//Get statistics.
		//This model uses the same measurement distance as loop coil vehicle detectors on Japanese expressways.
		CarElements::MomentValuesElements::Measurement* const measurement = car->Moment->measurement;
		if (measurement->passed) {
			if (x < statisticsParameters.MeasurementEndX && nextX >= statisticsParameters.MeasurementEndX) {
				transitTime = measurement->elapsedTime;
				measurement->Reset();
				transitTime += GetElapsedTime(car, x, statisticsParameters.MeasurementEndX);
				statistics->IncrementCounter();
				statistics->AddMeasurementSectionTransitTime(transitTime);
			}
			else {
				measurement->elapsedTime += ModelParameters.deltaT;
			}
		}
		else {
			if (x < statisticsParameters.MeasurementStartX && nextX >= statisticsParameters.MeasurementStartX) {
				measurement->passed = true;
				measurement->elapsedTime = GetElapsedTime(car, statisticsParameters.MeasurementStartX, nextX);
			}
		}
		if (x <= nextX) {
			_global_dX += nextX - x;
		}
		else {
			_global_dX += nextX + ModelParameters.L - x;
		}
		car->Moment->x = nextX;
		car->Moment->v = nextV;
	}
}

/*
	Determine the car's actual acceleration for the next timestep.
*/
void UpdatePositionClass::DecideNextCarAcceleration(const CarStruct* const car) const {
	DriverElements::MomentValuesElements::PedalInformations* const pedal = car->Driver->Moment->pedal;
	switch (PedalChange->DoPedalChange(pedal, car->Driver->Moment->recognitionHit)) {
	case PedalChangedState::NoChanged:
		break;
	case PedalChangedState::Changed:
		car->Moment->a = car->Driver->Moment->a;
		car->Driver->Moment->recognitionHit = false;
		break;
	case PedalChangedState::Changing:
		if (car->Moment->v > 0) {
			car->Moment->a = -car->Eigen->AResistance;
		}
		else {
			car->Moment->a = 0;
		}
		break;
	case PedalChangedState::ImmediatelyChanged:
		car->Moment->a = car->Driver->Moment->a;
		car->Driver->Moment->recognitionHit = false;
		break;
	default:
		break;
	}
}

double UpdatePositionClass::GetElapsedTime(const CarStruct* const car, const double& x0, const double& x1) const {
	const double a = car->Moment->a;
	const double v = car->Moment->v;
	if (a == 0) {
		return (x1 - x0) / v;
	}
	else {
		return (-v + sqrt(pow(v, 2) + 2 * a * (x1 - x0))) / a;
	}
}

void UpdatePositionClass::InitializeProperties(UpdatePositionClass* const thisPtr) {
	Global_dX(std::bind(&UpdatePositionClass::Get_Global_dX, thisPtr));
}

double UpdatePositionClass::Get_Global_dX() const {
	return _global_dX;
}
