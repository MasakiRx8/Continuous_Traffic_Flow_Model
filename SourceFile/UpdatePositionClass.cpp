/*
	This is cpp file of the class of "UpdatePositionClass" that move the car position by one time step.
	This inherits from "ModelBaseClass".
*/

#include "UpdatePositionClass.h"

//constructor
UpdatePositionClass::UpdatePositionClass(StatisticsClass* const statistics, const PedalChangePackage* const PedalChange, const ModelBaseClass* const baseClass)
	: statistics(statistics), PedalChange(PedalChange), ModelBaseClass(baseClass) {
	InitializeProperties(this);
}

//destructor
UpdatePositionClass::~UpdatePositionClass() { }

/*
	Move the car position by one time step.
*/
void UpdatePositionClass::UpdateCarPosition(const CarStruct* const car) {
	_dX = 0;

	double transitTime;
	const DriverStruct* const driver = car->Driver;

	//Determine the car's actual acceleration for the next timestep.
	DecideNextCarAcceleration(car);

	//Move the car position by one time step.
	CarElements::MomentValues* const carMoment = car->Moment;
	const double& x = carMoment->x;
	const double& v = carMoment->v;
	const double& a = carMoment->a;

	double nextX;
	double nextV = v + a * ModelParameters.deltaT;
	if (nextV > 0) {
		nextX = x + v * ModelParameters.deltaT + 0.5 * a * std::pow(ModelParameters.deltaT, 2);
	}
	else {
		//Due to the model, go backwards is not allowed.
		if (a < 0) {
			nextX = x - 0.5 * std::pow(v, 2) / a;
		}
		else {
			nextX = x;
		}
		nextV = 0;
		carMoment->a = 0;
		DriverElements::MomentValues* driverMoment = driver->Moment;
		if (driverMoment->a < 0) {
			driverMoment->recognitionHit = false;
			DriverElements::MomentValuesElements::PedalInformations* const pedal = driverMoment->pedal;
			pedal->changing = false;
			pedal->timeElapsed = 0;
			pedal->targetFootPosition = FootPositionType::Brake;
			pedal->footPosition = FootPositionType::Brake;
		}
	}
	if (nextX >= ModelParameters.L) {
		nextX -= ModelParameters.L;
	}
	//Get statistics.
	//This model uses the same measurement distance as loop coil vehicle detectors on Japanese expressways.
	CarElements::MomentValuesElements::Measurement* const measurement = carMoment->measurement;
	if (measurement->passed) {
		if (x < StatisticsParameters.MeasurementEndX && nextX >= StatisticsParameters.MeasurementEndX) {
			transitTime = measurement->elapsedTime;
			measurement->Reset();
			transitTime += GetElapsedTime(car, x, StatisticsParameters.MeasurementEndX);
			statistics->IncrementCounter();
			statistics->AddMeasurementSectionTransitTime(transitTime);
		}
		else {
			measurement->elapsedTime += ModelParameters.deltaT;
		}
	}
	else {
		if (x < StatisticsParameters.MeasurementStartX && nextX >= StatisticsParameters.MeasurementStartX) {
			if (nextX < StatisticsParameters.MeasurementEndX) {
				measurement->passed = true;
				measurement->elapsedTime = GetElapsedTime(car, StatisticsParameters.MeasurementStartX, nextX);
			}
			else {
				transitTime = GetElapsedTime(car, StatisticsParameters.MeasurementStartX, StatisticsParameters.MeasurementEndX);
				statistics->IncrementCounter();
				statistics->AddMeasurementSectionTransitTime(transitTime);
			}
		}
	}
	if (x <= nextX) {
		_dX = nextX - x;
	}
	else {
		_dX = nextX + ModelParameters.L - x;
	}
	carMoment->x = std::move(nextX);
	carMoment->v = std::move(nextV);
	_position = carMoment->x;
}

/*
	Determine the car's actual acceleration for the next timestep.
*/
void UpdatePositionClass::DecideNextCarAcceleration(const CarStruct* const car) const {
	CarElements::MomentValues* const carMoment = car->Moment;
	DriverElements::MomentValues* const driverMoment = car->Driver->Moment;
	DriverElements::MomentValuesElements::PedalInformations* const pedal = driverMoment->pedal;
	bool& recognitionHit = driverMoment->recognitionHit;
	switch (PedalChange->DoPedalChange(pedal, recognitionHit)) {
	case PedalChangedStateType::NoChanged:
		break;
	case PedalChangedStateType::Changed:
		carMoment->a = driverMoment->a;
		recognitionHit = false;
		break;
	case PedalChangedStateType::Changing:
		if (carMoment->v > 0) {
			carMoment->a = -car->Eigen->AResistance;
		}
		else {
			carMoment->a = 0;
		}
		break;
	case PedalChangedStateType::ImmediatelyChanged:
		carMoment->a = driverMoment->a;
		recognitionHit = false;
		break;
	default:
		break;
	}
}

double UpdatePositionClass::GetElapsedTime(const CarStruct* const car, const double& x0, const double& x1) const {
	const double& a = car->Moment->a;
	const double& v = car->Moment->v;
	if (a == 0) {
		return (x1 - x0) / v;
	}
	else {
		return (-v + std::sqrt(std::pow(v, 2) + 2 * a * (x1 - x0))) / a;
	}
}

void UpdatePositionClass::InitializeProperties(UpdatePositionClass* const thisPtr) {
	dX(std::bind(&UpdatePositionClass::Get_dX, thisPtr));
	Position(std::bind(&UpdatePositionClass::Get_Position, thisPtr));
}

const double& UpdatePositionClass::Get_dX() const {
	return _dX;
}

const double& UpdatePositionClass::Get_Position() const {
	return _position;
}
