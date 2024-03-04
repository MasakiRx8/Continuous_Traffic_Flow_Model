/*
	This is header file of the class of "InitializerClass" that reads car and driver-specific information from the ".ini" file and determines the initial position of the car.
	This inherits from "ModelBaseClass".
*/

#ifndef INITIALIZERCLASS_H
#define INITIALIZERCLASS_H
#include <algorithm>
#include "Common.h"
#include "ModelBaseClass.h"
#include "ReadIniFilePackage.h"
#include "ReadOnlyPropertyClass.h"

class InitializerClass : public ModelBaseClass {
public:
	InitializerClass(const ModelBaseClass* myBase, const std::string& IniFileFolderPath, const int& IniFileNumber);	//constructor
	~InitializerClass();	//destructor

	bool Initialize();	//Initialize all parameters of car and driver, in addition, initializes the set positions of all cars.
private:
	struct ToLower {
		char operator()(const char& c);
	};

	ReadIniFilePackage* ReadIniFile;
	double allCarLength;
	double allDclosest;

	void InitializeCarsAndDrivers();	//Initialize all parameters of car and driver reading ".ini" file.
	bool InitializePosition() const;			//Initializes the set positions of all cars.
	bool EqualizeAllGap() const;	//Set up all cars with an equal distance between them.
	void MoveBetweenFrontAndRearCars(const int& ID) const;	//Move the vehicle to a random position between the previous and following vehicles.

	void InitializeProperties(InitializerClass* const thisPtr);

	double Get_GlobalK() const;
public:
	ReadOnlyPropertyClass<double> GlobalK;
};

#endif // !INITIALIZERCLASS_H
