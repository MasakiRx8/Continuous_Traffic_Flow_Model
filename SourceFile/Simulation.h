/*
	This is header file of the class of "Simluation" that controls the all calculations of this model.
*/

#ifndef SIMULATION_H
#define SIMULATION_H
#include <fstream>
#include <sstream>
#include <string>
#include "ModelParametersClass.h"
#include "StatisticsParametersClass.h"
#include "AdvanceTimeAndMeasureClass.h"

class Simulation {
public:
	Simulation(const std::string& IniFileFolderPath, const int& IniFileNumber, const int& RunNumber, const std::string& ResultFileFolderPath, const bool& CreateSnapShot, const std::string& SnapShotFolderPath);	//constructor
	~Simulation();	//destructor
	void simulate();	//main function
private:
	const std::string IniFileFolderPath;
	const int IniFileNumber;
	const int RunNumber;
	const bool CreateSnapShot;
	const std::string SnapShotFolderPath;

	const ModelParametersClass* ModelParameters;				//Model parameters such as road length
	const StatisticsParametersClass* StatisticsParameters;	//Parameters for measuring results
	std::vector<int> NLists;	//List of number of cars to be calculated
	//The following is related to result creation.
	std::string fFDPath;
	std::string fGlovalVDPath;
	std::string fLocalVDPath;

	bool CreateNLists();		//A function that creates the NLists excluding those that results have already been created.
	void WriteCSVHeaderToCSV(const bool& isFirstSimulation);	//Write each header to CSV when this is simulated it for the first time.
	void WriteResultToCSV(const std::stringstream& sResultFD, const std::stringstream& sResultGlovalVD, const std::stringstream& sResultLocalVD);
};

#endif // !SIMULATION_H
