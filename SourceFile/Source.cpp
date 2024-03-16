/*
	This is the cpp file that is defined main function, and this function calls the class of "Simulation" that controls the all calculations of this model.
	The "IniFileFolderPath" and "ResultFileFolderPath" parameters are defined the path to the folder where the ".ini" initialization file is saved and the ".csv" where the results will be written, respectively.
	If use this code then you need rewrite these parameters according to your usage environment.
*/

#include "Simulation.h"

int main(int argc, char *argv[]) {
	int IniFileNumber;	//".ini" file number (ex:54.ini)
	int RunNumber;	//The execution number of this code
	bool CreateSnapShot;	//Whether to create a snapshot of each time step during the measurement period.
	bool CloseWhenFinished;
	if (argc == 1) {
		IniFileNumber = 54;
		RunNumber = 0;
		CreateSnapShot = true;
		CloseWhenFinished = false;
	}
	else {
		if (argc != 5) {
			throw std::invalid_argument("Need Specify 'IniFileNumber' And 'RunNumber'");
			return -1;
		}
		IniFileNumber = std::stoi(argv[1]);
		RunNumber = std::stoi(argv[2]);
		if (std::stoi(argv[3]) == 0) {
			CreateSnapShot = false;
		}
		else {
			CreateSnapShot = true;
		}
		if (std::stoi(argv[4]) == 0) {
			CloseWhenFinished = false;
		}
		else {
			CloseWhenFinished = true;
		}
	}
	std::string IniFileFolderPath = R"(./IniFiles)";	//The path to the folder where the ".ini" initialization file is saved.
	std::string ResultFileFolderPath = R"(./Result)";	//The path to the ".csv" where the results will be written.
	std::string SnapShotFolderPath = ResultFileFolderPath + R"(/SnapShot/Ini)" + std::to_string(IniFileNumber);	//The path to the ".csv" where the snap shots will be written.

	Simulation simulation(IniFileFolderPath, IniFileNumber, RunNumber, ResultFileFolderPath, CreateSnapShot, SnapShotFolderPath);
	simulation.simulate();
	std::cout << "FINSH!" << std::endl;
	if (!CloseWhenFinished) {
		getchar();
	}
	return 0;
}