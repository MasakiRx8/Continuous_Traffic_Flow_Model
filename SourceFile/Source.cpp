/*
	This is the cpp file that is defined main function, and this function calls the class of "Simulation" that controls the all calculations of this model.
	The "IniFileFolderPath" and "ResultFileFolderPath" parameters are defined the path to the folder where the ".ini" initialization file is saved and the ".csv" where the results will be written, respectively.
	If use this code then you need rewrite these parameters according to your usage environment.
*/

#include "Simulation.h"

int main() {
	int IniFileNum = 55;	//".ini" file number (ex:54.ini)
	std::string IniFileFolderPath = R"(./IniFiles)";	//the path to the folder where the ".ini" initialization file is saved
	std::string ResultFileFolderPath = R"(./Result)";	//the path to the ".csv" where the results will be written

	Simulation simulation(IniFileFolderPath, IniFileNum, ResultFileFolderPath);
	simulation.simulate();
	std::cout << "FINSH!" << std::endl;
	getchar();
	return 0;
}