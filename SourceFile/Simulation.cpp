/*
	This is cpp file of the class of "Simluation" that controls the all calculations of this model.
*/

#include "Simulation.h"

//constructor
Simulation::Simulation(const std::string& IniFileFolderPath, const int& IniFileNumber, const int& RunNumber, const std::string& ResultFileFolderPath, const bool& CreateSnapShot, const std::string& SnapShotFolderPath) 
	: IniFileFolderPath(IniFileFolderPath), IniFileNumber(IniFileNumber), RunNumber(RunNumber)
	, CreateSnapShot(CreateSnapShot), SnapShotFolderPath(SnapShotFolderPath) {
	if (RunNumber == 0) {
		fFDPath = ResultFileFolderPath + R"(/)" + "FD.csv";
		fGlovalVDPath = ResultFileFolderPath + R"(/)" + "Global_VD.csv";
		fLocalVDPath = ResultFileFolderPath + R"(/)" + "Local_VD.csv";
	}
	else {
		fFDPath = ResultFileFolderPath + R"(/)" + "FD" + std::to_string(RunNumber) + ".csv";
		fGlovalVDPath = ResultFileFolderPath + R"(/)" + "Global_VD" + std::to_string(RunNumber) + ".csv";
		fLocalVDPath = ResultFileFolderPath + R"(/)" + "Local_VD" + std::to_string(RunNumber) +  ".csv";
	}
	ModelParameters = new ModelParametersClass(IniFileFolderPath + R"(/ModelParameters.ini)");
	StatisticsParameters = new StatisticsParametersClass(IniFileFolderPath + R"(/StatisticsParameters.ini)");
}

//destructor
Simulation::~Simulation() {
	SafeDelete(ModelParameters);		//delete ModelParametersClass
	SafeDelete(StatisticsParameters);	//delete StatisticsParametersClass
}

/*
	Main Function
	Perform calculations for each number of cars and create results.
*/
void Simulation::simulate() {
	bool&& isFirstSimulation = CreateNLists();
	WriteCSVHeaderToCSV(isFirstSimulation);
#ifdef _OPENMP
#pragma omp parallel for schedule(guided)
#endif //  _OPENMP
	for (int i = 0; i < int(NLists.size()); i++) {
		int N = NLists[i];
		std::stringstream sResultFD;
		std::stringstream sResultGlovalVD;
		std::stringstream sResultLocalVD;
		//Model execution class construct and initialize model.
		AdvanceTimeAndMeasureClass* AdvanceTime = new AdvanceTimeAndMeasureClass(IniFileFolderPath, IniFileNumber, N, *ModelParameters, *StatisticsParameters, CreateSnapShot, RunNumber, SnapShotFolderPath);	
		if (AdvanceTime->InitializeSuccess) {
			AdvanceTime->AdvanceTimeAndMeasure();	//run-up and measurement
			if (AdvanceTime->SuccedMeasure) {
				//create each result stringstreams
				const StatisticsClass* const statistics = AdvanceTime->Statistics();
				const StatisticsElementsClass* const Global = statistics->Global;
				for (std::size_t j = 0; j < statistics->Local->size(); j++) {
					const StatisticsElementsClass* const local = (*statistics->Local)[j];
					sResultFD << N << "," << local->K << "," << local->Counter << "," << j + 1 << std::endl;
					sResultLocalVD << N << "," << local->K << "," << Calculate_m_s_To_Km_h(local->AverageVelocity) << "," << j + 1 << std::endl;
				}
				sResultGlovalVD << N << "," << Global->K << "," << Calculate_m_s_To_Km_h(Global->AverageVelocity) << std::endl;
			}
			if (AdvanceTime->SuccedMeasure) {
#ifdef  _OPENMP
#pragma omp critical
#endif //  _OPENMP
				{
					//write results
					WriteResultToCSV(sResultFD, sResultGlovalVD, sResultLocalVD);
					std::cout << sResultGlovalVD.str();
				}
			}
			else {
#ifdef  _OPENMP
#pragma omp critical
#endif //  _OPENMP
				{
					std::cout << "Error N::" << N << std::endl;
				}
			}
			delete AdvanceTime;	//delete AdvanceTimeAndMeasureClass
		}
	}
}

/*
	A function that creates the NLists excluding those that results have already been created.
*/
bool  Simulation::CreateNLists() {
	bool isFirstSimulation = true;
	std::ifstream ifs(fGlovalVDPath);
	std::size_t listSize = ModelParameters->NMax;
	std::vector<bool> NListsFG(listSize, true);
	if (ifs) {
		std::string S;
		std::getline(ifs, S);
		if (S != "") {
			isFirstSimulation = false;
		}
		int N;
		double val;
		char ch;
		std::stringstream SS;
		while (std::getline(ifs, S)) {
			if (S == "") {
				break;
			}
			SS << S;
			SS >> N >> ch >> val >> ch >> val;
			NListsFG[N - 1] = false;
			listSize--;
			SS.str("");
			SS.clear(std::stringstream::goodbit);
		}
	}
	ifs.close();

	if (listSize > 0) {
		for (std::size_t i = 0; i < NListsFG.size(); i++) {
			if (NListsFG[i]) {
				NLists.emplace_back(int(i + 1));
			}
		}
	}
	return isFirstSimulation;
}

/*
	Write each header to CSV when this is simulated it for the first time.
*/
void Simulation::WriteCSVHeaderToCSV(const bool& isFirstSimulation) {
	if (isFirstSimulation) {
		std::ofstream ofsFD(fFDPath, std::ios::app);
		std::ofstream ofsGlovalVD(fGlovalVDPath, std::ios::app);
		std::ofstream ofsLocalVD(fLocalVDPath, std::ios::app);
		ofsFD << "N,k,Flux,MeasureN" << std::endl;
		ofsGlovalVD << "N,rho,V" << std::endl;
		ofsLocalVD << "N,k,V,MeasureN" << std::endl;
		ofsFD.close();
		ofsGlovalVD.close();
		ofsLocalVD.close();
	}
}

void Simulation::WriteResultToCSV(const std::stringstream& sResultFD, const std::stringstream& sResultGlovalVD, const std::stringstream& sResultLocalVD) {
	std::ofstream ofsFD(fFDPath, std::ios::app);
	std::ofstream ofsGlovalVD(fGlovalVDPath, std::ios::app);
	std::ofstream ofsLocalVD(fLocalVDPath, std::ios::app);
	ofsFD << sResultFD.str();
	ofsGlovalVD << sResultGlovalVD.str();
	ofsLocalVD << sResultLocalVD.str();
	ofsFD.close();
	ofsGlovalVD.close();
	ofsLocalVD.close();
}
