/*
	This is cpp file of the class of "Simluation" that controls the all calculations of this model.
*/

#include "Simulation.h"

//constructor
Simulation::Simulation(const std::string& IniFileFolderPath, const int& IniFileNumber, const std::string& ResultFileFolderPath) : IniFileFolderPath(IniFileFolderPath), IniFileNumber(IniFileNumber) {
	fFDPath = ResultFileFolderPath + R"(\FD.csv)";
	fGlovalVDPath = ResultFileFolderPath + R"(\Global_VD.csv)";
	fLocalVDPath = ResultFileFolderPath + R"(\Local_VD.csv)";
	ModelParameters = new ModelParametersClass(IniFileFolderPath + R"(\ModelParameters.ini)");
	statisticsParameters = new StatisticsParametersClass(IniFileFolderPath + R"(\StatisticsParameters.ini)");
}

//destructor
Simulation::~Simulation() {
	SafeDelete(ModelParameters);		//delete ModelParametersClass
	SafeDelete(statisticsParameters);	//delete StatisticsParametersClass
}

/*
	Main Function
	Perform calculations for each number of cars and create results.
*/
void Simulation::simulate() {
	CreateNLists();
	WriteCSVHeaderToCSV();
#ifdef _OPENMP
#pragma omp parallel for schedule(guided)
#endif //  _OPENMP
	for (int i = 0; i < int(NLists.size()); i++) {
		int N = NLists[i];
		std::stringstream sResultFD;
		std::stringstream sResultGlovalVD;
		std::stringstream sResultLocalVD;
		AdvanceTimeAndMeasureClass* AdvanceTime = new AdvanceTimeAndMeasureClass(N, *ModelParameters, *statisticsParameters);	//model execution class
		if (AdvanceTime->Initialize(IniFileFolderPath, IniFileNumber)) {	//initialization
			AdvanceTime->AdvanceTimeAndMeasure();	//run-up and measurement
			if (AdvanceTime->SuccedMeasure) {
				//create each result stringstreams
				const StatisticsClass* statistics = AdvanceTime->Statistics();
				const StatisticsElementsClass* const Global = statistics->Global;
				const StatisticsElementsClass* local;
				for (std::size_t j = 0; j < statistics->Local->size(); j++) {
					local = (*statistics->Local)[j];
					sResultFD << local->K << "," << local->Counter << std::endl;
					sResultLocalVD << local->K << "," << Calculate_m_s_To_Km_h(local->AverageVelocity) << std::endl;
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
	ofsFD.close();
	ofsGlovalVD.close();
	ofsLocalVD.close();
}

/*
	A function that creates the NLists excluding those that results have already been created.
*/
void Simulation::CreateNLists() {
	isFirstSimulation = true;
	std::ifstream ifs(fGlovalVDPath);
	std::vector<bool> NListsFG(ModelParameters->NMax, true);
	int listSize = ModelParameters->NMax;
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
			SS.clear();
		}
	}
	ifs.close();

	if (listSize > 0) {
		for (std::size_t i = 0; i < NListsFG.size(); i++) {
			if (NListsFG[i]) {
				NLists.emplace_back(i + 1);
			}
		}
	}
}

/*
	Initialize each result ofstreams, and write each header to CSV when this is simulated it for the first time.
*/
void Simulation::WriteCSVHeaderToCSV() {
	ofsFD = std::ofstream(fFDPath, std::ios::app);
	ofsGlovalVD = std::ofstream(fGlovalVDPath, std::ios::app);
	ofsLocalVD = std::ofstream(fLocalVDPath, std::ios::app);
	if (isFirstSimulation) {
		ofsFD << "k,Flux" << std::endl;
		ofsGlovalVD << "N,rho,V" << std::endl;
		ofsLocalVD << "k,V" << std::endl;
	}
}

void Simulation::WriteResultToCSV(const std::stringstream& sResultFD, const std::stringstream& sResultGlovalVD, const std::stringstream& sResultLocalVD) {
	ofsFD << sResultFD.str();
	ofsGlovalVD << sResultGlovalVD.str();
	ofsLocalVD << sResultLocalVD.str();
}
