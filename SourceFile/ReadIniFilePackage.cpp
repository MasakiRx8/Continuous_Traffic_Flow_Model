/*
	This is cpp file of the class of "ReadIniFilePackage" that a class for reading ".ini" files.
*/

#include "ReadIniFilePackage.h"

char ReadIniFilePackage::ToUpper::operator()(const char& c) {
	return std::toupper(c);
}


char ReadIniFilePackage::ToLower::operator()(const char& c) {
	return std::tolower(c);
}

bool ReadIniFilePackage::IsSpace::operator()(const char& c) {
	return (c == ' ');
}

ReadIniFilePackage::SectionVariable::SectionVariable(const std::string& section, const std::string& variable) {
	this->section = section;
	this->variable = variable;
	std::transform(this->section.begin(), this->section.end(), this->section.begin(), ToUpper());
	std::transform(this->variable.begin(), this->variable.end(), this->variable.begin(), ToUpper());
}

bool ReadIniFilePackage::SectionVariable::operator<(const SectionVariable& val) const {
	if (this->section < val.section) {
		return true;
	}
	else {
		if (this->section > val.section) {
			return false;
		}
		else {
			if (this->variable < val.variable) {
				return true;
			}
			else {
				return false;
			}
		}
	}
}

ReadIniFilePackage::ReadIniFilePackage(const std::string& FileName) {
	datas = new std::vector<std::string*>;
	locations = new std::map<SectionVariable, int>;
	ReadAllData(FileName);
}

ReadIniFilePackage::~ReadIniFilePackage() {
	for (std::size_t i = 0; i < datas->size(); i++) {
		delete (*datas)[i];
	}
	delete datas;
	delete locations;
}

double ReadIniFilePackage::ReadIni(const std::string& SectionName, const std::string& VariableName) {
	double val;
	GetData(SectionName, VariableName, val);
	return val;
}

void ReadIniFilePackage::ReadIni(const std::string& SectionName, const std::string& VariableName, int& val) {
	GetData(SectionName, VariableName, val);
}

void ReadIniFilePackage::ReadIni(const std::string& SectionName, const std::string& VariableName, double& val) {
	GetData(SectionName, VariableName, val);
}

void ReadIniFilePackage::ReadIni(const std::string& SectionName, const std::string& VariableName, std::string& val) {
	ReadIni(SectionName, VariableName, val, TransformModeType::Original);
}

void ReadIniFilePackage::ReadIni(const std::string& SectionName, const std::string& VariableName, std::string& val, const TransformModeType& transformMode) {
	GetData(SectionName, VariableName, val);
	switch (transformMode) {
	case TransformModeType::Upper:
		std::transform(val.begin(), val.end(), val.begin(), ToUpper());
		break;
	case TransformModeType::Lower:
		std::transform(val.begin(), val.end(), val.begin(), ToLower());
		break;
	default:
		break;
	}
}

void ReadIniFilePackage::ReadAllData(const std::string& FileName) {
	std::ifstream ifs(FileName);
	if (!ifs) {
		throw std::invalid_argument("Not File Existants:" + FileName);
		return;
	}
	std::string section;
	std::string variable;
	std::string S;
	int i = 0;
	std::size_t ifind;
	while (std::getline(ifs, S)) {
		if (S != "") {
			if (S[0] == '[' && S[S.size() - 1] == ']') {
				section = S.substr(1, S.size() - 2);
			}
			else {
				ifind = S.find_first_of('=');
				if (ifind != std::string::npos) {
					variable = S.substr(0, ifind);
					S = S.substr(ifind + 1);
					S.erase(std::remove_if(S.begin(), S.end(), IsSpace()), S.end());
					S = S.substr(0, S.find_first_of('#'));
					if (S.size() > 0) {
						datas->emplace_back(new std::string(S));
						(*locations)[SectionVariable(section, variable)] = i;
						i++;
					}
				}
			}
		}
	}
	ifs.close();
}

template<typename _T>
void ReadIniFilePackage::GetData(const std::string& SectionName, const std::string& VariableName, _T& val) const {
	const SectionVariable SV(SectionName, VariableName);
	std::map<SectionVariable, int>::iterator i = locations->find(SV);
	if (i == locations->end()) {
		throw std::invalid_argument("Not Found SectionName:" + SectionName + " VariableName:" + VariableName);
	}
	else {
		std::stringstream SS;
		SS << *(*datas)[(*locations)[SV]];
		SS >> val;
	}
}