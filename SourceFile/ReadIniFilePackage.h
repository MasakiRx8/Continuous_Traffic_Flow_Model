/*
	This is header file of the class of "ReadIniFilePackage" that a class for reading ".ini" files.
*/

#ifndef READINIFILEPACKAGE_H
#define READINIFILEPACKAGE_H
#include <algorithm>
#include <cctype>
#include <exception>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

class ReadIniFilePackage {
public:
	enum class TransformModeType {
		Original
		, Upper
		, Lower
	};

	ReadIniFilePackage(const std::string& FileName);	//constructor
	~ReadIniFilePackage();	//destructor
	double ReadIni(const std::string& SectionName, const std::string& VariableName);
	void ReadIni(const std::string& SectionName, const std::string& VariableName, int& val);
	void ReadIni(const std::string& SectionName, const std::string& VariableName, double& val);
	void ReadIni(const std::string& SectionName, const std::string& VariableName, std::string& val);
	void ReadIni(const std::string& SectionName, const std::string& VariableName, std::string& val, const TransformModeType& transformMode);
private:
	struct ToUpper {
		char operator()(const char& c);
	};
	struct ToLower {
		char operator()(const char& c);
	};
	struct IsSpace	{
		bool operator()(const char& c);
	};
	class SectionVariable {
	public:
		std::string section;
		std::string variable;
		SectionVariable(const std::string& section, const std::string& variable);
		bool operator<(const SectionVariable& val) const;
	};
	std::vector<std::string*>* datas;
	std::map<SectionVariable, int>* locations;
	void ReadAllData(const std::string& FileName);

	template<typename _T>
	void GetData(const std::string& SectionName, const std::string& VariableName, _T& val) const;
};

#endif // !READINIFILEPACKAGE_H
