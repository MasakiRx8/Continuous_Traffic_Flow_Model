/*
	This is header file of the class of "StatisticsElementsArray" that is an array of class "StatisticsElementsClass".
	This class's private parameters are opened to class "StatisticsClass".
*/

#ifndef STATISTICSELEMENTSARRAY_H
#define STATISTICSELEMENTSARRAY_H
#include <vector>
#include "Common.h"
#include "StatisticsElementsClass.h"

class StatisticsElementsArray {
public:
	StatisticsElementsArray(const std::size_t& N);	//constructor
	~StatisticsElementsArray();	//destructor
	std::size_t size() const;
	StatisticsElementsClass* const operator[](std::size_t index) const;
private:
	const std::size_t N;
	std::vector<StatisticsElementsClass*>* vStattisticsElements;

	friend class StatisticsClass;	//open private parameters to class "StatisticsClass"
};

#endif // !STATISTICSELEMENTSARRAY_H
