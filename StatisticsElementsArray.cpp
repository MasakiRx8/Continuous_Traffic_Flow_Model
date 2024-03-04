/*
	This is cpp file of the class of "StatisticsElementsArray" that is an array of class "StatisticsElementsClass".
	This class's private parameters are opened to class "StatisticsClass".
*/

#include "StatisticsElementsArray.h"

//constructor
StatisticsElementsArray::StatisticsElementsArray(const std::size_t& N) : N(N) {
	vStattisticsElements = new std::vector<StatisticsElementsClass*>(N);
	for (std::size_t i = 0; i < N; i++) {
		(*vStattisticsElements)[i] = new StatisticsElementsClass;
	}
}

//destructor
StatisticsElementsArray::~StatisticsElementsArray() {
	for (std::size_t i = 0; i < vStattisticsElements->size(); i++) {
		SafeDelete((*vStattisticsElements)[i]);	//delete StatisticsElementsClass
	}
	SafeDelete(vStattisticsElements);	//delete vector
}

std::size_t StatisticsElementsArray::size() const {
	return vStattisticsElements->size();
}

StatisticsElementsClass* const StatisticsElementsArray::operator[](std::size_t index) const {
	return (*vStattisticsElements)[index];
}
