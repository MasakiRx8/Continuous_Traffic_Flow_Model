/*
	This is header file of the class of "StatisticsElementsClass" that stores statistical information.
	This class's private parameters are opened to class "StatisticsClass".
*/

#ifndef STATISTICSELEMENTSCLASS_H
#define STATISTICSELEMENTSCLASS_H
#include "ReadOnlyPropertyClass.h"

class StatisticsElementsClass {
public:
	StatisticsElementsClass();	//constructor
private:
	int _counter;
	double _k;
	double _averageVelocity;

	void InitializeProperties(StatisticsElementsClass* const thisPtr);

	int Get_Counter() const;
	double Get_K() const;
	double Get_AverageVelocity() const;
public:
	ReadOnlyPropertyClass<int> Counter;
	ReadOnlyPropertyClass<double> K;
	ReadOnlyPropertyClass<double> AverageVelocity;

	friend class StatisticsClass;	//open private parameters to class "StatisticsClass"
};

#endif // !STATISTICSELEMENTSCLASS_H
