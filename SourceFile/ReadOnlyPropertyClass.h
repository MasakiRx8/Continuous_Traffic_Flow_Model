/*
	This class is a class for reproducing read-only properties in C# and VB.Net.
*/

#ifndef READONLYPROPERTYCLASS_H
#define READONLYPROPERTYCLASS_H
#include <functional>

/*
	reference 
	Document Number: SC22/WG21/N1615=04-0055
	Author: Lois Goldthwaite
	C++ Properties -- a Library Solution
	http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2004/n1615.pdf
*/
template<class _T>
class ReadOnlyPropertyClass {
	std::function<_T()> getFunction;
public:
	void operator()(std::function<_T()> getFunction) {
		this->getFunction = getFunction;
	}

	_T operator()() const {
		return getFunction();
	}

	_T get() const {
		return getFunction();
	}

	operator _T() const {
		return getFunction();
	}

	typedef _T value_type;
};

#endif // !READONLYPROPERTYCLASS_H
