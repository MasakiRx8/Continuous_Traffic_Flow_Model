#ifndef SORT_H
#define SORT_H
#include <algorithm>
#include <vector>

class VectorSort {
public:
	VectorSort();
	~VectorSort();
	std::size_t size() const;
	std::size_t operator[](const std::size_t& index) const;

	void AscendingSort(std::vector<int>& v);
	void AscendingSort(std::vector<unsigned int>& v);
	void AscendingSort(std::vector<long>& v);
	void AscendingSort(std::vector<unsigned long>& v);
	void AscendingSort(std::vector<long long>& v);
	void AscendingSort(std::vector<unsigned long long>& v);
	void AscendingSort(std::vector<float>& v);
	void AscendingSort(std::vector<double>& v);
	void AscendingSort(std::vector<char>& v);

	void DescendingSort(std::vector<int>& v);
	void DescendingSort(std::vector<unsigned int>& v);
	void DescendingSort(std::vector<long>& v);
	void DescendingSort(std::vector<unsigned long>& v);
	void DescendingSort(std::vector<long long>& v);
	void DescendingSort(std::vector<unsigned long long>& v);
	void DescendingSort(std::vector<float>& v);
	void DescendingSort(std::vector<double>& v);
	void DescendingSort(std::vector<char>& v);

private:
	std::vector<std::size_t>* indexOrder;

	template<typename _T>
	void SortInAscendingOrder(std::vector<_T>& v);

	template<typename _T>
	void SortInDescendingOrder(std::vector<_T>& v);

	template<typename _T>
	_T GetMiddleValue(const _T& x, const _T& y, const _T& z) const;

	void CreateNumber(const std::size_t& size);

	template<typename _T>
	void RecursiveAscendingSortFunction(std::vector<_T>* const v, const std::size_t& left, const std::size_t& right) const;

	template<typename _T>
	void RecursiveDescendingSortFunction(std::vector<_T>* const v, const std::size_t& left, const std::size_t& right) const;

	void SafeDeleteIndexOrder();
};

#endif // !SORT_H
