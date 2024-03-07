#include "VectorSort.h"

VectorSort::VectorSort() {
	indexOrder = nullptr;
}

VectorSort::~VectorSort() {
	SafeDeleteIndexOrder();
}

std::size_t  VectorSort::size() const {
	if (indexOrder != nullptr) {
		return indexOrder->size();
	}
	else {
		return 0;
	}
}

std::size_t VectorSort::operator[](const std::size_t& index) const {
	return (*indexOrder)[index];
}

void VectorSort::AscendingSort(std::vector<int>& v) { 
	SortInAscendingOrder(v); 
}
void VectorSort::AscendingSort(std::vector<unsigned int>& v) { 
	SortInAscendingOrder(v); 
}
void VectorSort::AscendingSort(std::vector<long>& v) { 
	SortInAscendingOrder(v); 
}
void VectorSort::AscendingSort(std::vector<unsigned long>& v) { 
	SortInAscendingOrder(v); 
}
void VectorSort::AscendingSort(std::vector<long long>& v) { 
	SortInAscendingOrder(v); 
}
void VectorSort::AscendingSort(std::vector<unsigned long long>& v) { 
	SortInAscendingOrder(v); 
}
void VectorSort::AscendingSort(std::vector<float>& v) { 
	SortInAscendingOrder(v); 
}
void VectorSort::AscendingSort(std::vector<double>& v) {
	SortInAscendingOrder(v); 
}
void VectorSort::AscendingSort(std::vector<char>& v) { 
	SortInAscendingOrder(v); 
}

void VectorSort::DescendingSort(std::vector<int>& v) { 
	SortInDescendingOrder(v); 
}
void VectorSort::DescendingSort(std::vector<unsigned int>& v) { 
	SortInDescendingOrder(v); 
}
void VectorSort::DescendingSort(std::vector<long>& v) { 
	SortInDescendingOrder(v); 
}
void VectorSort::DescendingSort(std::vector<unsigned long>& v) { 
	SortInDescendingOrder(v); 
}
void VectorSort::DescendingSort(std::vector<long long>& v) { 
	SortInDescendingOrder(v); 
}
void VectorSort::DescendingSort(std::vector<unsigned long long>& v) { 
	SortInDescendingOrder(v); 
}
void VectorSort::DescendingSort(std::vector<float>& v) { 
	SortInDescendingOrder(v); 
}
void VectorSort::DescendingSort(std::vector<double>& v) { 
	SortInDescendingOrder(v); 
}
void VectorSort::DescendingSort(std::vector<char>& v) { 
	SortInDescendingOrder(v); 
}

template<typename _T>
void VectorSort::SortInAscendingOrder(std::vector<_T>& v) {
	const std::size_t left = 0;
	const std::size_t right = v.size() - 1;
	CreateNumber(v.size());
	RecursiveAscendingSortFunction(&v, left, right);
}

template<typename _T>
void VectorSort::SortInDescendingOrder(std::vector<_T>& v) {
	const std::size_t left = 0;
	const std::size_t right = v.size() - 1;
	CreateNumber(v.size());
	RecursiveDescendingSortFunction(&v, left, right);
}

template<typename _T>
_T VectorSort::GetMiddleValue(const _T& x, const _T& y, const _T& z) const { 
	return (std::min)((std::max)(x, y), z); 
}

void VectorSort::CreateNumber(const std::size_t& size) {
	SafeDeleteIndexOrder();
	indexOrder = new std::vector<std::size_t>(size);
	for (std::size_t i = 0; i < size; i++) {
		(*indexOrder)[i] = i;
	}
}

template<typename _T>
void VectorSort::RecursiveAscendingSortFunction(std::vector<_T>* const v, const std::size_t& left, const std::size_t& right) const {
	if (left < right) {
		const _T pivot = GetMiddleValue((*v)[left], (*v)[left + (right - left) / 2], (*v)[right]);
		std::size_t l = left;
		std::size_t r = right;
		while (true) {
			while ((*v)[l] < pivot) {
				l++;
			}
			while ((*v)[r] > pivot) {
				r--;
			}
			if (l >= r) {
				break;
			}
			std::swap((*v)[l], (*v)[r]);
			std::swap((*indexOrder)[l], (*indexOrder)[r]);
			l++;
			r--;
		}
		if (l > 0) {
			RecursiveAscendingSortFunction(v, left, l - 1);
		}
		RecursiveAscendingSortFunction(v, r + 1, right);
	}
}

template<typename _T>
void VectorSort::RecursiveDescendingSortFunction(std::vector<_T>* const v, const std::size_t& left, const std::size_t& right) const {
	if (left < right) {
		const _T pivot = GetMiddleValue((*v)[left], (*v)[left + (right - left) / 2], (*v)[right]);
		std::size_t l = left;
		std::size_t r = right;
		while (true) {
			while ((*v)[l] > pivot) {
				l++;
			}
			while ((*v)[r] < pivot) {
				r--;
			}
			if (l >= r) {
				break;
			}
			std::swap((*v)[l], (*v)[r]);
			std::swap((*indexOrder)[l], (*indexOrder)[r]);
			l++;
			r--;
		}
		if (l > 0) {
			RecursiveDescendingSortFunction(v, left, l - 1);
		}
		RecursiveDescendingSortFunction(v, r + 1, right);
	}
}

void VectorSort::SafeDeleteIndexOrder() {
	if (indexOrder != nullptr) {
		delete indexOrder;
		indexOrder = nullptr;
	}
}
