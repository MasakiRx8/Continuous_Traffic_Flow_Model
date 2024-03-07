#include "random.h"

Random::Random() {
	Initialize_mt19937(0);
}

Random::Random(const int& seedAuxiliaryValue) {
	Initialize_mt19937(seedAuxiliaryValue);
}

Random::~Random() {
	if (mt != nullptr) {
		delete mt;
		mt = nullptr;
	}
}

int Random::operator()(const int& N) const {
	return create_int_rand(0, N);
}

int Random::operator()(const int& Nmin, const int& Nmax) const {
	return create_int_rand(Nmin, Nmax);
}

double Random::operator()(const double& D) const {
	return create_double_rand(0.0, D);
}

double Random::operator()(const double& Dmin, const double& Dmax) const {
	return create_double_rand(Dmin, Dmax);
}

void Random::Initialize_mt19937(const int& seedAuxiliaryValue) {
	mt = new std::mt19937((unsigned int)(seedAuxiliaryValue * 1000 + time(nullptr)));
}

int Random::create_int_rand(const int& xmin, const int& xmax) const {
	std::uniform_int_distribution<> rd(xmin, xmax);
	return rd(*mt);
}

double Random::create_double_rand(const double& xmin, const double& xmax) const {
	std::uniform_real_distribution<> rd(xmin, xmax);
	return rd(*mt);
}