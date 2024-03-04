#include "Common.h"

double Calculate_Km_h_To_m_s(const double& v) {
	return v * 5 / 18;
}

double Calculate_m_s_To_Km_h(const double& v) {
	return v * 18 / 5;
}

Common::EigenValuesElements::PlusMinus::PlusMinus() {
	Plus = 0;
	Minus = 0;
}

Common::EigenValuesElements::UpperLower::UpperLower() {
	Upper = 0;
	Lower = 0;
}

Common::EigenValuesElements::GSerise::GSerise() {
	Closest = 0;
	Cruise = 0;
	Influenced = 0;
}

Common::MomentValuesElements::PlusMinus::PlusMinus() {
	plus = 0;
	minus = 0;
}

Common::MomentValuesElements::CurrentLast::CurrentLast() {
	current = 0;
	last = 0;
}

void Common::MomentValuesElements::CurrentLast::CopyCurrentToLast() {
	last = current;
}

Common::MomentValuesElements::GSerise::GSerise() {
	closest = 0;
	cruise = 0;
	influenced = 0;
}

Common::MomentValuesElements::VelocityGap::VelocityGap() {
	velocity = 0;
	gap = 0;
}
