#ifndef COMMON_H
#define COMMON_H

const double kappa = 0.1;

double Calculate_Km_h_To_m_s(const double& v);
double Calculate_m_s_To_Km_h(const double& v);

template <class _T>
void SafeDelete(_T*& p) {
	if (p != nullptr) {
		delete p;
		p = nullptr;
	}
}

enum class DriverModeType {
	Human
	, Auto
};

enum class InitialPositionModeType {
	Equal
	, Random
};

enum class FootPositionType {
	Free
	, Accel
	, Brake
};

enum class PedalChangedStateType {
	NoChanged
	, Changing
	, Changed
	, ImmediatelyChanged
};

namespace Common {
	namespace EigenValuesElements {
		struct PlusMinus {
		public:
			double Plus;
			double Minus;
			PlusMinus();
		};

		struct UpperLower {
		public:
			double Upper;
			double Lower;
			UpperLower();
		};

		struct GSerise {
		public:
			double Closest;
			double Cruise;
			double Influenced;
			GSerise();
		};
	}

	namespace MomentValuesElements {
		struct PlusMinus {
		public:
			double plus;
			double minus;
			PlusMinus();
		};

		struct CurrentLast {
		public:
			double current;
			double last;
			CurrentLast();
			void CopyCurrentToLast();
		};

		struct GSerise {
		public:
			double closest;
			double cruise;
			double influenced;
			GSerise();
		};

		struct VelocityGap {
		public:
			double velocity;
			double gap;
			VelocityGap();
		};
	}
}

#endif // !COMMON_H
