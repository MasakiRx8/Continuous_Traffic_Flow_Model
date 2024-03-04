#ifndef CARSTRUCT_H
#define CARSTRUCT_H
#include "Common.h"
#include "DriverStruct.h"

struct CarStruct;

namespace CarElements {
	namespace MomentValuesElements {
		struct GapSerise : Common::MomentValuesElements::GSerise {
		public:
			double gap;
			Common::MomentValuesElements::CurrentLast* const deltaGap;
			GapSerise();
			~GapSerise();
		private:
			bool deletedDeltaGap;
		};

		struct Measurement {
		public:
			bool passed;
			double elapsedTime;
			Measurement();
			void Reset();
		};

		struct ArroundCarInformations {
		public:
			const int ID;
			const double& a;
			const double& v;
			const double& x;
			const double& Length;
			ArroundCarInformations(const int& ID, const double& a, const double& v, const double& x, const double& Length);
		};

		struct Arround {
		public:
			ArroundCarInformations* const rear;
			ArroundCarInformations* const front;
			Arround(const CarStruct* const rear, const CarStruct* const front);
			~Arround();
		private:
			bool deletedRear;
			bool deletedFront;
		};
	}

	//Vehicle characteristic values.
	struct EigenValues {
	public:
		double Vmax;
		Common::EigenValuesElements::PlusMinus* const Amax;
		double AResistance;
		double Length;
		EigenValues();
		~EigenValues();
	private:
		bool deletedAmax;
	};

	//The value that the vehicle has while changing from moment to moment.
	struct MomentValues {
	public:
		double a;
		double x;
		double v;
		MomentValuesElements::GapSerise* const g;
		MomentValuesElements::Measurement* const measurement;
		MomentValuesElements::Arround* arround;
		MomentValues();
		~MomentValues();
	private:
		bool deletedG;
		bool deletedMeasurement;
		bool deletedArround;
	};
}

struct CarStruct {
public:
	int ID;
	CarElements::EigenValues* const Eigen;
	CarElements::MomentValues* const Moment;
	DriverStruct* const Driver;
	CarStruct();
	~CarStruct();
private:
	bool deletedEigen;
	bool deletedMoment;
	bool deletedDriver;
};

#endif // !CARSTRUCT_H
