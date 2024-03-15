#ifndef DRIVERSTRUCT_H
#define DRIVERSTRUCT_H
#include <cmath>
#include "Common.h"

namespace DriverElements {
	namespace EigenValuesElements {
		struct AccelerationSeries {
		public:
			double Acceptable;
			double Strong;
			double Normal;
			AccelerationSeries();
		};

		struct AccelerationPackage {
		public:
			AccelerationSeries* const Acceleration;
			AccelerationSeries* const Deceleration;
			AccelerationSeries* const FrontDeceleration;
			AccelerationPackage();
			~AccelerationPackage();
		private:
			bool deletedAcceleration;
			bool deletedDeceleration;
			bool deletedFrontDeceleration;
		};

		struct PedalChanging {
		public:
			Common::EigenValuesElements::UpperLower* const AccelToBrake;
			Common::EigenValuesElements::UpperLower* const BrakeToAccel;
			PedalChanging();
			~PedalChanging();
		private:
			bool deletedAccelToBrake;
			bool deletedBrakeToAccel;
		};

		struct PedalChangingTimeInformations {
		public:
			PedalChanging* const T;
			PedalChanging* const V;
			PedalChangingTimeInformations();
			~PedalChangingTimeInformations();
		private:
			bool deletedT;
			bool deletedV;
		};

		struct VSerise {
		public:
			double Cruise;
			Common::EigenValuesElements::PlusMinus* const DeltaAtCruise;
			Common::EigenValuesElements::PlusMinus* const DeltaAt0;
			VSerise();
			~VSerise();
		private:
			bool deletedDeltaAtCruise;
			bool deletedDeltaAt0;
		};

		struct TMargin {
		public:
			Common::EigenValuesElements::UpperLower* const V;
			Common::EigenValuesElements::UpperLower* const T;
			TMargin();
			~TMargin();
		private:
			bool deletedV;
			bool deletedT;
		};
	}

	namespace MomentValuesElements {
		struct NeedChangingTime {
		public:
			double accelToBrake;
			double brakeToAccel;
			NeedChangingTime();
		};

		struct PedalInformations {
		public:
			double needTime;
			double timeElapsed;
			bool changing;
			FootPositionType footPosition;
			FootPositionType targetFootPosition;
			NeedChangingTime* const t;
			PedalInformations();
			~PedalInformations();
		private:
			bool deletedT;
		};

		struct VSerise {
		public:
			double target;
			Common::MomentValuesElements::PlusMinus* const delta;
			Common::MomentValuesElements::CurrentLast* const deltaV;
			VSerise();
			~VSerise();
		private:
			bool deletedDelta;
			bool deletedDeltaV;
		};

		struct GSerise {
		public:
			double baseFg;
			double baseNg;
			bool emergency;
			GSerise();
			void SetBaseNg(const double& randomValue);
		};
	}

	//Eigenvalues that characterize drivers.
	struct EigenValues {
	public:
		EigenValuesElements::AccelerationPackage* const A;
		EigenValuesElements::PedalChangingTimeInformations* const PedalChange;
		EigenValuesElements::TMargin* const TMargin;
		EigenValuesElements::VSerise* const V;
		Common::EigenValuesElements::GSerise* const G;
		EigenValues();
		~EigenValues();
	private:
		bool deletedA;
		bool deletedPedalChange;
		bool deletedTMargin;
		bool deletedV;
		bool deletedG;
	};

	//The value that the driver has while driving, changing from moment to moment.
	struct MomentValues {
	public:
		double a;
		bool recognitionHit;
		Common::MomentValuesElements::VelocityGap* const R;
		MomentValuesElements::PedalInformations* const pedal;
		MomentValuesElements::VSerise* const v;
		MomentValuesElements::GSerise* const g;
		MomentValues();
		~MomentValues();
	private:
		bool deletedR;
		bool deletedPedal;
		bool deletedV;
		bool deletedG;
	};
}

struct DriverStruct {
public:
	DriverElements::EigenValues* const Eigen;
	DriverElements::MomentValues* const Moment;
	DriverStruct();
	~DriverStruct();
private:
	bool deletedEigen;
	bool deletedMoment;
};

#endif // !DRIVERSTRUCT_H
