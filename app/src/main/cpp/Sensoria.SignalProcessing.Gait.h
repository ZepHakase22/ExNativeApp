//
//  Sensoria.SignalProcessing.Gait.h
//  Sensoria
//
//  Created by Maurizio Macagno / Stefano Rossotti on 03/23/18.
//  Copyright (c) 2018 Sensoria Inc. All rights reserved.
//


#include <iostream>
//#include <cassert>
#include <cmath>
#include <algorithm>
#include <stdint.h>

#include "Sensoria.SignalProcessing.Gait.Constants.h"
#include "Sensoria.SignalProcessing.GaitEvents.h"
#include "../Common/DspLib/CircularBuffer.h"
#include "../Common/DspLib/DSPClass.h"
#include "../Common/DspLib/Sensoria.SignalProcessingLibrary.h"
#include "../Common/Physics/Sensoria.SignalProcessing.Physics.Quaternions.h"
#include "../Common/Physics/Sensoria.SignalProcessing.Physics.FVector.h"
#include "../Common/Physics/Sensoria.SignalProcessing.Physics.Random.h"
#include "../Common/Sensoria.SignalProcessing.Common.h"

using namespace Sensoria::SignalProcessing::Gait::Constants;
using namespace Sensoria::SignalProcessing::Physics::FVector;
using namespace Sensoria::SignalProcessing::Physics::RandomGenerator;
using namespace Sensoria::SignalProcessing::Physics::Quaternions;
using namespace Sensoria::SignalProcessing::Common;

#ifndef __Sensoria__SignalProcessing__Gait__
#define __Sensoria__SignalProcessing__Gait__

namespace Sensoria {
	namespace SignalProcessing {
		namespace Gait {
			
			class SignalProcessing {
			public:
				// Constructor/Destructor
				SignalProcessing() : SignalProcessing(K_DefaultSamplingFrequency, K_DefaultGarmentType, K_DefaultPressureSensorType, K_DefaultDevicePosition, NULL) {};
				SignalProcessing(double SamplingFrequency, GarmentType GarmentType, PressureSensorType PressureSensorType, DevicePosition DevicePosition, GaitEvents *objGaitEvents);
				~SignalProcessing();

				// Reset SP variables
				void Reset();

				// Public functions
				void ProcessIncomingData(const double *RAWIncomingPre, const double *RAWIncomingAcc, const double *RAWIncomingMag, const double *RAWIncomingGyr);

			protected:
				// Constructor variables
				double _SamplingFrequency;
				GarmentType _GarmentType;
				PressureSensorType _PressureSensorType;
				DevicePosition _DevicePosition;
				GaitEvents *_objGaitEvents;

				// Sensors being processed
				SensorType ComputingSensorTypeID;
				int ComputingSensorID;

				// Frames counter 
				int FramesProcessedCounter;
					
				// Biases on raw data
				quaternion qMagAdjustment;
				fvector3d  BiasMag;		// Bias on the raw mag measures
				fvector3d  BiasGyr;		// Bias on the raw gyr measures
				fvector3d  ScaleMag;	// Scale factor correction on the raw mag measures after bias removal

				// Number of sensors for each senror type
				int nSensors[nUsingSensorsTypes] = {nSensorsPre, nSensorsAcc, nSensorsMag, nSensorsGyr};
				int TotalNumberOfSensors = nSensorsPre + nSensorsAcc + nSensorsMag + nSensorsGyr;

				// Outliers
				RandomGenerator OutlierSuppressor;
				
				// Filters
				int const FilterOrder[nUsingSensorsTypes] = { K_DefaultFilterOrderPre, K_DefaultFilterOrderAcc, K_DefaultFilterOrderMag, K_DefaultFilterOrderGyr };
				int const FilterFrequencyLP[nUsingSensorsTypes] = { K_DefaultFilterFrequencyLPPre, K_DefaultFilterFrequencyLPAcc, K_DefaultFilterFrequencyLPMag, K_DefaultFilterFrequencyLPGyr };
				std::vector<SensoriaLibrary::DSP::Filter> LP;
				std::vector<std::vector<std::vector<double>>> LPHistory;

				// RAW Incoming Data
				fvector3d IncomingAcc;
				fvector3d IncomingMag;
				fvector3d IncomingGyr;

				// Signal processed from RAW data
				fvector<double, nSensorsPre> ProcessedSignalPre;
				fvector3d ProcessedSignalAcc;
				fvector3d ProcessedSignalMag;
				fvector3d ProcessedSignalGyr;

				// Reference Variables 
				bool DoSetReferenceVariables;

				// DynamicSignal variables
				double RatioAvgRange;
				double RatioOutlier;
				double RatioRange;
				bool IsDynamicalRange;
				bool IsDynamicalOutlier;
				bool IsDynamicalAvgRange;
				bool **IsDynamicSignal;

				// Mean crossing and Step detection
				bool MeanCrossTooFast;
				bool StepTimeInRange;
				int CrossingDirection;
				int **CurrentStepCrossUpFrame;
				int **PreviousStepCrossUpFrame;
				int **PreviousStepCrossDownFrame;
				int **PreviousCrossMeanFlag;
				int **CurrentCrossMeanFlag;
				bool **FirstTimeCrossedUp;
				double **SensorBufferMA_PeakThreshold;

				int *FramesFromLastDetection;

				int *MeanCrossNFramesThreshold;	// number of frames that have to pass in order to cross the mean again
				int *MetricsResetFrames;		// number of frames that have to pass in order to reset the metrics
				double *MaxStepSecs;			// Max seconds to perform a step
				double *MinStepSecs;			// Min seconds to perform a step

				// Sensors Sync
				SyncMask SensorsToSyncMask[2] = { SensorsToSyncMaskDefaultPre, SensorsToSyncMaskDefaultAcc };
				SyncMask SensorsInSyncMask[2] = { SensorsToSyncMaskDefaultPre, SensorsToSyncMaskDefaultAcc };

				int  *FramesDelayForSyncAll;
				int  MaxFramesDelayForSyncTwo;

				// Buffers for SensorData
				std::vector<std::vector<CircularBuffer<double>>> RAWSignalBuffer;
				std::vector<std::vector<CircularBuffer<double>>> ProcessedSignalBuffer;
				std::vector<std::vector<CircularBuffer<double>>> NoOutliersSignalBuffer;
				double CurrentDataPoint;

				// Buffer sizes
				int MillisecsInBuffer[nUsingSensorsTypes] = { K_DefaultMillisecsInBufferPre, K_DefaultMillisecsInBufferAcc, K_DefaultMillisecsInBufferMag, K_DefaultMillisecsInBufferGyr };
				int BufferLength[nUsingSensorsTypes];
				int MinBufferLength;
				int MaxBufferLength;

				// Statistical variables
				double **ProcessedSignalRange;		// Moving Average signal per sensor -> Range of MA
				double **ProcessedSignalMvAvg;		// Moving Average signal per sensor -> Mean
				double **ProcessedSignalStdDv;		// Moving Average signal per sensor -> Standard Deviation
				double **SensorBufferMA_Min;		// Minimum of the signal in a single step
				double **SensorBufferMA_Max;		// Maximum of the signal in a single step
				double *MinStdDv;

				// Cadence
				int *CadenceStartFrame;
				int *CadenceEndFrame;
				double *AverageCadenceTwoFeet;
				double *LocalCadenceTwoFeet;
				double *LocalCadenceOneFoot;
				double *MinCadence;
				double *MaxCadence;
				std::vector<CircularBuffer<double>> LocalCadenceHistory;

				// ToG variables
				double TimeOnGround[nStepsSensorsTypes];
				std::vector<CircularBuffer<double>> TimeOnGroundHistory;

				// Activity Type
				ActivityType DetectedActivity[nStepsSensorsTypes];
				double PressureSensorStandingThreshold[nSensorsPre] = {K_StandingThresholdS0 };
				
				// Steps
				int *StepCount;

				// Device Worn
				int nFramesToCheckWorn;
				int nFramesToConsiderNotWorn;
				bool IsAccDynamical;
				bool IsPreAboveThreshold;
				bool WornFlag;
				bool IsDeviceWorn;
				int nFramesNotWorn;

				// Orientation of the IMU
				quaternion qOrientation;
				quaternion qOldOrientation[2];
				quaternion qOrientationMag;
				quaternion qOrientationAcc;
				quaternion qOrientationMagAcc;
				quaternion qOrientationAccMag;
				quaternion qOrientationAll;
				quaternion qReferenceOrientation;
				quaternion qStepReferenceOrientation[2];
				quaternion qOmega;
				quaternion qOrientationVerlet;
				quaternion qDeltaOrientationMag;
				quaternion qCoreRotation;					// Core rotation applied to Accelerometer, Magnetometer and Gyroscope due to different position of the IMU

				fvector3d   OldMag;
				fvector3d	ReferenceGravity;
				fvector3d	ReferenceMag;
				fvector3d	PredictedGravityOrientation;
				double		GravityAngularDeviation;
				double		GravityNorm;

				// Gait length
				fvector3d GaitRotatedAcc;
				double **FootPosition;
				double *GaitLength;

				// Global variables
				SensorType SensorTypeToUse;				// Sensor type to use to set metrics
				int GlobalStepCount;
				double GlobalCadence;
				double GlobalTimeOnGround;
				double GlobalGaitLength;
				ActivityType GlobalDetectedActivity;

				// Previous Global variables
				int prevGlobalStepCount;
				double prevGlobalCadence;
				double prevGlobalTimeOnGround;
				ActivityType prevGlobalDetectedActivity;

				// Debug variables and functions
				SignalFeature **DBG_StepFlag;
				double  DBG_DynamicPre = 0;
				double *DBG_DynamicAcc;
				fvector3d x;
				fvector3d y;
				fvector3d z;

			private:
				// Signal Processing
				void ProcessIncomingPre(const double *IncomingPre);
				void ProcessIncomingAcc(const fvector3d &IncomingAcc);
				void ProcessIncomingMag(const fvector3d &IncomingMag);
				void ProcessIncomingGyr(const fvector3d &IncomingGyr);

				// Step computation
				void CycleOverSensorsPre();
				void CycleOverSensorsAcc();
				void CycleOverSensorsMag();
				void CycleOverSensorsGyr();

				// Step detection
				void DetectStepPre();
				void DetectStepAcc();

				// Update steps
				void UpdateStepPre();
				void UpdateStepAcc();

				//Reference settimg
				void SetReferenceVariables();

				// Reset metrics
				void ResetMetrics();

				// Calculate statistical variables
				void CalculateStatisticalVariables();

				// Dynamical signal evaluation
				bool CheckIsDynamicalPre();
				bool CheckIsDynamicalAcc();

				// Outliers
				void   DetectOutliersInSignal(double &NewDataPoint, CircularBuffer<double> &buffer, const SensorType &SensorTypeID);
				double DetectOutliersInCadence(const double &StartFrame, const  double &EndFrame, const  double &minCadence, const  double &maxCadence);

				// Sync
				void EvaluateSensorsInSync();
				int AverageInSyncTime ();
				int CountSensorsInSync();
				void ToggleSensor(SyncMask &Mask, const int &SensorID);
				bool CheckSensor (SyncMask &Mask, const int &SensorID);

				// Gait length
				void IntegrateGaitLength();

				// Activity Type
				bool IsStanding;
				void EvaluateRestActivity();

				// Is Worn
				void CheckIsDeviceWorn();

				// Orientation
				void CalculateOrientation();

				// Global Metrics
				void SetGlobalMetrics ();
				void SetGlobalActivity();

				// Switch SensorTypeToUse
				void SwitchSensorTypeToUse(const SensorType &NewSensorTypeID);
				
				// Debug
				void ResetFlags();
			};


			class SignalProcessingDebug : public SignalProcessing 	{
			public:
				SignalProcessingDebug() : SignalProcessing() {}
				SignalProcessingDebug(double SamplingFrequency, GarmentType GarmentType, PressureSensorType PressureSensorType, DevicePosition DevicePosition, GaitEvents *objGaitEvents) : SignalProcessing(SamplingFrequency, GarmentType, PressureSensorType, DevicePosition, objGaitEvents) {}

				// Return constructor variables
				double DBG_GetSamplingFrequency() { return _SamplingFrequency; }
				GarmentType DBG_GetGarmentType() { return _GarmentType; }
				PressureSensorType DBG_GetPressureSensorType() { return _PressureSensorType; }
				DevicePosition DBG_GetDevicePosition() { return _DevicePosition; }

				int DBG_GetFramesProcessedCounter() { return FramesProcessedCounter; }
				int DBG_GetTotalNumberOfSensors() { return TotalNumberOfSensors; }
				int DBG_GetnSensorsPre() { return nSensorsPre; }
				int DBG_GetnSensorsAcc() { return nSensorsAcc; }

				fvector<double, nSensorsPre> DBG_GetProcessedSignalPre() { return ProcessedSignalPre; }
				fvector3d DBG_GetProcessedSignalAcc() { return ProcessedSignalAcc; }
				fvector3d DBG_GetProcessedSignalMag() { return ProcessedSignalMag; }
				fvector3d DBG_GetProcessedSignalGyr() { return ProcessedSignalGyr; }

				std::vector<CircularBuffer<double>> &DBG_GetRAWSignalBufferPre() { return RAWSignalBuffer[Pre]; }
				std::vector<CircularBuffer<double>> &DBG_GetRAWSignalBufferAcc() { return RAWSignalBuffer[Acc]; }
				std::vector<CircularBuffer<double>> &DBG_GetRAWSignalBufferMag() { return RAWSignalBuffer[Mag]; }
				std::vector<CircularBuffer<double>> &DBG_GetRAWSignalBufferGyr() { return RAWSignalBuffer[Gyr]; }

				std::vector<CircularBuffer<double>> &DBG_GetProcessedSignalBufferPre() { return ProcessedSignalBuffer[Pre]; }
				std::vector<CircularBuffer<double>> &DBG_GetProcessedSignalBufferAcc() { return ProcessedSignalBuffer[Acc]; }
				std::vector<CircularBuffer<double>> &DBG_GetProcessedSignalBufferMag() { return ProcessedSignalBuffer[Mag]; }
				std::vector<CircularBuffer<double>> &DBG_GetProcessedSignalBufferGyr() { return ProcessedSignalBuffer[Gyr]; }

				double *DBG_GetProcessedSignalMvAvgPre() { return ProcessedSignalMvAvg[Pre]; }
				double *DBG_GetProcessedSignalMvAvgAcc() { return ProcessedSignalMvAvg[Acc]; }
				double *DBG_GetProcessedSignalMvAvgMag() { return ProcessedSignalMvAvg[Mag]; }
				double *DBG_GetProcessedSignalMvAvgGyr() { return ProcessedSignalMvAvg[Gyr]; }

				SignalFeature *DBG_GetStepFlagPre() { return DBG_StepFlag[Pre]; }
				SignalFeature *DBG_GetStepFlagAcc() { return DBG_StepFlag[Acc]; }
				SignalFeature *DBG_GetStepFlagMag() { return DBG_StepFlag[Mag]; }
				SignalFeature *DBG_GetStepFlagGyr() { return DBG_StepFlag[Gyr]; }

				int    *DBG_GetStepCount()		{ return StepCount; }
				double *DBG_GetLocalCadenceOneFoot()  { return LocalCadenceOneFoot; }
				double *DBG_GetLocalCadenceTwoFeet()  { return LocalCadenceTwoFeet; }
				double *DBG_GetAverageCadenceTwoFeet() { return AverageCadenceTwoFeet; }

				double *DBG_GetGaitLength() { return GaitLength; }

				double DBG_FtrA() { return (double)FootPosition[Pre][0]; };
				double DBG_FtrB() { return (double)FootPosition[Pre][1]; };
				double DBG_FtrC() { return (double)FootPosition[Pre][2]; };
				double DBG_FtrD() { return (double)qOrientation.rotate(x)[0]; };
				double DBG_FtrE() { return (double)qOrientation.rotate(x)[1]; };
				double DBG_FtrF() { return (double)qOrientation.rotate(x)[2]; };
				double DBG_FtrG() { return (double)GaitRotatedAcc[K_GaitIntegrationDirection]; };
				double DBG_FtrH() { return (double)DBG_DynamicPre; };
				double DBG_FtrI() { return (double)0.0; };
			};
		}
	}
}
#endif __Sensoria__SignalProcessing__Gait__
