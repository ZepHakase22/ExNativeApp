//  SignalProcessing.Gait.cpp
//  Sensoria
//
//  Created by Maurizio Macagno / Stefano Rossotti on 03/23/18.
//  Copyright (c) 2018 Sensoria Inc. All rights reserved.
//

#include "Sensoria.SignalProcessing.Gait.h"

using namespace Sensoria::SignalProcessing::Gait;

// Constructor ---------------------------------------------------------------------------------------------------------------
SignalProcessing::SignalProcessing(double SamplingFrequency, GarmentType GarmentType, PressureSensorType PressureSensorType, DevicePosition DevicePosition, GaitEvents *objGaitEvents)
{
	_SamplingFrequency = SamplingFrequency;
	_GarmentType = GarmentType;
	_PressureSensorType = PressureSensorType;
	_DevicePosition = DevicePosition;
	_objGaitEvents = objGaitEvents;
	
	switch (_GarmentType)
	{
	case OHI:
		SensorTypeToUse = Pre;
		break;
	case OptimaBoot:
		SensorTypeToUse = Acc;
		break;
	default:
		SensorTypeToUse = Pre;
		break;
	}
	
	// Buffer and History
	std::vector<CircularBuffer<double>> SensorBufferVectorTmp;
	std::vector<std::vector<double>> LPHistoryBufferVectorTmp;
	MinBufferLength = std::numeric_limits<int>::max();
	MaxBufferLength = std::numeric_limits<int>::min();

	ProcessedSignalRange = new double*[nUsingSensorsTypes];
	ProcessedSignalMvAvg = new double*[nUsingSensorsTypes];
	ProcessedSignalStdDv = new double*[nUsingSensorsTypes];
	SensorBufferMA_Min   = new double*[nUsingSensorsTypes];
	SensorBufferMA_Max   = new double*[nUsingSensorsTypes];
	MinStdDv = new double[nUsingSensorsTypes];

	IsDynamicSignal = new bool*[nUsingSensorsTypes];

	for (int SensorTypeID = 0; SensorTypeID < nUsingSensorsTypes; SensorTypeID++)		// cycle over sensor types
	{
		ProcessedSignalRange[SensorTypeID] = new double[nSensors[SensorTypeID]];
		ProcessedSignalMvAvg[SensorTypeID] = new double[nSensors[SensorTypeID]];
		ProcessedSignalStdDv[SensorTypeID] = new double[nSensors[SensorTypeID]];
		SensorBufferMA_Min[SensorTypeID]   = new double[nSensors[SensorTypeID]];
		SensorBufferMA_Max[SensorTypeID]   = new double[nSensors[SensorTypeID]];

		IsDynamicSignal[SensorTypeID] = new bool[nSensors[SensorTypeID]];

		LocalCadenceHistory.push_back(CircularBuffer<double>(K_CadenceHistoryLength, 0.0));
		TimeOnGroundHistory.push_back(CircularBuffer<double>(K_TimeOnGroundHistoryLength, 0.0));
		BufferLength[SensorTypeID] = (int)(_SamplingFrequency * (double)MillisecsInBuffer[SensorTypeID] / 1000.0);

		MinBufferLength = std::min(MinBufferLength, BufferLength[SensorTypeID]);
		MaxBufferLength = std::max(MaxBufferLength, BufferLength[SensorTypeID]);

		//Filters
		LP.push_back(SensoriaLibrary::DSP::Filter());
		LP[SensorTypeID].MakeFilter(FilterOrder[SensorTypeID], FilterFrequencyLP[SensorTypeID], _SamplingFrequency, "LowPass", "Butter");

		for (int SensorID = 0; SensorID < nSensors[SensorTypeID]; SensorID++)	// cycle over single sensors
		{
			SensorBufferVectorTmp.push_back(CircularBuffer<double>(BufferLength[SensorTypeID]));
			LPHistoryBufferVectorTmp.push_back(std::vector<double>(FilterOrder[SensorTypeID]));
		}

		RAWSignalBuffer.push_back(SensorBufferVectorTmp);
		ProcessedSignalBuffer.push_back(SensorBufferVectorTmp);
		NoOutliersSignalBuffer.push_back(SensorBufferVectorTmp);
		LPHistory.push_back(LPHistoryBufferVectorTmp);

		SensorBufferVectorTmp.clear();
		LPHistoryBufferVectorTmp.clear();
	}

	DoSetReferenceVariables = true;

	MeanCrossTooFast = false;
	StepTimeInRange = false;
	CrossingDirection = 0;

	CurrentStepCrossUpFrame = new int*[nStepsSensorsTypes];
	PreviousStepCrossUpFrame = new int*[nStepsSensorsTypes];
	PreviousStepCrossDownFrame = new int*[nStepsSensorsTypes];
	PreviousCrossMeanFlag = new int*[nStepsSensorsTypes];
	CurrentCrossMeanFlag = new int*[nStepsSensorsTypes];
	FirstTimeCrossedUp = new bool*[nStepsSensorsTypes];
	SensorBufferMA_PeakThreshold = new double*[nStepsSensorsTypes];

	FramesFromLastDetection = new int[nStepsSensorsTypes];

	DBG_StepFlag = new SignalFeature*[nStepsSensorsTypes];
	DBG_DynamicAcc = new double[nSensorsAcc];

	MeanCrossNFramesThreshold = new int[nStepsSensorsTypes];
	MetricsResetFrames = new int[nStepsSensorsTypes];
	MaxStepSecs = new double[nStepsSensorsTypes];
	MinStepSecs = new double[nStepsSensorsTypes];
	FramesDelayForSyncAll = new int[nStepsSensorsTypes];
	StepCount = new int[nStepsSensorsTypes];

	FootPosition = new double*[nStepsSensorsTypes];
	GaitLength = new double[nStepsSensorsTypes];

	CadenceStartFrame = new int[nStepsSensorsTypes];
	CadenceEndFrame = new int[nStepsSensorsTypes];
	AverageCadenceTwoFeet = new double[nStepsSensorsTypes];
	LocalCadenceTwoFeet = new double[nStepsSensorsTypes];
	LocalCadenceOneFoot = new double[nStepsSensorsTypes];
	MinCadence = new double[nStepsSensorsTypes];
	MaxCadence = new double[nStepsSensorsTypes];


	MaxFramesDelayForSyncTwo = (int)(_SamplingFrequency * K_MaxMillisecsDelayForSyncAll / 1000.0);

	for (int SensorTypeID = 0; SensorTypeID < nStepsSensorsTypes; SensorTypeID++)		// cycle over sensor types
	{
		CurrentStepCrossUpFrame[SensorTypeID] = new int[nSensors[SensorTypeID]];
		PreviousStepCrossUpFrame[SensorTypeID] = new int[nSensors[SensorTypeID]];
		PreviousStepCrossDownFrame[SensorTypeID] = new int[nSensors[SensorTypeID]];
		PreviousCrossMeanFlag[SensorTypeID] = new int[nSensors[SensorTypeID]];
		CurrentCrossMeanFlag[SensorTypeID] = new int[nSensors[SensorTypeID]];
		FirstTimeCrossedUp[SensorTypeID] = new bool[nSensors[SensorTypeID]];

		LocalCadenceHistory.push_back(CircularBuffer<double>(K_CadenceHistoryLength));

		SensorBufferMA_PeakThreshold[SensorTypeID] = new double[nSensors[SensorTypeID]];

		DBG_StepFlag[SensorTypeID] = new SignalFeature[nSensors[SensorTypeID]];

		FootPosition[SensorTypeID] = new double[3];
	}

	switch (GarmentType)
	{
		case GarmentType::OptimaBoot:
			qCoreRotation = K_OptimaqAccRotation;
			break;
		case GarmentType::OHI:
			qCoreRotation = K_OHIAccRotation;
			break;
		default:
			qCoreRotation = quaternion(1.0, 0.0, 0.0, 0.0);
			break;
	}
	
	Reset(); // Reset the SP Library
}
SignalProcessing::~SignalProcessing()
{
	for (int SensorTypeID = 0; SensorTypeID < nStepsSensorsTypes; SensorTypeID++)		// cycle over sensor types
	{
		delete[] CurrentStepCrossUpFrame[SensorTypeID];
		delete[] PreviousStepCrossUpFrame[SensorTypeID];
		delete[] PreviousStepCrossDownFrame[SensorTypeID];
		delete[] PreviousCrossMeanFlag[SensorTypeID];
		delete[] CurrentCrossMeanFlag[SensorTypeID];
		delete[] FirstTimeCrossedUp[SensorTypeID];
		delete[] SensorBufferMA_PeakThreshold[SensorTypeID];

		delete[] DBG_StepFlag[SensorTypeID];
		delete[] FootPosition[SensorTypeID];
	}

	for (int SensorTypeID = 0; SensorTypeID < nUsingSensorsTypes; SensorTypeID++)		// cycle over sensor types
	{
		delete[] ProcessedSignalRange[SensorTypeID];
		delete[] ProcessedSignalMvAvg[SensorTypeID];
		delete[] ProcessedSignalStdDv[SensorTypeID];
		delete[] SensorBufferMA_Min[SensorTypeID];
		delete[] SensorBufferMA_Max[SensorTypeID];
	}

	delete[] CurrentStepCrossUpFrame;
	delete[] PreviousStepCrossUpFrame;
	delete[] PreviousStepCrossDownFrame;
	delete[] PreviousCrossMeanFlag;
	delete[] CurrentCrossMeanFlag;
	delete[] FirstTimeCrossedUp;
	delete[] SensorBufferMA_PeakThreshold;

	delete[] FramesFromLastDetection;

	delete[] ProcessedSignalRange;
	delete[] ProcessedSignalMvAvg;
	delete[] ProcessedSignalStdDv;
	delete[] SensorBufferMA_Min;
	delete[] SensorBufferMA_Max;
	delete[] MinStdDv;

	delete[] DBG_StepFlag;
	delete[] FootPosition;
	delete[] GaitLength;

	delete[] DBG_DynamicAcc;

	delete[] MeanCrossNFramesThreshold;
	delete[] MetricsResetFrames;
	delete[] MaxStepSecs;
	delete[] MinStepSecs;
	delete[] FramesDelayForSyncAll;
	delete[] StepCount;

	delete[] CadenceStartFrame;
	delete[] CadenceEndFrame;
	delete[] LocalCadenceTwoFeet;
	delete[] LocalCadenceOneFoot;
	delete[] MinCadence;
	delete[] MaxCadence;
}

void SignalProcessing::Reset()
{
	// Biases on raw data
	fvector3d MagRotationDirection(0.0, 0.0, 1.0);
	qMagAdjustment.set_rotation(PI, MagRotationDirection);
	qMagAdjustment = qCoreRotation * qMagAdjustment;

	// TODO Remove when calibration will come from external 
	double Offset_x;
	double Offset_y;
	double Offset_z;
	double ScaleF_x;
	double ScaleF_y;
	double ScaleF_z;

	// TODO Remove when calibration will come from external 
	Offset_x = -0.102662;
	Offset_y = -0.586604;
	Offset_z = 0.628171;
	ScaleF_x = 0.956649;
	ScaleF_y = 0.963535;
	ScaleF_z = 1.025377;

	BiasMag[0] = Offset_x;
	BiasMag[1] = Offset_y;
	BiasMag[2] = Offset_z;

	ScaleMag[0] = ScaleF_x;
	ScaleMag[1] = ScaleF_y;
	ScaleMag[2] = ScaleF_z;

	BiasGyr[0] =  1.12;
	BiasGyr[1] = -3.36;
	BiasGyr[2] = -6.72;

	// Frames counter 
	FramesProcessedCounter = 0;

	// Processed signal
	ProcessedSignalPre = 0.0;
	ProcessedSignalAcc = 0.0;
	ProcessedSignalMag = 0.0;
	ProcessedSignalGyr = 0.0;

	// DynamicSignal variables
	IsDynamicalRange = false;
	IsDynamicalOutlier = false;
	IsDynamicalAvgRange = false;

	memset(FramesDelayForSyncAll,	0, sizeof(FramesDelayForSyncAll[0]	) * nStepsSensorsTypes);
	memset(StepCount,				0, sizeof(StepCount[0]				) * nStepsSensorsTypes);
	memset(CadenceStartFrame,		0, sizeof(CadenceStartFrame[0]		) * nStepsSensorsTypes);
	memset(CadenceEndFrame,			0, sizeof(CadenceEndFrame[0]		) * nStepsSensorsTypes);
	memset(AverageCadenceTwoFeet,	0, sizeof(AverageCadenceTwoFeet[0]	) * nStepsSensorsTypes);
	memset(LocalCadenceTwoFeet,		0, sizeof(LocalCadenceTwoFeet[0]	) * nStepsSensorsTypes);
	memset(LocalCadenceOneFoot,		0, sizeof(LocalCadenceOneFoot[0]	) * nStepsSensorsTypes);
	memset(FramesFromLastDetection, 0, sizeof(FramesFromLastDetection[0]) * nStepsSensorsTypes);

	MinStdDv[Pre] = K_MinStDvPre;
	MinStdDv[Acc] = K_MinStDvAcc;
	MinStdDv[Mag] = K_MinStDvMag;
	MinStdDv[Gyr] = K_MinStDvGyr;

	memset(DBG_DynamicAcc,	0, sizeof(DBG_DynamicAcc[0]) * nSensorsAcc);
	memset(GaitLength,		0, sizeof(GaitLength[0])	 * nSensorsAcc);
	
	
	for (int SensorTypeID = 0; SensorTypeID < nStepsSensorsTypes; SensorTypeID++)		// cycle over sensor types
	{
		MetricsResetFrames[SensorTypeID] = (int)(_SamplingFrequency * K_DefaultMetricsResetSeconds);
		MinStepSecs[SensorTypeID] = K_MinStepSecs;
		MaxStepSecs[SensorTypeID] = K_MaxStepSecs;
		LocalCadenceHistory[SensorTypeID].Clear();
		TimeOnGroundHistory[SensorTypeID].Clear();
		DetectedActivity[SensorTypeID] = ActivityUnknown;

		MinCadence[SensorTypeID] = K_MinCadence;
		MaxCadence[SensorTypeID] = K_MaxCadence;

		MeanCrossNFramesThreshold[SensorTypeID] = (int)(_SamplingFrequency * K_MeanCrossMillisecsThreshold/1000.0);

		SensorsInSyncMask[SensorTypeID] = None;

		memset(CurrentStepCrossUpFrame[SensorTypeID]	, 0, sizeof(CurrentStepCrossUpFrame[SensorTypeID][0]	) * nSensors[SensorTypeID]);
		memset(PreviousStepCrossUpFrame[SensorTypeID]	, 0, sizeof(PreviousStepCrossUpFrame[SensorTypeID][0]	) * nSensors[SensorTypeID]);
		memset(PreviousStepCrossDownFrame[SensorTypeID]	, 0, sizeof(PreviousStepCrossDownFrame[SensorTypeID][0]	) * nSensors[SensorTypeID]);
		memset(PreviousCrossMeanFlag[SensorTypeID]		, 0, sizeof(PreviousCrossMeanFlag[SensorTypeID][0]		) * nSensors[SensorTypeID]);
		memset(CurrentCrossMeanFlag[SensorTypeID]		, 0, sizeof(CurrentCrossMeanFlag[SensorTypeID][0]		) * nSensors[SensorTypeID]);
		memset(FirstTimeCrossedUp[SensorTypeID]			, 0, sizeof(FirstTimeCrossedUp[SensorTypeID][0]			) * nSensors[SensorTypeID]);
		memset(FootPosition[SensorTypeID]				, 0, sizeof(FootPosition[SensorTypeID][0]				) * 3);

		memset(DBG_StepFlag[SensorTypeID], 0, sizeof(DBG_StepFlag[SensorTypeID][0]) * nSensors[SensorTypeID]);

		qStepReferenceOrientation[SensorTypeID] = 1.0;

		for (int SensorID = 0; SensorID < nSensors[SensorTypeID]; SensorID++)	// cycle over single sensors
		{
			SensorBufferMA_PeakThreshold[SensorTypeID][SensorID] = DefaultPeakThreshold[SensorTypeID];
			RAWSignalBuffer[SensorTypeID][SensorID].Clear();
			ProcessedSignalBuffer[SensorTypeID][SensorID].Clear();
			NoOutliersSignalBuffer[SensorTypeID][SensorID].Clear();

			for (int order = 0; order < FilterOrder[SensorTypeID]; order++) {
				LPHistory[SensorTypeID][SensorID][order] = 0;
			}
		}
	}

	for (int SensorTypeID = 0; SensorTypeID < nUsingSensorsTypes; SensorTypeID++)		// cycle over sensor types
	{
		memset(ProcessedSignalRange[SensorTypeID],	0, sizeof(ProcessedSignalRange[SensorTypeID][0]) * nSensors[SensorTypeID]);
		memset(ProcessedSignalMvAvg[SensorTypeID],	0, sizeof(ProcessedSignalMvAvg[SensorTypeID][0]) * nSensors[SensorTypeID]);
		memset(ProcessedSignalStdDv[SensorTypeID],	0, sizeof(ProcessedSignalStdDv[SensorTypeID][0]) * nSensors[SensorTypeID]);
		memset(IsDynamicSignal[SensorTypeID],		0, sizeof(IsDynamicSignal[SensorTypeID][0])		 * nSensors[SensorTypeID]);
		std::fill(SensorBufferMA_Min[SensorTypeID], SensorBufferMA_Min[SensorTypeID] + nSensors[SensorTypeID], +std::numeric_limits<double>::max());
		std::fill(SensorBufferMA_Max[SensorTypeID], SensorBufferMA_Max[SensorTypeID] + nSensors[SensorTypeID], -std::numeric_limits<double>::max());
	}

	RatioRange    = 0.0;
	RatioOutlier  = 0.0;
	RatioAvgRange = 0.0;

	// Device Worn
	nFramesToCheckWorn = (int)(_SamplingFrequency * K_SecondsToCheckWorn);
	nFramesToConsiderNotWorn = (int)(_SamplingFrequency * K_SecondsToConsiderNotWorn);
	IsAccDynamical = false;
	IsPreAboveThreshold = false;
	WornFlag = false;
	IsDeviceWorn = false;
	nFramesNotWorn = 0;


	// Orientation of the IMU
	qOrientation = 1.0;
	qOldOrientation[0] = 1.0;
	qOldOrientation[1] = 1.0;
	qOrientationMag = 1.0;
	qOrientationAcc = 1.0;
	qOrientationMagAcc = 1.0;
	qOrientationAccMag = 1.0;
	qOrientationAll = 1.0;
	qReferenceOrientation = 1.0;
	qOmega = 0.0;
	qOrientationVerlet;
	qDeltaOrientationMag;

	OldMag = 0.0;
	ReferenceGravity = 0.0;
	ReferenceMag = 0.0;
	PredictedGravityOrientation = 0.0;
	GravityAngularDeviation = 0.0;
	GravityNorm = 1.0;
	
	// Gait length
	GaitRotatedAcc = 0.0;

	// Global variables
	GlobalStepCount = 0;
	GlobalCadence = 0.0;
	GlobalTimeOnGround = 0.0;
	GlobalGaitLength = 0.0;
	GlobalDetectedActivity = ActivityNone;

	// Debug
	x = 0.0; x[0] = 1.0;
	y = 0.0; y[1] = 1.0;
	z = 0.0; z[2] = 1.0;
}

// Functions ---------------------------------------------------------------------------------------------------------------
void SignalProcessing::ProcessIncomingData(const double *RAWIncomingPre, const double *RAWIncomingAcc, const double *RAWIncomingMag, const double *RAWIncomingGyr)
{
	ResetFlags();

	if (false)	// TODO add check to change the type of the sensor to use in case Pre or Acc stops working
		SwitchSensorTypeToUse(Acc);

	// Create fixed vectors for vectorial quantities
	IncomingAcc = RAWIncomingAcc;
	IncomingMag = RAWIncomingMag;
	IncomingGyr = RAWIncomingGyr;

	// Process Incoming signals
	ProcessIncomingPre(RAWIncomingPre);
	ProcessIncomingAcc(IncomingAcc);
	ProcessIncomingMag(IncomingMag);
	ProcessIncomingGyr(IncomingGyr);

	// Compute steps
	CycleOverSensorsPre();
	CycleOverSensorsAcc();
	CycleOverSensorsMag();
	CycleOverSensorsGyr();

	// Is Worn
	if (FramesProcessedCounter % nFramesToCheckWorn)
		CheckIsDeviceWorn();

	// Orientation and GaitLength
	if (FramesProcessedCounter > MaxBufferLength)
	{
		if (DoSetReferenceVariables)
		{
			SetReferenceVariables();
			DoSetReferenceVariables = false;
		}
	
		CalculateOrientation();
		IntegrateGaitLength();
	}

	// Update number of processed frames
	++FramesProcessedCounter;
}

// Signal Processing
void SignalProcessing::ProcessIncomingPre(const double    *IncomingPre)
{
	for (int SensorID = 0; SensorID < nSensors[Pre]; SensorID++)
	{
		// Filling RAW buffer
		RAWSignalBuffer[Pre][SensorID].Add(IncomingPre[SensorID]);

		// Switch pressure sensor type
		switch (_PressureSensorType)
		{
			case Sensoria::SignalProcessing::Common::PressureSensorType::Textile:
				ProcessedSignalPre[SensorID] = 1023 - IncomingPre[SensorID];
				break;
			case Sensoria::SignalProcessing::Common::PressureSensorType::FSR:
				ProcessedSignalPre[SensorID] = IncomingPre[SensorID];
				break;
			default:
			case Sensoria::SignalProcessing::Common::PressureSensorType::TypeUnknown:
				ProcessedSignalPre[SensorID] = -1;
			break;
		}

		if (FramesProcessedCounter >= BufferLength[Pre])	// Fill the whole buffer for the first time
		{
			DetectOutliersInSignal(ProcessedSignalPre[SensorID], NoOutliersSignalBuffer[Pre][SensorID], Pre);	// detect outliers
		}		
		else												// Buffer filled. Search for outliers
		{
			NoOutliersSignalBuffer[Pre][SensorID].Add(ProcessedSignalPre[SensorID]);
		}

		// Filter
		ProcessedSignalPre[SensorID] = SensoriaLibrary::DSP::FiltersFunctions::filter(LP[Pre], ProcessedSignalPre[SensorID], LPHistory[Pre][SensorID]);	// filter

		// Fill buffer
		ProcessedSignalBuffer[Pre][SensorID].Add(ProcessedSignalPre[SensorID]);
	}
}
void SignalProcessing::ProcessIncomingAcc(const fvector3d &IncomingAcc)
{
	ProcessedSignalAcc = qCoreRotation.rotate(IncomingAcc);

	for (int SensorID = 0; SensorID < nSensors[Acc]; SensorID++)
	{
		// Fill the whole buffer for the first time
		if (FramesProcessedCounter >= BufferLength[Acc])
		{
			DetectOutliersInSignal(ProcessedSignalAcc[SensorID], NoOutliersSignalBuffer[Acc][SensorID], Acc);	// detect outliers
		}
		// Buffer filled. Search for outliers
		else
		{
			NoOutliersSignalBuffer[Acc][SensorID].Add(ProcessedSignalAcc[SensorID]);
		}

		// Filter
		ProcessedSignalAcc[SensorID] = SensoriaLibrary::DSP::FiltersFunctions::filter(LP[Acc], ProcessedSignalAcc[SensorID], LPHistory[Acc][SensorID]);	
	
		// Fill buffer
		ProcessedSignalBuffer[Acc][SensorID].Add(ProcessedSignalAcc[SensorID]);
	}
}
void SignalProcessing::ProcessIncomingMag(const fvector3d &IncomingMag) {
	ProcessedSignalMag = qMagAdjustment.rotate((IncomingMag - BiasMag) | ScaleMag);

	for (int SensorID = 0; SensorID < nSensors[Mag]; SensorID++)
	{
	//	// Buffer filled. Search for outliers
	//	if (FramesProcessedCounter >= BufferLength[Mag])
	//	{
	//		DetectOutliersInSignal(ProcessedSignalMag[SensorID], NoOutliersSignalBuffer[Mag][SensorID], Mag);	// detect outliers
	//	}
	//	// Fill the whole buffer for the first time
	//	else
	//	{
	//		NoOutliersSignalBuffer[Mag][SensorID].Add(ProcessedSignalMag[SensorID]);
	//	}

	//	//// Filter
	//	ProcessedSignalMag[SensorID] = SensoriaLibrary::DSP::FiltersFunctions::filter(LP[Mag], ProcessedSignalMag[SensorID], LPHistory[Mag][SensorID]);

	//	// Fill buffer
		ProcessedSignalBuffer[Mag][SensorID].Add(ProcessedSignalMag[SensorID]);
	}
}
void SignalProcessing::ProcessIncomingGyr(const fvector3d &IncomingGyr) {
	ProcessedSignalGyr = qCoreRotation.rotate(IncomingGyr - BiasGyr);

	for (int SensorID = 0; SensorID < nSensors[Gyr]; SensorID++)
	{
	//	// Fill the whole buffer for the first time
	//	if (FramesProcessedCounter >= BufferLength[Gyr])
	//	{
	//		DetectOutliersInSignal(ProcessedSignalGyr[SensorID], NoOutliersSignalBuffer[Gyr][SensorID], Gyr);	// detect outliers
	//	}
	//	// Buffer filled. Search for outliers
	//	else
	//	{
	//		NoOutliersSignalBuffer[Gyr][SensorID].Add(ProcessedSignalGyr[SensorID]);
	//	}

		// Filter
		ProcessedSignalGyr[SensorID] = SensoriaLibrary::DSP::FiltersFunctions::filter(LP[Gyr], ProcessedSignalGyr[SensorID], LPHistory[Gyr][SensorID]);

		// Fill buffer
		ProcessedSignalBuffer[Gyr][SensorID].Add(ProcessedSignalGyr[SensorID]);
	}
}

// Step computation
void SignalProcessing::CycleOverSensorsPre()
{
	ComputingSensorTypeID = Pre;

	// if too much time elapsed from the last calculation, reset all the metrics
	if (FramesFromLastDetection[Pre] > MetricsResetFrames[Pre])
		ResetMetrics();

	if (FramesProcessedCounter >= BufferLength[Pre])
	{
		for (ComputingSensorID = 0; ComputingSensorID < nSensors[Pre]; ComputingSensorID++)
		{
			if (CheckSensor(SensorsToSyncMask[Pre], ComputingSensorID)) {
				CalculateStatisticalVariables();
				DetectStepPre();
			}
		}
		
		UpdateStepPre();
	}
}
void SignalProcessing::CycleOverSensorsAcc()
{
	ComputingSensorTypeID = Acc;

	// if too much time elapsed from the last calculation, reset all the metrics
	if (FramesFromLastDetection[Acc] > MetricsResetFrames[Acc])
		ResetMetrics();

	if (FramesProcessedCounter >= BufferLength[Acc])	// fill the whole buffer for the first time
	{
		for (int ProcessingSensorID = 0; ProcessingSensorID < nSensors[Acc]; ProcessingSensorID++)
		{
			if (CheckSensor(SensorsToSyncMask[Acc], ProcessingSensorID)) {
				CalculateStatisticalVariables();
				DetectStepAcc();
			}
		}

		UpdateStepAcc();
	}
}
void SignalProcessing::CycleOverSensorsMag()
{
	ComputingSensorTypeID = Mag;

	if (FramesProcessedCounter >= BufferLength[Gyr])	// fill the whole buffer for the first time
	{
		for (int SensorID = 0; SensorID < nSensors[Gyr]; SensorID++)
		{
			if (CheckSensor(SensorsToSyncMask[Gyr], SensorID))
				CalculateStatisticalVariables();
		}
	}
}
void SignalProcessing::CycleOverSensorsGyr()
{
	ComputingSensorTypeID = Gyr;

	if (FramesProcessedCounter >= BufferLength[Gyr])	// fill the whole buffer for the first time
	{
		for (int SensorID = 0; SensorID < nSensors[Gyr]; SensorID++)
		{
			if (CheckSensor(SensorsToSyncMask[Gyr], SensorID)) {
				CalculateStatisticalVariables();
			}
		}
	}
}

// Step detection
void SignalProcessing::DetectStepPre()
{
	CurrentDataPoint = ProcessedSignalBuffer[Pre][ComputingSensorID].Last();

	SensorBufferMA_Max[Pre][ComputingSensorID] = std::max(SensorBufferMA_Max[Pre][ComputingSensorID], CurrentDataPoint);
	SensorBufferMA_Min[Pre][ComputingSensorID] = std::min(SensorBufferMA_Min[Pre][ComputingSensorID], CurrentDataPoint);

	PreviousCrossMeanFlag[Pre][ComputingSensorID] = CurrentCrossMeanFlag[Pre][ComputingSensorID];
	IsDynamicSignal[Pre][ComputingSensorID] = CheckIsDynamicalPre();

	if (IsDynamicSignal[Pre][ComputingSensorID])
	{
		if (CurrentDataPoint > ProcessedSignalMvAvg[Pre][ComputingSensorID] + SensorBufferMA_PeakThreshold[Pre][ComputingSensorID]) // if the signal is above the average
		{
			CrossingDirection = +1 - PreviousCrossMeanFlag[Pre][ComputingSensorID];
		}
		else if (CurrentDataPoint < ProcessedSignalMvAvg[Pre][ComputingSensorID] - SensorBufferMA_PeakThreshold[Pre][ComputingSensorID])
		{
			CrossingDirection = -1 - PreviousCrossMeanFlag[Pre][ComputingSensorID];
		}
		else
		{
			CrossingDirection = 0;
		}

		if (CrossingDirection > 0) // signal crossed the average on way up
		{
			DBG_StepFlag[Pre][ComputingSensorID] = CrossUp;

			if (!FirstTimeCrossedUp[Pre][ComputingSensorID])
			{
				DBG_StepFlag[Pre][ComputingSensorID] = CrossUpAccepted;
				CurrentCrossMeanFlag[Pre][ComputingSensorID] = 1;
				CurrentStepCrossUpFrame[Pre][ComputingSensorID] = FramesProcessedCounter;

				CadenceStartFrame[Pre] = FramesProcessedCounter;
				FirstTimeCrossedUp[Pre][ComputingSensorID] = true;
				FramesFromLastDetection[Pre] = 0;

				memset(FootPosition[Pre], 0, sizeof(FootPosition[Pre][0]) * 3);
			}
			else
			{
				MeanCrossTooFast = SignalProcessingLibrary::IsMeanCrossTooFast(PreviousStepCrossDownFrame[Pre][ComputingSensorID], FramesProcessedCounter, MeanCrossNFramesThreshold[Pre]);
				StepTimeInRange  = SignalProcessingLibrary::IsStepTimeInRange(FramesProcessedCounter - CurrentStepCrossUpFrame[Pre][ComputingSensorID], _SamplingFrequency, MinStepSecs[Pre], MaxStepSecs[Pre]);

				if (StepTimeInRange && !MeanCrossTooFast)
				{
					DBG_StepFlag[Pre][ComputingSensorID] = CrossUpAccepted;
					CurrentCrossMeanFlag[Pre][ComputingSensorID] = 1;
					CurrentStepCrossUpFrame[Pre][ComputingSensorID] = FramesProcessedCounter;
					SensorsInSyncMask[Pre] = (SyncMask)(SensorsInSyncMask[Pre] | SensorsList[ComputingSensorID]);

					PreviousStepCrossUpFrame[Pre][ComputingSensorID] = CurrentStepCrossUpFrame[Pre][ComputingSensorID];
					EvaluateSensorsInSync();

					FramesFromLastDetection[Pre] = 0;
				}
				else
				{
					DBG_StepFlag[Pre][ComputingSensorID] = CrossUpFailed;
				}
			}
		}

		if (CrossingDirection < 0) // signal crossed the average on way down
		{
			DBG_StepFlag[Pre][ComputingSensorID] = CrossDown;

			MeanCrossTooFast = SignalProcessingLibrary::IsMeanCrossTooFast(CurrentStepCrossUpFrame[Pre][ComputingSensorID], FramesProcessedCounter, MeanCrossNFramesThreshold[Pre]);
			StepTimeInRange = SignalProcessingLibrary::IsStepTimeInRange(FramesProcessedCounter - PreviousStepCrossDownFrame[Pre][ComputingSensorID], _SamplingFrequency, MinStepSecs[Pre], MaxStepSecs[Pre]);

			if (!MeanCrossTooFast && StepTimeInRange)
			{
				DBG_StepFlag[Pre][ComputingSensorID] = CrossDownAccepted;
				PreviousStepCrossDownFrame[Pre][ComputingSensorID] = FramesProcessedCounter;
				CurrentCrossMeanFlag[Pre][ComputingSensorID] = -1;
			}
			else
			{
				DBG_StepFlag[Pre][ComputingSensorID] = CrossDownFailed;
			}
		}
	}
	if (CurrentDataPoint > K_PreWornThreshold)
	{
		IsPreAboveThreshold = true;
	}
}
void SignalProcessing::DetectStepAcc()
{
	CurrentDataPoint = ProcessedSignalBuffer[Acc][ComputingSensorID].Last();

	SensorBufferMA_Max[Acc][ComputingSensorID] = std::max(SensorBufferMA_Max[Acc][ComputingSensorID], CurrentDataPoint);
	SensorBufferMA_Min[Acc][ComputingSensorID] = std::min(SensorBufferMA_Min[Acc][ComputingSensorID], CurrentDataPoint);

	PreviousCrossMeanFlag[Acc][ComputingSensorID] = CurrentCrossMeanFlag[Acc][ComputingSensorID];
	IsDynamicSignal[Acc][ComputingSensorID] = CheckIsDynamicalAcc();

	if (IsDynamicSignal[Acc][ComputingSensorID])
	{
		IsAccDynamical = true;

		if (CurrentDataPoint > ProcessedSignalMvAvg[Acc][ComputingSensorID] + SensorBufferMA_PeakThreshold[Acc][ComputingSensorID]) // if the signal is above the average
		{
			CrossingDirection = +1 - PreviousCrossMeanFlag[Acc][ComputingSensorID];
		}
		else if (CurrentDataPoint < ProcessedSignalMvAvg[Acc][ComputingSensorID] - SensorBufferMA_PeakThreshold[Acc][ComputingSensorID])
		{
			CrossingDirection = -1 - PreviousCrossMeanFlag[Acc][ComputingSensorID];
		}
		else
		{
			CrossingDirection = 0;
		}

		if (CrossingDirection > 0) // signal crossed the average on way up
		{
			DBG_StepFlag[Acc][ComputingSensorID] = CrossUp;

			if (!FirstTimeCrossedUp[Acc][ComputingSensorID])
			{
				DBG_StepFlag[Acc][ComputingSensorID] = CrossUpAccepted;
				CurrentCrossMeanFlag[Acc][ComputingSensorID] = 1;
				CurrentStepCrossUpFrame[Acc][ComputingSensorID] = FramesProcessedCounter;

				CadenceStartFrame[Acc] = FramesProcessedCounter;
				FirstTimeCrossedUp[Acc][ComputingSensorID] = true;
				FramesFromLastDetection[Acc] = 0;

				memset(FootPosition[Acc], 0, sizeof(FootPosition[Acc][0]) * 3);
			}
			else
			{
				MeanCrossTooFast = SignalProcessingLibrary::IsMeanCrossTooFast(PreviousStepCrossDownFrame[Acc][ComputingSensorID], FramesProcessedCounter, MeanCrossNFramesThreshold[Acc]);
				StepTimeInRange = SignalProcessingLibrary::IsStepTimeInRange(FramesProcessedCounter - CurrentStepCrossUpFrame[Acc][ComputingSensorID], _SamplingFrequency, MinStepSecs[Acc], MaxStepSecs[Acc]);

				if (StepTimeInRange && !MeanCrossTooFast)
				{
					DBG_StepFlag[Acc][ComputingSensorID] = CrossUpAccepted;
					CurrentCrossMeanFlag[Acc][ComputingSensorID] = 1;
					CurrentStepCrossUpFrame[Acc][ComputingSensorID] = FramesProcessedCounter;
					SensorsInSyncMask[Acc] = (SyncMask)(SensorsInSyncMask[Acc] | SensorsList[ComputingSensorID]);

					PreviousStepCrossUpFrame[Acc][ComputingSensorID] = CurrentStepCrossUpFrame[Acc][ComputingSensorID];
					EvaluateSensorsInSync();

					FramesFromLastDetection[Acc] = 0;
				}
				else
				{
					DBG_StepFlag[Acc][ComputingSensorID] = CrossUpFailed;
				}
			}
		}

		if (CrossingDirection < 0 && CurrentCrossMeanFlag[Acc][ComputingSensorID] == +1) // signal crossed the average on way down
		{
			DBG_StepFlag[Acc][ComputingSensorID] = CrossDown;

			MeanCrossTooFast = SignalProcessingLibrary::IsMeanCrossTooFast(CurrentStepCrossUpFrame[Acc][ComputingSensorID], FramesProcessedCounter, MeanCrossNFramesThreshold[Acc]);
			StepTimeInRange = SignalProcessingLibrary::IsStepTimeInRange(FramesProcessedCounter - PreviousStepCrossDownFrame[Acc][ComputingSensorID], _SamplingFrequency, MinStepSecs[Acc], MaxStepSecs[Acc]);

			if (!MeanCrossTooFast && StepTimeInRange)
			{
				DBG_StepFlag[Acc][ComputingSensorID] = CrossDownAccepted;
				PreviousStepCrossDownFrame[Acc][ComputingSensorID] = FramesProcessedCounter;
				CurrentCrossMeanFlag[Acc][ComputingSensorID] = -1;
			}
			else
			{
				DBG_StepFlag[Acc][ComputingSensorID] = CrossDownFailed;
			}
		}
	}
}

// Update steps
void SignalProcessing::UpdateStepPre()
{
	if (CountSensorsInSync() >= nMinSensorsInSyncPre)
	{
		// Increase number of steps taken 
		++StepCount[Pre];

		for (int SensorID = 0; SensorID < nSensors[Pre]; SensorID++)
		{
			DBG_StepFlag[Pre][SensorID] = StepTaken;
		}

		// Caluclate peaks height and reset all the Max and Min for the pressure
		for (int SensorID = 0; SensorID < nSensors[Pre]; SensorID++)
		{
			SensorBufferMA_PeakThreshold[Pre][SensorID] = (SensorBufferMA_Max[Pre][SensorID] - SensorBufferMA_Min[Pre][SensorID]) * K_PeakThresholdRatio;
		}

		std::fill(SensorBufferMA_Min[Pre], SensorBufferMA_Min[Pre] + nSensors[Pre], +std::numeric_limits<double>::max());
		std::fill(SensorBufferMA_Max[Pre], SensorBufferMA_Max[Pre] + nSensors[Pre], -std::numeric_limits<double>::max());

		// Update Cadence
		CadenceEndFrame[Pre]     = AverageInSyncTime(); // get the end time index for cadence as the average step end time of all in sync sensors
		LocalCadenceOneFoot[Pre] = DetectOutliersInCadence(CadenceStartFrame[Pre], CadenceEndFrame[Pre], MinCadence[Pre], MaxCadence[Pre]);
		CadenceStartFrame[Pre]   = CadenceEndFrame[Pre];

		LocalCadenceTwoFeet[Pre] = 120.0 * LocalCadenceOneFoot[Pre];	// 2.0 (for taking into account both feet) * 60.0 (for having cadence in steps per Minute)
		LocalCadenceHistory[Pre].Add(LocalCadenceTwoFeet[Pre]);
		LocalCadenceTwoFeet[Pre] = LocalCadenceHistory[Pre].Average();
		AverageCadenceTwoFeet[Pre] = ((StepCount[Pre] - 1) * AverageCadenceTwoFeet[Pre] + LocalCadenceTwoFeet[Pre]) / StepCount[Pre];

		// Update Metrics Time and Mena Cross Threshold
		MetricsResetFrames[Pre] = (int)(std::round(K_MetricsResetNumberOfSteps * _SamplingFrequency / LocalCadenceOneFoot[Pre]));
		MeanCrossNFramesThreshold[Pre] = (int)(_SamplingFrequency * K_MeanCrossThresholdRatio / LocalCadenceOneFoot[Pre]);

		// Update of the Minimum and Maximum lenght of the next step
		MinCadence[Pre]  = std::max(K_MinCadence,  (1.0 - K_MaxCadenceVariation)  * LocalCadenceOneFoot[Pre]);
		MaxCadence[Pre]  = std::min(K_MaxCadence,  (1.0 + K_MaxCadenceVariation)  * LocalCadenceOneFoot[Pre]);
		MinStepSecs[Pre] = std::max(K_MinStepSecs, (1.0 - K_MaxStepSecsVariation) / LocalCadenceOneFoot[Pre]);
		MaxStepSecs[Pre] = std::min(K_MaxStepSecs, (1.0 + K_MaxStepSecsVariation) / LocalCadenceOneFoot[Pre]);

		// After step has been computed reset variables
		SensorsInSyncMask[Pre] = None;
		FramesDelayForSyncAll[Pre] = 0;
		
		// Set Activity Type
		DetectedActivity[Pre] = ActivityWalking;

		// Update Global Variables
		if (SensorTypeToUse == Acc)
		{
			SetGlobalMetrics();
			SetGlobalActivity();
		}

		// Orientation and Gait length
		qStepReferenceOrientation[Pre] = qOrientation;
		GaitLength[Pre] = FootPosition[Pre][2] * K_GravityAcceleration;
		memset(FootPosition[Pre], 0, sizeof(FootPosition[Pre][0]) * 3);
	}
	else
	{
		++FramesFromLastDetection[Pre];
		++FramesDelayForSyncAll[Pre];
	}
}
void SignalProcessing::UpdateStepAcc()
{
	if (CountSensorsInSync() >= nMinSensorsInSyncAcc)
	{
		// Increase number of steps taken 
		++StepCount[Acc];

		for (int SensorID = 0; SensorID < nSensors[Acc]; SensorID++)
		{
			DBG_StepFlag[Acc][SensorID] = StepTaken;
		}

		// Caluclate peaks height and reset all the Max and Min for the pressure
		for (int SensorID = 0; SensorID < nSensors[Acc]; SensorID++)
		{
			SensorBufferMA_PeakThreshold[Acc][SensorID] = (SensorBufferMA_Max[Acc][SensorID] - SensorBufferMA_Min[Acc][SensorID]) * K_PeakThresholdRatio;
		}

		std::fill(SensorBufferMA_Min[Acc], SensorBufferMA_Min[Acc] + nSensors[Acc], +std::numeric_limits<double>::max());
		std::fill(SensorBufferMA_Max[Acc], SensorBufferMA_Max[Acc] + nSensors[Acc], -std::numeric_limits<double>::max());

		// Update Cadence
		CadenceEndFrame[Acc]	 = AverageInSyncTime(); // get the end time index for cadence as the average step end time of all in sync sensors
		LocalCadenceOneFoot[Acc] = DetectOutliersInCadence(CadenceStartFrame[Acc], CadenceEndFrame[Acc], MinCadence[Acc], MaxCadence[Acc]);
		CadenceStartFrame[Acc]	 = CadenceEndFrame[Acc];

		LocalCadenceTwoFeet[Acc] = 120.0 * LocalCadenceOneFoot[Acc];	// 2.0 (for taking into account both feet) * 60.0 (for having cadence in steps per Minute)
		LocalCadenceHistory[Acc].Add(LocalCadenceTwoFeet[Acc]);
		LocalCadenceTwoFeet[Acc] = LocalCadenceHistory[Acc].Average();
		AverageCadenceTwoFeet[Acc] = ((StepCount[Acc] - 1) * AverageCadenceTwoFeet[Acc] + LocalCadenceTwoFeet[Acc]) / StepCount[Acc];

		// Update Metrics Time and Mena Cross Threshold
		MetricsResetFrames[Acc] = (int)(std::round(K_MetricsResetNumberOfSteps * _SamplingFrequency / LocalCadenceOneFoot[Acc]));
		MeanCrossNFramesThreshold[Acc] = (int)(_SamplingFrequency * K_MeanCrossThresholdRatio / LocalCadenceOneFoot[Acc]);

		// Update of the Minimum and Maximum lenght of the next step
		MinCadence[Acc]  = std::max(K_MinCadence,  (1.0 - K_MaxCadenceVariation)  * LocalCadenceOneFoot[Acc]);
		MaxCadence[Acc]  = std::min(K_MaxCadence,  (1.0 + K_MaxCadenceVariation)  * LocalCadenceOneFoot[Acc]);
		MinStepSecs[Acc] = std::max(K_MinStepSecs, (1.0 - K_MaxStepSecsVariation) / LocalCadenceOneFoot[Acc]);
		MaxStepSecs[Acc] = std::min(K_MaxStepSecs, (1.0 + K_MaxStepSecsVariation) / LocalCadenceOneFoot[Acc]);

		// After step has been computed reset variables
		SensorsInSyncMask[Acc] = None;
		FramesDelayForSyncAll[Acc] = 0;

		// Set Activity Type
		DetectedActivity[Acc] = ActivityWalking;

		// Update Global Variables
		if (SensorTypeToUse == Acc)
		{
			SetGlobalMetrics();
			SetGlobalActivity();
		}

		// Orientation and Gait length
		qStepReferenceOrientation[Acc] = qOrientation;
		GaitLength[Acc] = FootPosition[Acc][2] * K_GravityAcceleration;
		memset(FootPosition[Acc], 0, sizeof(FootPosition[Acc][0]) * 3);
	}
	else
	{
		++FramesFromLastDetection[Acc];
		++FramesDelayForSyncAll[Acc];
	}
}

// Reference setting
void SignalProcessing::SetReferenceVariables()
{
	ReferenceGravity = 0.0;
	ReferenceMag = 0.0;

	for (int i = 1; i <= K_DefaultNReferenceSamples; i++)
	{
		for (int SensorID = 0; SensorID < nSensors[Acc]; SensorID++)
			ReferenceGravity[SensorID] += ProcessedSignalBuffer[Acc][SensorID][BufferLength[Acc] - i];
		
		for (int SensorID = 0; SensorID < nSensors[Mag]; SensorID++)
			ReferenceMag[SensorID] += ProcessedSignalBuffer[Mag][SensorID][BufferLength[Mag] - i];
	}

	ReferenceGravity = ReferenceGravity / K_DefaultNReferenceSamples;
	ReferenceMag = ReferenceMag / K_DefaultNReferenceSamples;
	
	GravityNorm = ReferenceGravity.norm();
	ReferenceGravity.normalize();
	OldMag = ReferenceMag;
}

// Reset metrics
void SignalProcessing::ResetMetrics()
{
	MaxStepSecs[ComputingSensorTypeID] = 0.0;

	MetricsResetFrames[ComputingSensorTypeID] += (int)(_SamplingFrequency * K_DefaultMetricsResetSeconds);

	LocalCadenceHistory[ComputingSensorTypeID].Clear();
	LocalCadenceTwoFeet[ComputingSensorTypeID] = 0.0;
	LocalCadenceOneFoot[ComputingSensorTypeID] = 0.0;
	TimeOnGround[ComputingSensorTypeID] = 0.0;
	EvaluateRestActivity(); 

	if (SensorTypeToUse == ComputingSensorTypeID)
	{
		SetGlobalMetrics();
		SetGlobalActivity();
	}

	GaitLength[ComputingSensorTypeID] = 0.0;

	CadenceEndFrame[ComputingSensorTypeID] = FramesProcessedCounter;
	memset(FirstTimeCrossedUp[ComputingSensorTypeID],		0, sizeof(FirstTimeCrossedUp[ComputingSensorTypeID][0]		) * nSensors[ComputingSensorTypeID]);
	memset(PreviousCrossMeanFlag[ComputingSensorTypeID],	0, sizeof(PreviousCrossMeanFlag[ComputingSensorTypeID][0]	) * nSensors[ComputingSensorTypeID]);
	memset(CurrentCrossMeanFlag[ComputingSensorTypeID],		0, sizeof(CurrentCrossMeanFlag[ComputingSensorTypeID][0]	) * nSensors[ComputingSensorTypeID]);
	memset(FootPosition[ComputingSensorTypeID],				0, sizeof(FootPosition[ComputingSensorTypeID][0]			) * 3);
}

// Calculate statistical variables
void SignalProcessing::CalculateStatisticalVariables()
{
	ProcessedSignalMvAvg[ComputingSensorTypeID][ComputingSensorID] = ProcessedSignalBuffer[ComputingSensorTypeID][ComputingSensorID].Average();
	ProcessedSignalRange[ComputingSensorTypeID][ComputingSensorID] = ProcessedSignalBuffer[ComputingSensorTypeID][ComputingSensorID].Max() - ProcessedSignalBuffer[ComputingSensorTypeID][ComputingSensorID].Min();
	ProcessedSignalStdDv[ComputingSensorTypeID][ComputingSensorID] = SignalProcessingLibrary::StandardDeviation(ProcessedSignalBuffer[ComputingSensorTypeID][ComputingSensorID]);
}

// Dynamical signal evaluation
bool SignalProcessing::CheckIsDynamicalPre()
{
	double StdDev = ProcessedSignalStdDv[Pre][ComputingSensorID];

	if (StdDev > K_StdDevThreshodl)
	{
		RatioRange = StdDev;// / ProcessedSignalRange[Pre][ComputingSensorID];
		RatioOutlier  = StdDev / std::abs(ProcessedSignalBuffer[Pre][ComputingSensorID].Last());
		RatioAvgRange = StdDev * StdDev / (ProcessedSignalRange[Pre][ComputingSensorID] * std::abs(ProcessedSignalMvAvg[Pre][ComputingSensorID]));
	}
	else
	{
		RatioRange    = 0.0;
		RatioOutlier  = 0.0;
		RatioAvgRange = 0.0;
	}

	IsDynamicalRange    = RatioRange   > K_IsDynamicRangeThresholdLowPre;
	IsDynamicalOutlier  = RatioOutlier > K_IsDynamicOutlierThresholdPre;
	IsDynamicalAvgRange = ((K_IsDynamicAvgRangeThresholdLowPre < RatioAvgRange) && (RatioAvgRange < K_IsDynamicAvgRangeThresholdHighPre));

	DBG_DynamicPre = IsDynamicalRange;

	return IsDynamicalRange;
}
bool SignalProcessing::CheckIsDynamicalAcc()
{
	double StdDev = ProcessedSignalStdDv[Acc][ComputingSensorID];

	if (StdDev > K_StdDevThreshodl)
	{
		RatioRange    = StdDev;
		RatioOutlier  = StdDev / std::abs(ProcessedSignalBuffer[Acc][ComputingSensorID].Last());
		RatioAvgRange = StdDev * StdDev / (ProcessedSignalRange[Acc][ComputingSensorID] * std::abs(ProcessedSignalMvAvg[Acc][ComputingSensorID]));
	}
	else
	{
		RatioRange    = 0.0;
		RatioOutlier  = 0.0;
		RatioAvgRange = 0.0;
	}

	IsDynamicalRange    = (K_IsDynamicRangeThresholdAcc       < RatioRange);
	IsDynamicalOutlier  = (K_IsDynamicOutlierThresholdAcc     < RatioOutlier);
	IsDynamicalAvgRange = (K_IsDynamicAvgRangeThresholdLowAcc < RatioAvgRange);// && (RatioAvgRange < K_IsDynamicAvgRangeThresholdHighAcc);

	DBG_DynamicAcc[ComputingSensorID] = IsDynamicalRange;

	return IsDynamicalRange ;
}

// Outliers
void   SignalProcessing::DetectOutliersInSignal(double &NewDataPoint, CircularBuffer<double> &buffer, const SensorType &SensorTypeID)
{
	double Last = buffer.Last();
	double Sigma = std::max(SignalProcessingLibrary::StandardDeviation(buffer), MinStdDv[SensorTypeID]);

	double Distance = NewDataPoint - Last;

	if (std::abs(Distance) > K_OutlierMaxDeviation * Sigma)	// is an outlier
	{
		NewDataPoint = Last + OutlierSuppressor.random_exp(K_OutlierSuppression / Sigma, Distance);
	}

	buffer.Add(NewDataPoint);
}
double SignalProcessing::DetectOutliersInCadence(const double &StartFrame, const double &EndFrame, const double &minCadence, const double &maxCadence)
{
	// to enable, change to true
	double newCadence = _SamplingFrequency / (EndFrame - StartFrame); // computes steps per sec for the single foot

	if (K_CadenceResistanceEnabled)
	{
		return std::min(maxCadence, std::max(minCadence, newCadence));
	}
	else
	{
		return newCadence;
	}
}

// Sync
void SignalProcessing::EvaluateSensorsInSync()
{
	for (int OtherSensorID = 0; OtherSensorID < nSensors[ComputingSensorTypeID]; OtherSensorID++)	//look at the other sensors
	{
		if (CheckSensor(SensorsInSyncMask[ComputingSensorTypeID], OtherSensorID)) // did any of the other sensors trigger?
		{
			if (CurrentStepCrossUpFrame[ComputingSensorTypeID][ComputingSensorID] - CurrentStepCrossUpFrame[ComputingSensorTypeID][OtherSensorID] > MaxFramesDelayForSyncTwo)
			{
				ToggleSensor(SensorsInSyncMask[ComputingSensorTypeID], OtherSensorID);
				FramesDelayForSyncAll[ComputingSensorTypeID] = 0;
			}
		}
	}
}
int  SignalProcessing::AverageInSyncTime ()
{
	double SensorsInAverage = 0;
	double SumOfFrames = 0;

	for (int SensorID = 0; SensorID < nSensors[ComputingSensorTypeID]; SensorID++)
	{
		if (CheckSensor(SensorsInSyncMask[ComputingSensorTypeID], SensorID))
		{
				SensorsInAverage++;
				SumOfFrames += CurrentStepCrossUpFrame[ComputingSensorTypeID][SensorID];
		}
	}
	return (int)std::rint(SumOfFrames / SensorsInAverage);
}
int  SignalProcessing::CountSensorsInSync()
{
	int TmpMask = (int)SensorsInSyncMask[ComputingSensorTypeID];
	int Count = 0;

	while (TmpMask)
	{
		Count += TmpMask & 1;
		TmpMask >>= 1;
	}
	return Count;
}
void SignalProcessing::ToggleSensor(SyncMask &Mask, const int &SensorID)
 {
	 Mask = (SyncMask)(Mask ^ (1 << SensorID));
 }
bool SignalProcessing::CheckSensor (SyncMask &Mask, const int &SensorID)
 {
	 return (Mask >> SensorID) & 1U;
 }

// Gait length
void SignalProcessing::IntegrateGaitLength()
{
	for (int SensorTypeID = 0; SensorTypeID < nStepsSensorsTypes; SensorTypeID++)
	{
		if (SensorTypeID == Pre)
			GaitRotatedAcc = (qStepReferenceOrientation[SensorTypeID].conj() * qOrientation).rotate(ProcessedSignalAcc);

		// Verlet integration x(t) = 2x(t-dt) - x(t-2dt) + a(t-dt) dt^2. NOTE: Has to be multiplied by K_GravityAcceleration
		FootPosition[SensorTypeID][2] = 2 * FootPosition[SensorTypeID][1] - FootPosition[SensorTypeID][0] + GaitRotatedAcc[K_GaitIntegrationDirection]/(_SamplingFrequency*_SamplingFrequency);
		FootPosition[SensorTypeID][0] = FootPosition[SensorTypeID][1];
		FootPosition[SensorTypeID][1] = FootPosition[SensorTypeID][2];
	}
}

// Activity Type
void SignalProcessing::EvaluateRestActivity()
{
	IsStanding = false;

	switch (ComputingSensorTypeID)
	{
	case Pre:
		for (int SensorID = 0; SensorID < nSensors[Pre]; SensorID++)
			IsStanding = IsStanding || (ProcessedSignalBuffer[Pre][SensorID].Average() > PressureSensorStandingThreshold[SensorID]);
	
		DetectedActivity[Pre] = IsStanding ? ActivityStanding : ActivitySitting;
		break;

	case Acc:
		// TODO add way to distinguish between seating and standing with pressure
		break;

	default:
		break;
	}
}

// Is Worn
void SignalProcessing::CheckIsDeviceWorn()
{
	switch (_GarmentType)
	{
	case OHI:
		WornFlag = IsAccDynamical;
		break;
	case OptimaBoot:
		WornFlag = IsPreAboveThreshold;
		break;
	default:
		WornFlag = false;
		break;
	}

	if (WornFlag)
	{
		nFramesNotWorn = 0;
		IsAccDynamical = false;
		IsPreAboveThreshold = false;

		if (!IsDeviceWorn && _objGaitEvents)
			_objGaitEvents->OnDeviceWornChanged(true);

		IsDeviceWorn = true;
	}
	else
	{
		nFramesNotWorn += nFramesToCheckWorn;
		if (nFramesNotWorn > nFramesToConsiderNotWorn)
		{
			if (IsDeviceWorn && _objGaitEvents)
				_objGaitEvents->OnDeviceWornChanged(false);

			IsDeviceWorn = false;
		}
	}
}

// Global Metrics
void SignalProcessing::SetGlobalMetrics()
{
	GlobalStepCount = StepCount[ComputingSensorTypeID];
	GlobalCadence = LocalCadenceTwoFeet[ComputingSensorTypeID];
	GlobalTimeOnGround = TimeOnGround[ComputingSensorTypeID];
	GlobalGaitLength = GaitLength[ComputingSensorTypeID];

	if (_objGaitEvents)
		_objGaitEvents->OnStepChanged(GlobalStepCount, GlobalCadence, GlobalTimeOnGround, GlobalGaitLength);
}

void SignalProcessing::SetGlobalActivity()
{
	if (GlobalDetectedActivity != DetectedActivity[ComputingSensorTypeID])
	{
		GlobalDetectedActivity = DetectedActivity[ComputingSensorTypeID];
		if (_objGaitEvents)
			_objGaitEvents->OnActivityTypeChanged(GlobalDetectedActivity);
	}
}

// Orientation
void SignalProcessing::CalculateOrientation()
{
	qOrientationVerlet   = qOldOrientation[0] + qOldOrientation[1] * qOmega / _SamplingFrequency;
	qOrientationVerlet.normalize();

	qDeltaOrientationMag = QuaternionFromVectors(ProcessedSignalMag, OldMag, qOmega.vector_part(), true);
		
	qOrientation = RiemannAvg(qOrientationVerlet, qOrientation * qDeltaOrientationMag, 0.0);

	fvector3d mag  = ProcessedSignalMag.get_normalized();
	fvector3d rmag = ReferenceMag.get_normalized();
	fvector3d omag = OldMag.get_normalized();
	fvector3d acc  = ProcessedSignalAcc.get_normalized();

	//std::cout << omag << rmag << std::endl;
	//std::cout << OldMag << ReferenceMag << std::endl;
	//std::cout << (qDeltaOrientationMag.rotate(mag) - omag).norm() << " " << ((qOrientation * qDeltaOrientationMag).rotate(mag) - rmag).norm() << std::endl;
	int a = 0;

	if (std::abs(ProcessedSignalAcc.norm() - GravityNorm) < K_MaxGravityNormDeviation)
	{
		PredictedGravityOrientation = qOrientation.rotate(ProcessedSignalAcc).get_normalized();
		a = 1;
		GravityAngularDeviation = std::acos(PredictedGravityOrientation * ReferenceGravity);

		if (GravityAngularDeviation < K_MaxGravityAngularDeviation)
		{
			a = 2;
			// Perfectly align Magnetometer
			qOrientationMag = QuaternionFromVectors(ProcessedSignalMag, ReferenceMag);
			qOrientationAcc = QuaternionFromVectors(qOrientationMag.rotate(ProcessedSignalAcc), ReferenceGravity, ReferenceMag, true);
			qOrientationMagAcc = qOrientationAcc * qOrientationMag;

			// Perfectly align Accelerometer
			qOrientationAcc = QuaternionFromVectors(ProcessedSignalAcc, ReferenceGravity);
			qOrientationMag = QuaternionFromVectors(qOrientationAcc.rotate(ProcessedSignalMag), ReferenceMag, ReferenceGravity, true);
			qOrientationAccMag = qOrientationMag * qOrientationAcc;

			qOrientationAll = RiemannAvg(qOrientationMagAcc, qOrientationAccMag, 0.0);
			
			/*std::cout << FramesProcessedCounter << " " 
				<< (qOrientationMagAcc.rotate(mag) - rmag).norm() << " " 
				<< (qOrientationAccMag.rotate(mag) - rmag).norm() << " " 
				<< (qOrientationMagAcc.rotate(acc) - ReferenceGravity).norm() << " "
				<< (qOrientationAccMag.rotate(acc) - ReferenceGravity).norm() << " " 
				<< (qOrientation.rotate(mag) - rmag).norm() << " " 
				<< (qOrientation.rotate(acc) - ReferenceGravity).norm() 
				<< std::endl;*/

			/*std::cout << FramesProcessedCounter << " "
				<< (qOrientationMagAcc.rotate(mag) * rmag) << " "
				<< (qOrientationAccMag.rotate(mag) * rmag) << " "
				<< (qOrientationMagAcc.rotate(acc) * ReferenceGravity) << " "
				<< (qOrientationAccMag.rotate(acc) * ReferenceGravity) << " "
				<< (qOrientationAll.rotate(mag) * rmag) << " "
				<< (qOrientationAll.rotate(acc) * ReferenceGravity)
				<< std::endl;*/

			//if (qOrientationAll.rotate(mag) * rmag * 180.0/PI < 12)
				qOrientation =  RiemannAvg(qOrientation, qOrientationAll, 1.0);
		}
	}
				std::cout << (qOrientation.rotate(acc) - ReferenceGravity.get_normalized()).norm() << " " << (qOrientation.rotate(mag) - rmag).norm() << std::endl;

	//std::cout << FramesProcessedCounter << " " << (qOrientation.rotate(mag) - rmag).norm() << "" << mag << acc << rmag << ReferenceGravity << std::endl;
	
	qOrientation.normalize();

	// Set Orientation history
	qOldOrientation[0] = qOldOrientation[1];
	qOldOrientation[1] = qOrientation;

	OldMag = ProcessedSignalMag;
	qOmega = ProcessedSignalGyr * K_GyrConversion;
}

// Switch SensorTypeToUse
void SignalProcessing::SwitchSensorTypeToUse(const SensorType &NewSensorTypeID)
{
	StepCount[NewSensorTypeID] = StepCount[SensorTypeToUse];
	SensorTypeToUse = NewSensorTypeID;
	if (_objGaitEvents)
		_objGaitEvents->OnSensorTypeChanged(NewSensorTypeID);
}

// Debug
void SignalProcessing::ResetFlags()
{
	for (int SensorTypeID = 0; SensorTypeID < nStepsSensorsTypes; SensorTypeID++)		// cycle over sensor types
	{
		for (int SensorID = 0; SensorID < nSensors[SensorTypeID]; SensorID++)	// cycle over single sensors
		{
			DBG_StepFlag[SensorTypeID][SensorID] = Nothing;
		}
	}
}

// TODO Add flattening check for pressure sensor(s)