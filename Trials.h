#ifndef __Trials_H__
#define __Trials_H__

//#include "ParallelPort\CPaPiC.h"
#include "stdafx.h"
#include "dmccom.h"
#include "WriteToExternalFile.h"
#include "DecideFeedback.h"
#include "Robot.h"
#include "Drawings.h"
#include "ForceProgram.h"
#include "Target.h"
#include "Cursor.h"
#include <vector>
#include <fstream>
using namespace std;

class Trial
{
public:
	Trial(Robot& robot, Drawings& draw, forceProgram& forces, const vector<TwoDoubles>& pointLocations, const vector<TwoDoubles>& numLocations);
	~Trial();
	void loadBehavioralParameters(const ParameterList& behaviorParams, const ParameterList& blockParams);
	void loadTrialParameters(const ParameterVectorList& trialParams, int trialNum);

	void processTrial(Drawings& draw);
	bool trialFinished() { return (currentStage == trialDone); }
	void startTrial();



	const ParameterVectorList& savedTrialData() { return saveDataTimeStep; }
	const ParameterList& savedTrialStatistics() { return saveDataTrial; }

private:
	Robot& _robot;
	Drawings& _draw;
	forceProgram& _forces;
	const vector<TwoDoubles>& _pointLocations;
	const vector<TwoDoubles>& _numLocations;

	enum Stage
	{
		trialStart, 
		outsideOriginMode,
		insideOriginMode,
		reactionMode,
		targetMode,
		dampMode,
		setFeedbackMode,
		showFeedbackMode,
		pushBackMode,
		trialDone,
		rewardFeedback,
		showTotalScore
	};

	bool warningBeep;
	long zeroTime; // This is the time of the beginning of the trial
	double delayTimer;
	long robotTime;
	TwoDoubles robotPosition;
	TwoDoubles robotVelocity;
	bool robotMoving;

	Stage currentStage;

	ParameterVectorList saveDataTimeStep;
	ParameterList saveDataTrial;
};

class trialList
{
public:
	trialList();
	~trialList();

	void loadBehaviorParameters(const string& fileName);
	void loadPointLocations();
	void loadNumLocations();
	void loadTrials(const string& ID, const string& day, int blockNumber);
	
	//In-Out methods
	int getNumberOfTrials() {return _numberOfTrials;}
	int getNumberOfPoints() {return ((int) _pointLocations.size());}
	int getNumberOfNums() {return ((int) _numLocations.size());}
	int getBlockNumber() {return _blockNumber;}
	
	const ParameterList& getBehaviorParams() { return _behaviorParams; }
	const vector<TwoDoubles>& getPointLocations() { return _pointLocations; }
	const vector<TwoDoubles>& getNumLocations() { return _numLocations; }
	const ParameterVectorList& getTrialParams() { return _trialParamsList; }
	const ParameterList& getBlockInfo() { return _blockInfo; }
private:
	vector<TwoDoubles> _pointLocations;		//position in Pixels of targets\origins.
	vector<TwoDoubles> _numLocations;	
	ParameterList _behaviorParams;
	ParameterList _blockInfo; // Parameters for the current block
	ParameterVectorList _trialParamsList; // This data list will start counting from 1

	int _numberOfTrials;
	int _blockNumber;
	int _numberOfPoints;

	string _subjectID;
	string _subjectDay;
};

#endif
