#include "stdafx.h"
#include <math.h>
#include "Resource.h"
#include "Robot.h"
#include "ForceProgram.h"
#include "ErrorHandler.h"
#include "Trials.h"
#include "Drawings.h"
#include "Target.h"
#include "Cursor.h"
#include "definition.h"
#include "Parameters.h"
#include "KalmanFilter.h"
#include "GlobalStuff.h"
#include "ParallelPort\CPaPiC.h"
/*GM 19-04-17
	Step 1: uncomment or add the previous line.
	Step 2: add CPaPic.h to the header files and add CPaPiC.cpp to the source files
	Step 3: add the nawxcwd.lib library (right click on GUI, properties, linker, input, add it to the line with libraries)
	Step 4: uncomment the two lines about the parallel port right after **start trial**
	Step 5: go to the desired trial phase (here: targetMode) and uncomment
		CPaPiC papi
		pin_output_mode & set_pin for the desired pin
*/
#include <fstream>
#include <sstream>
#include <exception>
#include <vector>
#define M_PI 3.14159265358979323846 /* pi */
using namespace std;

#include "mmsystem.h"	//for playing sounds
#pragma comment( lib, "Winmm.lib" )	//add the necessary .lib file

Trial::Trial(Robot& robot, Drawings& draw, forceProgram& forces, const vector<TwoDoubles>& pointLocations, const vector<TwoDoubles>& numLocations)
	: _robot(robot), _draw(draw), _forces(forces), _pointLocations(pointLocations), _numLocations(numLocations)
{
}

Trial::~Trial()
{}

// Initialize all variables for the Behavior Parameters
double movementSpeedThreshold;
double originRadius;
double originHitRadius;
Drawings::colorType originColor;
Drawings::colorType originColorReach;
double endpointRadius;
Drawings::colorType endpointColor;
double shoulderX;
double shoulderY;
double upperArmLength;
double foreArmLength;
double targetDrawRadius;
double targetHitRadius;
Drawings::colorType targetGoColor;
double displayCursorDistance;
double displayCursorDistanceTarget;
Drawings::colorType successColor;
Drawings::colorType slowColor;
Drawings::colorType fastColor;
Drawings::colorType missColor;
Drawings::colorType itiColor;
double trialFeedbackTime;
double maxPushToOriginTime;
double showOriginRadius;
double trialTime;
double trialWindow;
double maxTrialTime;
double previewCircleTime;
double previewTargetTime;
double originHoldTime;
double maxReactionTime;
double dampingTime;
double rewardTime;
double rewardScoreTime;
double maxPushbackTime;
double breakThreshold;
double itiThreshold;
int	playSuccessSound;
int	playMissSound;
int	playFastSound;
int	playSlowSound;
string	successSoundFile;
string	missSoundFile;
string	fastSoundFile;
string	slowSoundFile;
Drawings::colorType positionFeedback;
TwoDoubles savePosition;

// Load all Behavior Parameters from the .dat file
void Trial::loadBehavioralParameters(const ParameterList& behaviorParams, const ParameterList& blockParams)
{
	movementSpeedThreshold = behaviorParams["MovementSpeedThreshold[cm/ms]"].getValueD();
	originRadius = behaviorParams["OriginRadius[cm]"].getValueD();
	originHitRadius = behaviorParams["OriginHitRadius[cm]"].getValueD();
	originColor = Drawings::char2Color(behaviorParams["OriginColor"].getValueC());
	endpointRadius = behaviorParams["EndpointRadius[cm]"].getValueD();
	endpointColor = Drawings::char2Color(behaviorParams["EndpointColor"].getValueC());
	positionFeedback = Drawings::char2Color(behaviorParams["PositionFeedback"].getValueC());
	shoulderX = behaviorParams["ShoulderX[cm]"].getValueD();
	shoulderY = behaviorParams["ShoulderY[cm]"].getValueD();
	upperArmLength = behaviorParams["UpperArmLength[cm]"].getValueD();
	foreArmLength = behaviorParams["ForeArmLength[cm]"].getValueD();
	targetDrawRadius = behaviorParams["TargetDrawRadius[cm]"].getValueD();
	targetHitRadius = behaviorParams["TargetHitRadius[cm]"].getValueD();
	targetGoColor = Drawings::char2Color(behaviorParams["TargetGoColor"].getValueC());
	displayCursorDistance = behaviorParams["DisplayCursorDistance[cm]"].getValueD();
	displayCursorDistanceTarget = behaviorParams["DisplayCursorDistanceTarget[cm]"].getValueD();
	itiColor = Drawings::char2Color(behaviorParams["TargetITIColor"].getValueC());
	trialFeedbackTime = behaviorParams["TrialFeedbackTime[ms]"].getValueD();
	maxPushToOriginTime = behaviorParams["MaxPushToOriginTime[ms]"].getValueD();
	showOriginRadius = behaviorParams["ShowOriginRadius[cm]"].getValueD();
	trialTime = behaviorParams["TrialTime[ms]"].getValueD();
	trialWindow = behaviorParams["TrialWindow[ms]"].getValueD();
	maxTrialTime = behaviorParams["MaxTrialTime[ms]"].getValueD();
	previewCircleTime = behaviorParams["PreviewCircleTime[ms]"].getValueD();
	previewTargetTime = behaviorParams["PreviewTargetTime[ms]"].getValueD();
	maxReactionTime = behaviorParams["MaxReactionTime[ms]"].getValueD();
	dampingTime = behaviorParams["DampingTime[ms]"].getValueD();
	rewardTime = behaviorParams["RewardTime[ms]"].getValueD();
	rewardScoreTime = behaviorParams["RewardTime[ms]"].getValueD();
	maxPushbackTime = behaviorParams["MaxPushbackTime[ms]"].getValueD();
	breakThreshold = behaviorParams["BreakThreshold[ms]"].getValueD();
	itiThreshold = behaviorParams["ITIThreshold[ms]"].getValueD();
	playSuccessSound = behaviorParams["PlaySuccessSound"].getValueI();
	playMissSound = behaviorParams["PlayMissSound"].getValueI();
	playFastSound = behaviorParams["PlayFastSound"].getValueI();
	playSlowSound = behaviorParams["PlaySlowSound"].getValueI();
	successSoundFile = behaviorParams["SuccessSoundFile"].getValueS();
	missSoundFile = behaviorParams["MissSoundFile"].getValueS();
	fastSoundFile = behaviorParams["FastSoundFile"].getValueS();
	slowSoundFile = behaviorParams["SlowSoundFile"].getValueS();

	// Params we get from block inputFiles and not behaviorParams.dat
	successColor = Drawings::char2Color(blockParams["TargetSuccessColor"].getValueC());
	slowColor = Drawings::char2Color(blockParams["TargetSlowColor"].getValueC());
	fastColor = Drawings::char2Color(blockParams["TargetFastColor"].getValueC());
	missColor = Drawings::char2Color(blockParams["TargetMissColor"].getValueC());
}

// Initialize the variables for Trial Parameters
int originPoint;
int visualPerturbationType;
int showCursor;
int showTarget;
int showTargetColor;
int reward;
int showEndpointError;
int timing;
double rotation;
int targetPoint;
int forceType;
double forceGain;
double ITI;
int nextOriginPoint;
TwoDoubles originLocation;
TwoDoubles targetLocation;
TwoDoubles nextOriginLocation;
vector<TwoDoubles> numberLocations;
vector<int> numSigns;
bool cursorUpdate = true;
bool reachedTarget = false;
bool semiCircle = false;

// Variables for trial statistics
bool succesfulTrial;
double trialMovementTime;
double reactionTime;
double totalTrialMovementTime;
bool saveData = false;
double trialWindowNext;
int score = 0;
int scoreDelta = 0;




// Load the Trial Parameters from input .dat files
void Trial::loadTrialParameters(const ParameterVectorList& trialParams, int trialNum)
{
	originPoint = trialParams(trialNum, "Origin").getValueI();
	visualPerturbationType = trialParams(trialNum, "VisualPerturbationType").getValueI();
	showCursor = trialParams(trialNum, "showCursor").getValueI();
	showTarget = trialParams(trialNum, "showTarget").getValueI();
	showTargetColor = trialParams(trialNum, "showTargetColor").getValueI();
	showEndpointError = trialParams(trialNum, "showEndpointError").getValueI();
	timing = trialParams(trialNum, "timing").getValueI();
	reward = trialParams(trialNum, "reward").getValueI();
	rotation = trialParams(trialNum, "VisualRotationDeg").getValueD();
	targetPoint = trialParams(trialNum, "Target").getValueI();
	forceType = trialParams(trialNum, "forceType").getValueI();
	forceGain = trialParams(trialNum, "forceGain").getValueD();
	ITI = trialParams(trialNum, "interTrialInterval").getValueD();
	nextOriginPoint = trialParams(trialNum, "NextOrigin").getValueI();  // This trials origin

	originLocation = _pointLocations[originPoint-1];
	targetLocation = _pointLocations[targetPoint-1];
	nextOriginLocation = _pointLocations[nextOriginPoint-1];
	rotation = (rotation*(M_PI/180));
	numberLocations = _numLocations;
}

// Sets the output variables for saving data
const int numOutputDataVars = 8;
char *outputDataNames[numOutputDataVars] = {	"Time", "PosX", "PosY",	"VelX",	"VelY",	"ForceX", "ForceY",	"Stage" };
char outputDataTypes[numOutputDataVars] = { 'i', 'd', 'd', 'd', 'd', 'd', 'd', 'i' };

Parameter outputData[numOutputDataVars] = {
	Parameter((int) 0), Parameter((double) 0.0), Parameter((double) 0.0), Parameter((double) 0.0),	Parameter((double) 0.0),
	Parameter((double) 0.0), Parameter((double) 0.0), Parameter((int) 0) };


	/****************************************************************
	Start Trial
	****************************************************************/
	void Trial::startTrial() {
		currentStage = trialStart;
		CPaPiC papi; // Parallel port pin class object. GM 19-04-17 Step 4: uncomment
		papi.clear_pin(LP_OUTPUT_PINS); // clear all pins. GM 19-04-17 Step 4: uncomment

		// Generate a ParameterListVector for storing the data from the trial
		vector<string> dataNames;
		vector<char> dataTypes;
		dataNames.assign(outputDataNames, outputDataNames+numOutputDataVars);
		dataTypes.assign(outputDataTypes, outputDataTypes+numOutputDataVars);
		saveDataTimeStep.createEmpty( dataNames, dataTypes );

		// Generate a 0 time for the data
		//    Actually the 0 time is found when we first start running 
		//    processTrial(), but here we reset it so that it is 'refound
		zeroTime = -1;

		saveDataTrial.clear();
	}

	// The main routine of the Trial program
	void Trial::processTrial(Drawings& draw) {
		static double delayTimer = 0.0;
		static double trialMovementTime = 0.0;
		static double dampTime = 0.0;
		static Target target;
		static Target origin;
		static Cursor cursor;
		static TwoDoubles lastCursorPosition;
		static Cursor cursorReturn;

		TwoDoubles robotPosition;
		if (!_robot.getPosition(robotPosition)) {
			return;
		} else {
			robotPosition = robotPosition + _robot.getCorrection();
			if (!cursorUpdate) {
				// no update of cursor position
				cursor.setPosition(savePosition);
				cursorReturn.setPosition(robotPosition);
			} else {
				cursor.setPosition(robotPosition);
				cursorReturn.setPosition(robotPosition);
			}
		}
		TwoDoubles robotVelocity;
		if (!_robot.getVelocity(robotVelocity)) {
			return;
		}
		bool robotMoving = robotVelocity.magnitude() >= movementSpeedThreshold;
		TwoDoubles robotForce;
		if (!_robot.getForce(robotForce)) {
			return;
		}

		// We need an 'old time' but we have to make sure it is valid coming from the robot
		static long storedTime = -1;
		if (storedTime <= 0 || zeroTime <= 0) {
			long tempTime;
			if (_robot.getTimeMs(tempTime)) {
				storedTime = tempTime;
				if (zeroTime <= 0) {
					zeroTime = tempTime;
					lastCursorPosition = cursor.getDisplayedPosition(); // Set the last cursor position to current at the beginning of each trial
				}
			}
			return;
		}
		if (!_robot.getTimeMs(robotTime)) {
			return;
		}
		double deltaTime = (double) (robotTime - storedTime);
		storedTime = robotTime;

		// 14-04-2016 ZDJ For PreviewTrials Charlotte
		if (semiCircle) {
		// Draw semicircle to guide subjects
			_draw.drawSemiCircle(originLocation, 10, endpointColor);
			for(int i = 0; i<numSigns.size();i++) 
				{
				_draw.drawNumbers(numSigns[i], numberLocations[i]);
				}
	    }

		// Switch of currentStage that "runs" all trials
		
		/*GM 19-04-17
		At desired trial, uncomment 'CPaPic papi'
		then make sure the lines
			//papi.pin_output_mode(LP_PIN02);
			//papi.set_pin(LP_PIN02);
		(or whichever pin is used) is uncommented
		*/
		
		switch(currentStage)
		{
		case trialStart:
			{
				// trigger signal
				// ZEB -> omzetten naar aparte functie met getal als input
				//CPaPiC papi; // Parallel port pin class object
				//papi.pin_output_mode(LP_PIN03);
				//papi.set_pin(LP_PIN03);
				
				// Save trial characteristics to output file
				saveDataTrial.addParam("OriginX", Parameter(originLocation.x));
				saveDataTrial.addParam("OriginY", Parameter(originLocation.y));
				saveDataTrial.addParam("TargetX", Parameter(targetLocation.x));
				saveDataTrial.addParam("TargetY", Parameter(targetLocation.y));
				saveDataTrial.addParam("visualPerturbationType", Parameter(visualPerturbationType));
				saveDataTrial.addParam("forceType", Parameter(forceType));
				saveDataTrial.addParam("showCursor", Parameter(showCursor));
				saveDataTrial.addParam("showTarget", Parameter(showTarget));
				saveDataTrial.addParam("showTargetColor", Parameter(showTarget));
				saveDataTrial.addParam("rotation", Parameter(rotation));

				// Draw origin and cursor
				origin.setPosition(originLocation);
				origin.setDrawParameters(originRadius, originColor);
				origin.on();
				cursor.setDrawParameters(endpointRadius, endpointColor);
				cursor.on();
				currentStage = outsideOriginMode;

		
			}
		case outsideOriginMode:
			{
				// trigger signal
				//CPaPiC papi; // Parallel port pin class object
				//papi.pin_output_mode(LP_PIN02);
				//papi.clear_pin(LP_PIN02);
				//papi.pin_output_mode(LP_PIN03);
				//papi.set_pin(LP_PIN03);

				// Check whether cursor is inside the origin
				if(origin.checkHit(cursor.getDisplayedPosition())){
					delayTimer = 0.0;
					deltaTime = 0.0;
					succesfulTrial = false;
					warningBeep = true;
					currentStage = insideOriginMode;
					// position
				}
				break;
			}
		case insideOriginMode:
			{
				// trigger signal
				//CPaPiC papi; // Parallel port pin class object
				//papi.pin_output_mode(LP_PIN02);
				//papi.set_pin(LP_PIN02);
				//papi.pin_output_mode(LP_PIN03);
				//papi.set_pin(LP_PIN03);

				origin.setColor(originColorReach);
				// Let time pass independent of cursor being in the origin
				delayTimer = delayTimer + deltaTime;
				// When the ITI is 'reasonably long' play beep 1500ms before trialstart
				if(ITI > itiThreshold) {
					// Start pushing
					_forces.startPush(originLocation);
					// Play a beep 1500ms before trialstart
					if (warningBeep && (delayTimer > (ITI-1500.0))) {
						PlaySound(TEXT("beep-8.wav"), NULL, SND_FILENAME | SND_ASYNC);
						warningBeep = false;
					}
				}

				// When the ITI is very long ('set break') change origin color and play beep 1000ms before trialstart
				if(ITI > breakThreshold) {
					origin.setColor(itiColor);
					_forces.startPush(originLocation);
					// Play a beep 1500ms before trialstart
					if (warningBeep && (delayTimer > (ITI-1500.0))) {
						PlaySound(TEXT("beep-8.wav"), NULL, SND_FILENAME | SND_ASYNC);
						warningBeep = false;
					}
				}

				// 14-04-2016 ZDJ For PreviewTrials Charlotte
				if (!showTargetColor) {
					if (delayTimer > ITI-previewCircleTime-previewTargetTime && delayTimer < ITI-previewCircleTime){
						target.on();
					}else if (delayTimer > ITI - previewCircleTime && delayTimer < ITI){
						semiCircle = true;
						target.off();
					} else {
					semiCircle = false;
					}
				}

				if (delayTimer > ITI){
					// In case we had a set break we have to redraw origin
					semiCircle = false;
					origin.setDrawParameters(originRadius, originColor);
					target.setPosition(targetLocation);
					target.setDrawParameters(targetDrawRadius, targetGoColor);
					target.on();

					// Show Cursor or not
					if (!showCursor) {
						cursor.off();
					}

					// ZDJ tryout cursor other color in loc trial


					delayTimer = 0.0;
					deltaTime = 0.0;
					PlaySound(NULL, NULL, 0); // Stop any sounds
					_forces.stopPush();
					saveData = true;
					currentStage = reactionMode;
					
				} // end of ITI if
				break;
			}

		
		case reactionMode:
			// Reaction = when robotVelocity gets above a certain treshold
			{
				// trigger signal
				//CPaPiC papi; // Parallel port pin class object
				//papi.pin_output_mode(LP_PIN02);
				//papi.set_pin(LP_PIN02);
				//papi.pin_output_mode(LP_PIN03);
				//papi.set_pin(LP_PIN03);
				//papi.pin_output_mode(LP_PIN04);
				//papi.set_pin(LP_PIN04);

				// Added by Charlotte 13-04-2017
				//semiCircle = false;
				target.on();

				origin.setColor(originColorReach);
				delayTimer = delayTimer + deltaTime;
				// Save the time it takes for subject to react to drawing of target
				if(robotMoving || delayTimer > maxReactionTime){
					reactionTime = delayTimer;
					saveDataTrial.addParam("reactionTime", Parameter(reactionTime));

					// origin.off();

					// Generate perturbation after reacion
					// Generate viscousField
					if (forceType == 1) {
						double matVals[] = {0, forceGain, -forceGain, 0};
						_forces.startViscous(Matrix(2,2, matVals));
					}
					// Generate forceChannel
					if (forceType == 2) {
						_forces.startChannel(robotPosition, targetLocation);
						cursor.clampError(robotPosition, targetLocation);
					}
					// Generate visualPerturbation
					if (visualPerturbationType == 1) {
						cursor.rotateDisplay(robotPosition, rotation);
					}
					// Generate visualChannel
					if (visualPerturbationType == 2) {
						cursor.clampError(robotPosition, targetLocation);
					}

					currentStage = targetMode;			

				}

				break;
			}
		case targetMode:
			// After reaction from the subject, when he/she is moving towards the target
			{
				// trigger signal (#ZDJ, this is when movement starts)
				/* GM 19-04-17 Step 5
				In this version, pin 2 gives a high(er) signal when the target appears				
				*/
				CPaPiC papi; // Parallel port pin class object
				papi.pin_output_mode(LP_PIN02);
				papi.set_pin(LP_PIN02);
				//papi.pin_output_mode(LP_PIN03);
				//papi.set_pin(LP_PIN03);
				//papi.pin_output_mode(LP_PIN04);
				//papi.set_pin(LP_PIN04);

				origin.setColor(originColorReach);
				delayTimer = delayTimer + deltaTime;
				// Check if the cursor is outside an imaginary circle with radius equal to the distance between the origin and the target
				TwoDoubles radius = originLocation - targetLocation;
				double radiusMag = radius.magnitude();
				double radiusMagTest = radiusMag + displayCursorDistanceTarget;

				// Check if we hit the target
				if(target.checkHit(cursor.getDisplayedPosition())) {
					succesfulTrial = true;
				}

				if(!(origin.checkHit(cursor.getDisplayedPosition(), radiusMagTest )) || (delayTimer >= maxTrialTime && timing)) {
					// Get trialTime
					trialMovementTime = delayTimer;

					// Turn off forces
					_forces.stopChannel();
					_forces.stopViscous();

					if (!showEndpointError) {
						cursor.off();
					} else {
						cursor.on();

					}

					cursor.setDrawParameters(endpointRadius, positionFeedback);
			
					cursorUpdate = false;
					if (delayTimer < maxTrialTime){
						TwoDoubles differenceVector = robotPosition - originLocation;
						savePosition = 10*((robotPosition-originLocation)/differenceVector.magnitude()) + originLocation;
					} else {
						savePosition = robotPosition;
					}

					// Save the time and success parameters
					saveDataTrial.addParam("trialMovementTime", Parameter(trialMovementTime));
					saveDataTrial.addParam("Success", Parameter(succesfulTrial));

					// Commented damping out, since this might give proprioceptive feedback on the location of your hand
					//_forces.startDamping();
					//cursor.off();
					delayTimer = 0.0;
					deltaTime = 0.0;
					currentStage = setFeedbackMode;
				}
				break;
			}

		case dampMode:
			{
				// trigger signal
				//CPaPiC papi; // Parallel port pin class object
				//papi.pin_output_mode(LP_PIN02);
				//papi.set_pin(LP_PIN02);
				//papi.pin_output_mode(LP_PIN03);
				//papi.set_pin(LP_PIN03);
				//papi.pin_output_mode(LP_PIN04);
				//papi.set_pin(LP_PIN04);

				delayTimer = delayTimer + deltaTime;
				if (delayTimer > dampingTime) {

					//Make sure all forces are off
					_forces.stopDamping();
					delayTimer = 0.0;
					deltaTime = 0.0;

					if(reward){
						currentStage = rewardFeedback;
					} else {
						trialWindow = trialWindowNext;
						currentStage = pushBackMode;
						_forces.startPush(nextOriginLocation);
					}
				}
				break;
			}
		case rewardFeedback:
			{
				// trigger signal (#ZDJ, this is when reward is shown)
				//CPaPiC papi; // Parallel port pin class object
				//papi.pin_output_mode(LP_PIN02);
				//papi.clear_pin(LP_PIN02);
				//papi.pin_output_mode(LP_PIN03);
				//papi.clear_pin(LP_PIN03);
				//papi.pin_output_mode(LP_PIN04);
				//papi.clear_pin(LP_PIN04);
				//papi.pin_output_mode(LP_PIN05);
				//papi.set_pin(LP_PIN05);

				delayTimer = delayTimer + deltaTime;
				scoreDelta = 0;
				score = score+scoreDelta;
				
				// ZEB -> positie beloning
				TwoDoubles differenceVector = targetLocation - originLocation;
				TwoDoubles drawLocation = 8.5*(differenceVector/differenceVector.magnitude()) + originLocation;
				_draw.drawNumbers(scoreDelta, drawLocation);
			
				

				if (delayTimer > rewardTime) {
					trialWindow = trialWindowNext;
					delayTimer = 0.0;
					deltaTime = 0.0;
					currentStage = pushBackMode;
					_forces.startPush(nextOriginLocation);
				}
				break;
			}
		case setFeedbackMode:
			{
				// trigger signal (#ZDJ, this is when target is hit)
				//CPaPiC papi; // Parallel port pin class object
				//papi.pin_output_mode(LP_PIN02);
				//papi.clear_pin(LP_PIN02);
				//papi.pin_output_mode(LP_PIN03);
				//papi.set_pin(LP_PIN03);
				//papi.pin_output_mode(LP_PIN04);
				//papi.set_pin(LP_PIN04);
				
				saveData = false;
				origin.setColor(originColor);
				delayTimer = delayTimer + deltaTime;
				saveDataTrial.addParam("trialWindow", Parameter(trialWindow));
				// Assume not playing any sound
				bool playSound = false;
				string soundFile = "";
				
				// Set the color of the target to reflect the success level
				// First check for movement time, then check for target hit

				// ZDJ add no target in no vision trial during localisation 
				if (!showTarget) {
						target.off();
						cursor.setColor(targetGoColor);
				}
				// ZDJ add no target color in loc trial after localisation
				else if (!showTargetColor) {
						target.setColor(targetGoColor);
						cursor.setColor(endpointColor);
				}
				// Speed OK && Target hit
				else if((trialMovementTime >= (trialTime - trialWindow)) && (trialMovementTime <= (trialTime + trialWindow))&& succesfulTrial)
				{
					trialWindowNext = (trialWindow * 1);
					target.setColor(successColor);
					cursor.setColor(endpointColor);
					target.on();
					if (playSuccessSound == 1) {
						playSound = true;
						soundFile = successSoundFile;
					}
				}
				// Speed OK && Target miss
				// In my case I don't care if you hit the target or not, just the speed matters (set missColor to be the same in behavParameters.dat)
				else if((trialMovementTime >= (trialTime - trialWindow)) && (trialMovementTime <= (trialTime + trialWindow)) && !succesfulTrial)
				{
					trialWindowNext = (trialWindow * 1);
					target.setColor(missColor);
					cursor.setColor(endpointColor);
					target.on();
					if (playMissSound == 1) {
						playSound = true;
						soundFile = missSoundFile;
					}
				}

				// Too slow
				else if(trialMovementTime > (trialTime + trialWindow))
				{
					trialWindowNext = (trialWindow * 1);
					target.setColor(slowColor);
					cursor.setColor(endpointColor);
					target.on();
					if (playSlowSound == 1) {
						playSound = true;
						soundFile = slowSoundFile;
					}
				}

				// Too fast
				else if(trialMovementTime < (trialTime - trialWindow))
				{
					trialWindowNext = (trialWindow * 1);
					target.setColor(fastColor);
					cursor.setColor(endpointColor);
					target.on();
					origin.setColor(fastColor);
					if (playFastSound == 1) {
						playSound = true;
						soundFile = fastSoundFile;
					}
				}

				else
				{
					handleError("Wrong feedback number!", "doneReachMode");
				}

				if (playSound == true) {
					PlaySound(soundFile.c_str(), NULL, SND_FILENAME | SND_ASYNC);
				}

				currentStage = dampMode;
				break;
			}
				

		case pushBackMode:
			{
				// trigger signal
				//CPaPiC papi; // Parallel port pin class object
				//papi.pin_output_mode(LP_PIN02);
				//papi.set_pin(LP_PIN02);
				//papi.pin_output_mode(LP_PIN05);
				//papi.set_pin(LP_PIN05);
				
				// Set next origin location
				origin.setPosition(nextOriginLocation);
				origin.on();
				
				// Wait for reaching next origin
				if(cursorReturn.checkHit(nextOriginLocation, displayCursorDistance)) {
						cursorUpdate = true;
						target.off();
						semiCircle = false;
						cursor.setPosition(robotPosition);
						cursorReturn.setPosition(robotPosition);
						
						// give cursor another color in localisation trial
					
						cursor.setColor(targetGoColor);
						
					
						// Turn off cursor perturbation 
						cursor.rotateOff();
						cursor.clampErrorOff();

						// Turn off all forces

						_forces.stopDamping();
						_forces.stopChannel();
						_forces.stopViscous();
						_forces.stopPush();

						if(reward){
							currentStage = showTotalScore;
						} else {
							currentStage = trialDone;
						}
					
				}
				break;
			}

		case showTotalScore:
			{
				// trigger signal
				//CPaPiC papi; // Parallel port pin class object
				//papi.pin_output_mode(LP_PIN02);
				//papi.clear_pin(LP_PIN02);
				//papi.pin_output_mode(LP_PIN03);
				//papi.set_pin(LP_PIN03);
				//papi.pin_output_mode(LP_PIN05);
				//papi.set_pin(LP_PIN05);

				TwoDoubles drawLocation = originLocation;
				drawLocation.x = drawLocation.x + 2;
				_draw.drawNumbers(score, drawLocation);
				delayTimer = delayTimer + deltaTime;

				if (delayTimer > rewardScoreTime) {
					currentStage = trialDone;
					delayTimer = 0;
					deltaTime = 0;
				}
				break;
			}
		case trialDone:	
			// The end case of the trial
			{
				// trigger signal
				//CPaPiC papi; // Parallel port pin class object
				//papi.pin_output_mode(LP_PIN03);
				//papi.clear_pin(LP_PIN03);
				//papi.pin_output_mode(LP_PIN05);
				//papi.clear_pin(LP_PIN05);
				//papi.pin_output_mode(LP_PIN09);
				//papi.set_pin(LP_PIN09);

				break;
			}
		}//switch
		
		// save Data each timestep we loop over processTrial function
		if (saveData) {
		outputData[0].setValue((robotTime - zeroTime));
		outputData[1].setValue(robotPosition.x);
		outputData[2].setValue(robotPosition.y);
		outputData[3].setValue(robotVelocity.x);
		outputData[4].setValue(robotVelocity.y);
		outputData[5].setValue(robotForce.x);
		outputData[6].setValue(robotForce.y);
		outputData[7].setValue(currentStage);
		saveDataTimeStep.addData(outputData);
		}

		// Draw target, origin and cursor on screen
		target.draw(_draw);
		origin.draw(_draw);
		cursor.draw(_draw);


		// Update lastCursorPosition to current (so it will be good for next time)
		lastCursorPosition = cursor.getDisplayedPosition();
	}

	
	trialList::trialList()
		:
	_numberOfTrials(0),
		_blockNumber(0),
		_trialParamsList(1) // Start counting from trial 1
	{}


	trialList::~trialList()
	{}

	// Get the location of the behaviorParameters.dat
	void trialList::loadBehaviorParameters(const string& fileName)
	{
		_behaviorParams.read(fileName);
	}

	// Get the positions of the different origins/targets.
	void trialList::loadPointLocations()	
	{
		string fileName = _behaviorParams["PointLocationFile"].getValueS();

		ifstream pointsLocationfile(fileName.c_str());
		if(pointsLocationfile.fail()){
			string message = "Can't open file: ";
			message += fileName;
			handleError(message, "Trials::ReadPointsLocationdFromFile()");
		}
		else
		{
			_pointLocations.clear();
			while(!pointsLocationfile.eof())	//read until the end of the file
			{
				TwoDoubles location;
				pointsLocationfile>>location.x;
				pointsLocationfile>>location.y;
				if(pointsLocationfile.fail())
					handleError("  can't read points location file", "Trials::ReadPointsLocationdFromFile()");
				_pointLocations.push_back(location);
			}
			pointsLocationfile.close();

		}//else
	}

	// 14-04-2016 ZDJ For PreviewTrials Charlotte
	// Get the positions of the different numbers.
	void trialList::loadNumLocations()	
	{
		string fileName = _behaviorParams["NumLocationFile"].getValueS();

		ifstream numsLocationfile(fileName.c_str());
		if(numsLocationfile.fail()){
			string message = "Can't open file: ";
			message += fileName;
			handleError(message, "Trials::ReadNumsLocationsFromFile()");
		}
		else
		{
			_numLocations.clear();
			while(!numsLocationfile.eof())	//read until the end of the file
			{
				TwoDoubles location;
				int sign;
				numsLocationfile>>location.x;
				numsLocationfile>>location.y;
				numsLocationfile>>sign;
				if(numsLocationfile.fail())
					handleError("  can't read nums location file", "Trials::ReadNumsLocationdFromFile()");
				_numLocations.push_back(location);
				numSigns.push_back(sign);
			}
			numsLocationfile.close();

		}//else
	}

	// Get the location of the .input files
	void trialList::loadTrials(const string& ID, const string& day, int blockNumber)
	{
		//generate a string to know from where to read the information
		ostringstream fileName;
		//"XXXXXXX_DayX_BlockX.input"
		static TCHAR szDirectory[1500] = "";
		GetCurrentDirectory(sizeof(szDirectory) - 1, szDirectory); //directory from which the exe runs

		fileName<<szDirectory<<"\\InputFiles\\" << ID << "_Day" << day << "_Block" << blockNumber << ".input";

		string filenameString = fileName.str();

		ifstream input_file(filenameString.c_str(), ios::in);	//parameters will be read from "input file"
		if(input_file.fail()) {	//check if read successfully
			handleError( "file does not exist in the necessary folder","Located in: ParameterList::ReadFromFile()");
		}

		input_file >> _blockInfo;

		_trialParamsList.readFromFile(input_file);

		_numberOfTrials = _trialParamsList.size();
		_blockNumber = blockNumber;
		_subjectID = ID;
		_subjectDay = day;
	}//end of getTrials