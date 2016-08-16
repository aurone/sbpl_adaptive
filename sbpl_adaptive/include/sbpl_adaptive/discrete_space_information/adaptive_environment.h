#ifndef __ADAPTIVE_ENVIRONMENT_H_
#define __ADAPTIVE_ENVIRONMENT_H_

#include <sbpl/headers.h>

/** \brief base class for environments defining planning graphs
It is independent of the graph search used
The main means of communication between environment and graph search is through stateID.
Each state is uniquely defined by stateID and graph search is ONLY aware of stateIDs. It doesn't know anything about the actual state variables.
Environment, on the other hand, maintains a mapping from stateID to actual state variables (coordinates) using StateID2IndexMapping array
*/
using namespace std;

enum AdaptiveStateType_t : unsigned char { LD=0, NearHD=1, HD=2 };

namespace sbpl_adaptive {

class AdaptiveDiscreteSpaceInformation : virtual public DiscreteSpaceInformation
{

public:
    AdaptiveDiscreteSpaceInformation(){
        trackMode = false;
        StartStateID = -1;
        GoalStateID = -1;
        getPredTime = 0;
        getSuccTime = 0;
    }

    /** \brief destructor
     */
    ~AdaptiveDiscreteSpaceInformation() {

    }

    /* for TRA* planner */
    void GetPreds(int TargetStateID, int expansionStep, std::vector<int>* PredIDV, std::vector<int>* CostV) {
        if(trackMode){
            GetPreds_Track(TargetStateID, expansionStep, PredIDV, CostV);
        } else {
            GetPreds_Plan(TargetStateID, expansionStep, PredIDV, CostV);
        }
    };

    /* for TRA* planner */
    void GetSuccs(int SourceStateID, int expansionStep, std::vector<int>* SuccIDV, std::vector<int>* CostV) {
        if(trackMode){
            GetSuccs_Track(SourceStateID, expansionStep, SuccIDV, CostV);
        } else {
            GetSuccs_Plan(SourceStateID, expansionStep, SuccIDV, CostV);
        }
    };

    void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) {
        if(trackMode){
            GetPreds_Track(TargetStateID, PredIDV, CostV);
        } else {
            GetPreds_Plan(TargetStateID, PredIDV, CostV);
        }
    };

    void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV) {
        if(trackMode){
            GetSuccs_Track(SourceStateID, SuccIDV, CostV);
        } else {
            GetSuccs_Plan(SourceStateID, SuccIDV, CostV);
        }
    };


    //the important functions

    /** \brief checks if the given stateID path is executable
     */
    virtual bool isExecutablePath(const std::vector<int> &stateIDV) = 0;

	/** \brief used by the tracker to tell the environment which state is being expanded
	 */
	virtual void expandingState(int StateID) = 0;

	/** \brief gets the state ID of the 'best' state encountered during tracking
	 */
	virtual int getBestSeenState() = 0;

	/** \brief sets the environment in adaptive planning mode
	*/
	virtual void setPlanMode() = 0;

	/** \brief constructs a tunnel of width tunnelWidth around the path specified by stateIDs_V
	and sets the environment in tracking mode
	*/
	virtual void setTrackMode(const std::vector<int> &stateIDs_V, int cost, std::vector<int> *ModStates = NULL) =0;

	/** \brief adds a new sphere of radius rad at the state coordinates specified by StateID
	 * a list of modified stateIDs is returned in modifiedStates if not NULL
	*/
	virtual void addSphere(int StateID, std::vector<int> *modifiedStates=NULL) = 0;

	/** \brief adds a new sphere of radius rad at the state coordinates specified by StateID
     * returns the earliest expansion step of the modified states
    */
    virtual void addSphere(int StateID, int &first_mod_step) = 0;

    virtual void processCostlyPath(const std::vector<int> &planning_path, const std::vector<int> &tracking_path, std::vector<int>* new_sphere_locations) = 0;

    /** resets the environment to its original state - no spheres, etc. */
    virtual void reset() = 0;

    // used by interleaving planner

    /** \brief gets the executable piece of a stateID path
     */
    virtual int getExecutablePiece(const std::vector<int> &stateIDv, std::vector<std::vector<double>> &traj) = 0;

    /** \brief used by the interleaving adaptive planner to check if the goal has been reached
     */
    virtual bool isGoalReached() = 0;

	virtual void advanceStartTo(int StateID, std::vector<int> *modStates=NULL) = 0;

	//less important functions -- mainly for debugging

	virtual void visualizeStatePath(std::vector<int> *path, int scolor, int ecolor, std::string name) {
        SBPL_ERROR("visualizeStatePath not implemented!");
    };
	virtual void visualizeState(int sID, int scolor, std::string name) {
        SBPL_ERROR("visualizeState not implemented!");
    };

    virtual void visualizeTrajectory(std::vector<std::vector<double> > &traj, int scolor, int ecolor, std::string name) {
        SBPL_ERROR("visualizeTrajectory not implemented!");
    };

	virtual void visualizePath(std::vector<int>* path){
		SBPL_ERROR("visualizePath not implemented for this environment!");
	}

	virtual void visualizeStates(std::vector<int> *stateIDs, int lcolor, int hcolor, std::string ns){

	}

	virtual void visualizeStates(std::vector<int> *stateIDs, std::vector<int> *colorsV, std::string ns){

	}

	virtual void visualizeState(std::string name, std::vector<double> &coords, int col){

	}

	virtual void visualizeEnvironment(){

	}

    // MHA - return anchor
    virtual Heuristic* getAnchorHeur(){
        SBPL_ERROR("Not implemented");
    }

    // MHA - return other heurs
    virtual Heuristic** getHeurs(){
        SBPL_ERROR("Not implemented");    
    }

    // MHA - return number of heurs
    virtual int getNumHeur(){
        SBPL_ERROR("Not implemented");    
    }

	/* useful to have for debugging */
	void pause(){
  		printf("Enter to continue...");
  		char inp;
  		do {
  			inp = getchar();
  		} while (inp != '\n');
  	}

	bool prompt(){
		printf("[y/n]?");
		char inp;
		do {
		    inp = getchar();
		} while(inp == '\n' || inp == '\r'); //skip enter and carriage return
		if(inp == 'y' || inp == 'Y') return true;
		return false;
	}

  	virtual bool stateIDPathToTraj(std::vector<int> stateIDV, std::vector<std::vector<double> > *traj){
  		SBPL_ERROR("stateIDPathToTraj not implemented for this environment!");
  		return false;
  	}

  	inline bool isInTrackingMode() { return trackMode; }

  	std::vector<int> getLastAdaptivePath(){ return lastAdaptivePath_; }

	int StartStateID;
	int GoalStateID;


protected:
	/** NOTES:
	the environment is always in tracking or planning mode. use setTrackMode and setPlanMode to change the mode of the environment
	the SBPL AdaptivePlanner will use setTrackMode and setPlanMode to switch between environment modes
	getSuccs and getPreds functions should take into account the environment mode when generating successor or predecessor states for the planner
	*/

	bool trackMode; // true - tracking, false - planning
	double getSuccTime; //time spent generating successors - for debugging purposes
	double getPredTime; //time spent generating predecessors - for debugging purposes

	std::vector<int> lastAdaptivePath_;

    // Number of heuristics MHA
    int num_heur_;

	/** \brief gets successors for tracking mode -- this should be specified by the user when creating their adaptive_environment file
	*/
	virtual void GetSuccs_Track(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV) = 0;

	/** \brief gets successors for planning mode -- this should be specified by the user when creating their adaptive_environment file
	*/
	virtual void GetSuccs_Plan(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV) = 0;

	/** \brief gets predecessors for tracking mode -- this should be specified by the user when creating their adaptive_environment file
	*/
	virtual void GetPreds_Track(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) = 0;

	/** \brief gets predecessors for planning mode -- this should be specified by the user when creating their adaptive_environment file
	*/
	virtual void GetPreds_Plan(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) = 0;

    /** \brief gets successors for tracking mode -- this should be specified by the user when creating their adaptive_environment file
    */
    virtual void GetSuccs_Track(int SourceStateID, int expansion_step, std::vector<int>* SuccIDV, std::vector<int>* CostV) = 0;

    /** \brief gets successors for planning mode -- this should be specified by the user when creating their adaptive_environment file
    */
    virtual void GetSuccs_Plan(int SourceStateID, int expansion_step, std::vector<int>* SuccIDV, std::vector<int>* CostV) = 0;

    /** \brief gets predecessors for tracking mode -- this should be specified by the user when creating their adaptive_environment file
    */
    virtual void GetPreds_Track(int TargetStateID, int expansion_step, std::vector<int>* PredIDV, std::vector<int>* CostV) = 0;

    /** \brief gets predecessors for planning mode -- this should be specified by the user when creating their adaptive_environment file
    */
    virtual void GetPreds_Plan(int TargetStateID, int expansion_step, std::vector<int>* PredIDV, std::vector<int>* CostV) = 0;
};

}

#endif
