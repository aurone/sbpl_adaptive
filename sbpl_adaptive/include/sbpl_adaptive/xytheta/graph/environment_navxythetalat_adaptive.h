#ifndef SBPL_ADAPTIVE_ENVIRONMENT_NAVXYTHETALAT_ADAPTIVE_H
#define SBPL_ADAPTIVE_ENVIRONMENT_NAVXYTHETALAT_ADAPTIVE_H

class SBPL2DGridSearch;

/** \brief 3D (x,y,theta) planning using lattice-based graph problem. For general structure see comments on parent class DiscreteSpaceInformation
For info on lattice-based planning used here, you can check out the paper: 
Maxim Likhachev and Dave Ferguson, " Planning Long Dynamically-Feasible Maneuvers for Autonomous Vehicles", IJRR'09
 */
class EnvironmentNAVXYTHETALATTICE_AD : public AdaptiveDiscreteSpaceInformation
{
public:

    EnvNAVXYTHETALATConfig_t EnvNAVXYTHETALATCfg;
    EnvironmentNAVXYTHETALATTICE_AD();

    /**
     * \brief initialization of environment from file. See .cfg files for examples
     *  it also takes the perimeter of the robot with respect to some reference point centered at x=0,y=0 and orientation = 0 (along x axis).
     *  The perimeter is defined in meters as a sequence of vertices of a polygon defining the perimeter. If vector is of zero size, then
     *  robot is assumed to be point robot (you may want to inflate all obstacles by its actual radius)
     *  Motion primitives file defines the motion primitives available to the robot
     */
    bool InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const char* sMotPrimFile);
    /**
     * \brief see comments on the same function in the parent class
     */
    bool InitializeEnv(const char* sEnvFile);
    /**
     * \brief way to set up various parameters. For a list of parameters, see the body of the function - it is pretty straightforward
     */
    virtual bool SetEnvParameter(const char* parameter, int value);
    /**
     * \brief returns the value of specific parameter - see function body for the list of parameters
     */
    virtual int GetEnvParameter(const char* parameter);
    /**
     * \brief see comments on the same function in the parent class
     */
    bool InitializeMDPCfg(MDPConfig *MDPCfg);
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int  GetFromToHeuristic(int FromStateID, int ToStateID) = 0;
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int  GetGoalHeuristic(int stateID) = 0;
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int  GetStartHeuristic(int stateID) = 0;
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) = 0;
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void SetAllPreds(CMDPSTATE* state);
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) = 0;

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void EnsureHeuristicsUpdated(bool bGoalHeuristics);

    /**
     * \brief see comments on the same function in the parent class
     */
    void PrintEnv_Config(FILE* fOut);

    /**
     * \brief initialize environment. Gridworld is defined as matrix A of size width by height.
     * So, internally, it is accessed as A[x][y] with x ranging from 0 to width-1 and and y from 0 to height-1
     * Each element in A[x][y] is unsigned char. A[x][y] = 0 corresponds to fully traversable and cost is just Euclidean distance
     * The cost of transition between two neighboring cells is EuclideanDistance*(max(A[sourcex][sourcey],A[targetx][targety])+1)
     * f A[x][y] >= obsthresh, then in the above equation it is assumed to be infinite.
     * The cost also incorporates the length of a motion primitive and its cost_multiplier (see getcost function)
     * mapdata is a pointer to the values of A. If it is null, then A is initialized to all zeros. Mapping is: A[x][y] = mapdata[x+y*width]
     * start/goal are given by startx, starty, starttheta, goalx,goaly, goaltheta in meters/radians.
     * If they are not known yet, just set them to 0. Later setgoal/setstart can be executed
     * finally obsthresh defined obstacle threshold, as mentioned above
     * goaltolerances are currently ignored
     * for explanation of perimeter, see comments for InitializeEnv function that reads all from file
     * cellsize is discretization in meters
     * nominalvel_mpersecs is assumed velocity of vehicle while moving forward in m/sec
     * timetoturn45degsinplace_secs is rotational velocity in secs/45 degrees turn
     */
    bool InitializeEnv(int width, int height,
        /** if mapdata is NULL the grid is initialized to all freespace */
        const unsigned char* mapdata,
        double startx, double starty, double starttheta,
        double goalx, double goaly, double goaltheta,
        double goaltol_x, double goaltol_y, double goaltol_theta,
        const vector<sbpl_2Dpt_t> & perimeterptsV,
        double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
        unsigned char obsthresh, const char* sMotPrimFile);
    /**
     * \brief update the traversability of a cell<x,y>
     */
    bool UpdateCost(int x, int y, unsigned char newcost);

    /**
     * \brief re-setting the whole 2D map
     * transform from linear array mapdata to the 2D matrix used internally: Grid2D[x][y] = mapdata[x+y*width]
     */
    bool SetMap(const unsigned char* mapdata);


    /**
     * \brief this function fill in Predecessor/Successor states of edges whose costs changed
     * It takes in an array of cells whose traversability changed, and returns (in vector preds_of_changededgesIDV)
     * the IDs of all states that have outgoing edges that go through the changed cells
     */
    virtual void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV) = 0;
    /**
     * \brief same as GetPredsofChangedEdges, but returns successor states. Both functions need to be present for incremental search
     */
    virtual void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV) = 0;

    /**
     * returns true if cell is untraversable
     */
    bool IsObstacle(int x, int y);
    /**
     * \brief returns false if robot intersects obstacles or lies outside of the map. Note this is pretty expensive operation since it computes the footprint
     * of the robot based on its x,y,theta
     */
    bool IsValidConfiguration(int X, int Y, int Theta);

    /**
     * \brief returns environment parameters. Useful for creating a copy environment
     */
    void GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double* starttheta, double* goalx, double* goaly, double* goaltheta,
        double* cellsize_m, double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs, unsigned char* obsthresh, vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
    /**
     * \brief get internal configuration data structure
     */
    const EnvNAVXYTHETALATConfig_t* GetEnvNavConfig();


    virtual ~EnvironmentNAVXYTHETALATTICE_AD();
    /**
     * \brief prints time statistics
     */
    void PrintTimeStat(FILE* fOut);
    /**
     * \brief returns the cost corresponding to the cell <x,y>
     */
    unsigned char GetMapCost(int x, int y);

    /**
     * \brief returns true if cell is within map
     */
    bool IsWithinMapCell(int X, int Y);

    /** \brief Transform a pose into discretized form. The angle 'pth' is
      considered to be valid if it lies between -2pi and 2pi (some
      people will prefer 0<=pth<2pi, others -pi<pth<=pi, so this
      compromise should suit everyone).

      \note Even if this method returns false, you can still use the
      computed indices, for example to figure out how big your map
      should have been.

      \return true if the resulting indices lie within the grid bounds
      and the angle was valid.
     */
    bool PoseContToDisc(double px, double py, double pth,
        int &ix, int &iy, int &ith) const;

    /** \brief Transform grid indices into a continuous pose. The computed
      angle lies within 0<=pth<2pi.

      \note Even if this method returns false, you can still use the
      computed indices, for example to figure out poses that lie
      outside of your current map.

      \return true if all the indices are within grid bounds.
     */
    bool PoseDiscToCont(int ix, int iy, int ith,
        double &px, double &py, double &pth) const;

    /** \brief prints environment variables for debugging
     */
    virtual void PrintVars(){};

protected:

    virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);


    //member data
    EnvironmentNAVXYTHETALAT_t EnvNAVXYTHETALAT;
    vector<EnvNAVXYTHETALAT3Dcell_t> affectedsuccstatesV; //arrays of states whose outgoing actions cross cell 0,0
    vector<EnvNAVXYTHETALAT3Dcell_t> affectedpredstatesV; //arrays of states whose incoming actions cross cell 0,0
    int iteration;

    //2D search for heuristic computations
    bool bNeedtoRecomputeStartHeuristics; //set whenever grid2Dsearchfromstart needs to be re-executed
    bool bNeedtoRecomputeGoalHeuristics; //set whenever grid2Dsearchfromgoal needs to be re-executed
    SBPL2DGridSearch* grid2Dsearchfromstart; //computes h-values that estimate distances from start x,y to all cells
    SBPL2DGridSearch* grid2Dsearchfromgoal;  //computes h-values that estimate distances to goal x,y from all cells

    virtual void ReadConfiguration(FILE* fCfg);

    void InitializeEnvConfig(vector<SBPL_xytheta_mprimitive>* motionprimitiveV);


    bool CheckQuant(FILE* fOut);

    void SetConfiguration(int width, int height,
        /** if mapdata is NULL the grid is initialized to all freespace */
        const unsigned char* mapdata,
        int startx, int starty, int starttheta,
        int goalx, int goaly, int goaltheta,
        double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs, const vector<sbpl_2Dpt_t> & robot_perimeterV);

    bool InitGeneral( vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
    void PrecomputeActionswithBaseMotionPrimitive(vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
    void PrecomputeActionswithCompleteMotionPrimitive(vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
    void PrecomputeActions();

    void CreateStartandGoalStates();

    virtual void InitializeEnvironment() = 0;

    void ComputeHeuristicValues();

    virtual bool IsValidCell(int X, int Y);

    void CalculateFootprintForPose(EnvNAVXYTHETALAT3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint);
    void CalculateFootprintForPose(EnvNAVXYTHETALAT3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint, const vector<sbpl_2Dpt_t>& FootprintPolygon);
    void RemoveSourceFootprint(EnvNAVXYTHETALAT3Dpt_t sourcepose, vector<sbpl_2Dcell_t>* footprint);
    void RemoveSourceFootprint(EnvNAVXYTHETALAT3Dpt_t sourcepose, vector<sbpl_2Dcell_t>* footprint, const vector<sbpl_2Dpt_t>& FootprintPolygon);

    virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETALATAction_t*>* actionindV=NULL) = 0;

    double EuclideanDistance_m(int X1, int Y1, int X2, int Y2);

    void ComputeReplanningData();
    void ComputeReplanningDataforAction(EnvNAVXYTHETALATAction_t* action);

    bool ReadMotionPrimitives(FILE* fMotPrims);
    bool ReadinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim, FILE* fIn);
    bool ReadinCell(EnvNAVXYTHETALAT3Dcell_t* cell, FILE* fIn);
    bool ReadinPose(EnvNAVXYTHETALAT3Dpt_t* pose, FILE* fIn);

    void PrintHeuristicValues();

};


class EnvironmentNAVXYTHETALAT_AD : public EnvironmentNAVXYTHETALATTICE_AD
{

public:
    EnvironmentNAVXYTHETALAT_AD()
{
        HashTableSize = 0;
        Coord2StateIDHashTable = NULL;
        Coord2StateIDHashTable_lookup = NULL;
};

    ~EnvironmentNAVXYTHETALAT_AD();

    /** \brief sets start in meters/radians
     */
    int SetStart(double x, double y, double theta);
    /** \brief sets goal in meters/radians
     */
    int SetGoal(double x, double y, double theta);
    /** \brief sets goal tolerance. (Note goal tolerance is ignored currently)
     */
    void SetGoalTolerance(double tol_x, double tol_y, double tol_theta) { /**< not used yet */ }
    /** \brief returns state coordinates of state with ID=stateID
     */
    void GetCoordFromState(int stateID, int& x, int& y, int& theta) const;
    /** \brief returns stateID for a state with coords x,y,theta
     */
    int GetStateFromCoord(int x, int y, int theta);

    /** \brief converts a path given by stateIDs into a sequence of coordinates. Note that since motion primitives are short actions represented as a sequence of points,
  the path returned by this function contains much more points than the number of points in the input path. The returned coordinates are in meters,meters,radians
     */
    void ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<EnvNAVXYTHETALAT3Dpt_t>* xythetaPath);
    /** \brief prints state info (coordinates) into file
     */
    void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
    /** \brief returns all predecessors states and corresponding costs of actions
     */
    virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
    /** \brief returns all successors states, costs of corresponding actions and pointers to corresponding actions, each of which is a motion primitive
  if actionindV is NULL, then pointers to actions are not returned
     */
    virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETALATAction_t*>* actionindV=NULL);

    /** \brief this function fill in Predecessor/Successor states of edges whose costs changed
  It takes in an array of cells whose traversability changed, and returns (in vector preds_of_changededgesIDV) 
  the IDs of all states that have outgoing edges that go through the changed cells
     */
    void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV);
    /** \brief same as GetPredsofChangedEdges, but returns successor states. Both functions need to be present for incremental search
     */
    void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV);

    /** \brief see comments on the same function in the parent class
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    /** \brief see comments on the same function in the parent class
     */
    virtual int  GetFromToHeuristic(int FromStateID, int ToStateID);
    /** \brief see comments on the same function in the parent class
     */
    virtual int  GetGoalHeuristic(int stateID);
    /** \brief see comments on the same function in the parent class
     */
    virtual int  GetStartHeuristic(int stateID);

    /** \brief see comments on the same function in the parent class
     */
    virtual int	 SizeofCreatedEnv();
    /** \brief see comments on the same function in the parent class
     */
    virtual void PrintVars(){};

    /** \brief sets the environment in adaptive planning mode
     */

protected:

    //hash table of size x_size*y_size. Maps from coords to stateId
    int HashTableSize;
    vector<EnvNAVXYTHETALATHashEntry_t*>* Coord2StateIDHashTable;
    //vector that maps from stateID to coords
    vector<EnvNAVXYTHETALATHashEntry_t*> StateID2CoordTable;

    EnvNAVXYTHETALATHashEntry_t** Coord2StateIDHashTable_lookup;

    unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Theta);

    EnvNAVXYTHETALATHashEntry_t* GetHashEntry_hash(int X, int Y, int Theta);
    EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta);
    EnvNAVXYTHETALATHashEntry_t* GetHashEntry_lookup(int X, int Y, int Theta);
    EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta);

    //pointers to functions
    EnvNAVXYTHETALATHashEntry_t* (EnvironmentNAVXYTHETALAT_AD::*GetHashEntry)(int X, int Y, int Theta);
    EnvNAVXYTHETALATHashEntry_t* (EnvironmentNAVXYTHETALAT_AD::*CreateNewHashEntry)(int X, int Y, int Theta);


    virtual void InitializeEnvironment();

    void PrintHashTableHist(FILE* fOut);


};

#endif
