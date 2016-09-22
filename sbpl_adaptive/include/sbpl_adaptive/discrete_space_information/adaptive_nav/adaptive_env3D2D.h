#ifndef __ADAPTIVE_ENVIRONMENT_3D2D_
#define __ADAPTIVE_ENVIRONMENT_3D2D_

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

using namespace std;

typedef struct
{
    unsigned char sphereState; //0 means in sphere, 1 means near sphere, 2 means outside sphere
    unsigned char stateInfo; //0th bit set - obstacle, 1st bit set - low-D expansion, 2nd bit set - high-D expansion, 3rd bit set - tracking expansion
    float distance;
    float sphereDist;
    short int sphereIndex;
} tGridCell;

class AdaptiveEnvironment3D2D : public GenericAdaptiveEnvironment
{ 
private:
    EnvNAVXYTHETALATConfig_t EnvNAVXYTHETALATCfg;
    EnvironmentNAVXYTHETALAT_t EnvNAVXYTHETALAT;
    int INVALID_THETA_;
    std::vector<std::vector<tAction*> > actions_;
    tGridCell** grid2D;
    double NEARTRESH3D_;

public:
    /** \brief destructor
     */
    ~AdaptiveEnvironment3D2D() {
        spheres_.clear();
        if(l_cspace_!=NULL) delete l_cspace_;
        if(h_cspace_!=NULL) delete h_cspace_;
    }

    /** \brief constructor - specify the high-dim size and low-dim size
     */
    AdaptiveEnvironment3D2D()
    {
        //TODO: fix
        INVALID_THETA_ = 16;
        this->l_angle_mask_.resize(2, 0);
        this->h_angle_mask_.resize(3, 0);
        l_cspace_ = new GenericCollisionChecker(this);
        h_cspace_ = new GenericCollisionChecker(this);
    }

    bool InitializeEnv(const char* sEnvFile, const char* mPrimFile){
        bool res = (InitializeEnvFromCFGFile(sEnvFile) &&
                Initialize3DActionsFromFile(mPrimFile) &&
                Initialize2DActions(1000.0));

        reset();

        if(res){
            std::vector<short int> coords (3, 0);
            coords[0] = EnvNAVXYTHETALATCfg.StartX_c;
            coords[1] = EnvNAVXYTHETALATCfg.StartY_c;
            coords[2] = EnvNAVXYTHETALATCfg.StartTheta;
            this->EnvNAVXYTHETALAT.startstateid = this->CreateHashEntry(coords, NULL)->stateID;
            StartStateID = EnvNAVXYTHETALAT.startstateid;
            coords[0] = EnvNAVXYTHETALATCfg.EndX_c;
            coords[1] = EnvNAVXYTHETALATCfg.EndY_c;
            coords[2] = EnvNAVXYTHETALATCfg.EndTheta;
            this->EnvNAVXYTHETALAT.goalstateid = this->CreateHashEntry(coords, NULL)->stateID;
            GoalStateID = EnvNAVXYTHETALAT.goalstateid;
        }

        return res;
    }

    bool InitializeEnv(const char* sEnvFile){
        printf("Use InitializeEnv(const char* sEnvFile, const char* mPrimFile) instead!");
        exit(0);
        return false;
    }

    bool InitializeMDPCfg(MDPConfig *MDPCfg){
        MDPCfg->goalstateid = EnvNAVXYTHETALAT.goalstateid;
        MDPCfg->startstateid = EnvNAVXYTHETALAT.startstateid;

        return true;
    }

    int GetFromToHeuristic(int FromStateID, int ToStateID)
    {
#if USE_HEUR==0
        return 0;
#endif

#if DEBUG
        if(FromStateID >= (int)StateID2CoordTable.size()
                || ToStateID >= (int)StateID2CoordTable.size())
        {
            SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
            throw new SBPL_Exception();
        }
#endif

        //get X, Y for the state
        EnvHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
        EnvHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];

        //TODO - check if one of the gridsearches already computed and then use it.
        return (int)(EuclideanDistance(FromHashEntry, ToHashEntry) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);
    }

    int GetGoalHeuristic(int stateID)
    {
#if USE_HEUR==0
        return 0;
#endif

#if DEBUG
        if(stateID >= (int)StateID2CoordTable.size())
        {
            SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
            throw new SBPL_Exception();
        }
#endif

        EnvHashEntry_t* HashEntry = StateID2CoordTable[stateID];
        EnvHashEntry_t* GoalEntry = StateID2CoordTable[EnvNAVXYTHETALAT.goalstateid];
        int h2D = 0;//grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); //computes distances from start state that is grid2D, so it is EndX_c EndY_c
        int hEuclid = (int)(EuclideanDistance(HashEntry, GoalEntry) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);

        //define this function if it is used in the planner (heuristic backward search would use it)
        return (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETALATCfg.nominalvel_mpersecs);
    }

    int GetStartHeuristic(int stateID){
#if USE_HEUR==0
        return 0;
#endif

#if DEBUG
        if(stateID >= (int)StateID2CoordTable.size())
        {
            SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
            throw new SBPL_Exception();
        }
#endif

        EnvHashEntry_t* HashEntry = StateID2CoordTable[stateID];
        EnvHashEntry_t* StartEntry = StateID2CoordTable[EnvNAVXYTHETALAT.startstateid];
        int h2D = 0;//grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); //computes distances from start state that is grid2D, so it is EndX_c EndY_c
        int hEuclid = (int)(EuclideanDistance(HashEntry, StartEntry) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);

        //define this function if it is used in the planner (heuristic backward search would use it)
        return (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETALATCfg.nominalvel_mpersecs);
    }

    void SetAllActionsandAllOutcomes(CMDPSTATE* state) {
        SBPL_ERROR("ERROR in EnvAdaptive3D2D... SetAllActionsandAllOutcomes not implemented!");
        throw new SBPL_Exception();
    }
    void SetAllPreds(CMDPSTATE* state) {
        SBPL_ERROR("ERROR in EnvAdaptive3D2D... SetAllPreds not implemented!");
        throw new SBPL_Exception();
    }

    int	 SizeofCreatedEnv() {
        return StateID2CoordTable.size();
    }

    void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL) {
        if(fOut==NULL){
            EnvHashEntry_t* state = GetHashEntry(stateID);
            printf("StateID: %d (%d, %d, %d)\n", stateID, state->coord[0], state->coord[1], state->coord[2]);
        } else {
            EnvHashEntry_t* state = GetHashEntry(stateID);
            fprintf(fOut, "StateID: %d (%d, %d, %d)\n", stateID, state->coord[0], state->coord[1], state->coord[2]);
        }
    }

    void PrintEnv_Config(FILE* fOut) {
        SBPL_ERROR("ERROR in EnvAdaptive3D2D... PrintEnv_Config not implemented!");
        throw new SBPL_Exception();
    }

    void resetExpandedStates() {
        SBPL_ERROR("ERROR in EnvAdaptive3D2D... ResetExpandedStates not implemented!");
        throw new SBPL_Exception();
    }

    void expandingState(int StateID) {
        SBPL_ERROR("ERROR in EnvAdaptive3D2D... ExpandingState not implemented!");
        throw new SBPL_Exception();
    }

    void sendStateData(int StateID, int fVal, int gVal, int hVal) {
        SBPL_ERROR("ERROR in EnvAdaptive3D2D... sendStateData not implemented!");
        throw new SBPL_Exception();
    }

    void setPlanMode() {
        for(int x = 0; x < this->EnvNAVXYTHETALATCfg.EnvWidth_c; x++){
            for(int y = 0; y < this->EnvNAVXYTHETALATCfg.EnvHeight_c; y++){
                grid2D[x][y].sphereState = grid2D[x][y].sphereState & 3;
            }
        }
        trackMode = false;
    }

    void setTrackMode(std::vector<int> *stateIDs_V, double tunnelWidth, double TIMEOUT=1000.0, std::vector<int> *ModStates=NULL){
        trackMode = true;
        //TODO: fix
        SBPL_ERROR("ERROR in EnvAdaptive3D2D... SetTrackMode not implemented!");
        throw new SBPL_Exception();
    }

    void addSphere(int StateID, double rad, std::vector<int> *modStates) {
        //if in tracking mode put the spheres in both grids
        //if in planning mode put the spheres in the tracking grid
        // in shpereStateGrid each char 00      :00      :00      :00
        //                              not used not used tracking planning
        int x_d, y_d;
        double x, y;
        double r = rad;
        EnvHashEntry_t* state = this->GetHashEntry(StateID);
        x = this->disc2cont(state->coord[0]);
        y = this->disc2cont(state->coord[1]);
        bool growing = false;
        if(!trackMode) {
            if(this->lookup_stategrid(&(state->coord))==STATEGRID_IN){
                //sphere is here already
                //find the sphere and grow it
                for(int i = spheres_.size() - 1; i >= 0; i--){
                    if(((x-spheres_[i][0])*(x-spheres_[i][0]) + (y-spheres_[i][1])*(y-spheres_[i][1])) < fabs(max(spheres_[i][2] - rad, 0.10))){
                        SBPL_WARN("Growing sphere!");
                        growing = true;
                        //in this sphere - grow it
                        //EnvHashEntry_t* StartEntry = GetHashEntry(this->EnvNAVXYTHETALAT.startstateid);
                        r = 1.5*spheres_[i][2];
                        spheres_[i][2] = r;
                        x = spheres_[i][0];
                        y = spheres_[i][1];
                        x_d = cont2disc(x);
                        y_d = cont2disc(y);
                        break;
                    } //end if inside sphere
                } //end for each sphere
            } //end if in sphere
        } //end if !trackMode

        //TODO: in track mode for each cell store the shortest distance and the index of the point along the path at that distance
        //so that given a cell, we can quickly look up the the nearest point along the path corresponding to that cell
        if(!trackMode && !growing){
            std::vector<double> s (3,0.0);
            spheres_.push_back(s);
            spheres_[spheres_.size()-1][0] = x;
            spheres_[spheres_.size()-1][1] = y;
            spheres_[spheres_.size()-1][2] = r;
        }

        double tresh = NEARTRESH3D_;
        int min_x = max(0, cont2disc(x - r - tresh) - 1);
        int max_x = min(cont2disc(x + r + tresh) + 1, EnvNAVXYTHETALATCfg.EnvWidth_c - 1);
        int min_y = max(0, cont2disc(y - r - tresh) - 1);
        int max_y = min(cont2disc(y + r + tresh) + 1, EnvNAVXYTHETALATCfg.EnvHeight_c - 1);

        for(int i = min_x; i <= max_x; i++){
            double i_d = disc2cont(i);
            for(int j = min_y; j <= max_y; j++){
                double j_d = disc2cont(j);
                double dist = sqrt( (x - i_d)*(x - i_d) + (y - j_d)*(y - j_d) );
                char currSphereState = this->lookup_stategrid(i, j);
                if(currSphereState == STATEGRID_IN) continue; //in a sphere already -- no change
                char newSphereState;
                if (dist <= r && currSphereState != STATEGRID_IN){
                    newSphereState = STATEGRID_IN;
                    //in sphere -- modified states in both tracking and planning
                    if(modStates!=NULL){ //not is sphere already
                        if(!trackMode){
                            std::vector<short int> coord(3,0);
                            coord[0] = i; coord[1] = j; coord[2] = INVALID_THETA_;
                            if(this->ExistsHashEntry(coord)){
                                int stateID = this->GetHashEntryFromCoord(coord)->stateID;
                                if(stateID != this->EnvNAVXYTHETALAT.startstateid){
                                    modStates->push_back(stateID);
                                }
                            }
                        } else {
                            std::vector<short int> coord(3,0);
                            coord[0] = i; coord[1] = j;
                            for(int th = 0; th < INVALID_THETA_; th++){
                                coord[2] = th;
                                if(this->ExistsHashEntry(coord)){
                                    int stateID = this->GetHashEntryFromCoord(coord)->stateID;
                                    if(stateID != this->EnvNAVXYTHETALAT.startstateid){
                                        modStates->push_back(stateID);
                                    }
                                }
                            }
                        }
                    }
                    this->set_stategrid(i, j, newSphereState);
                    continue;
                }
                if(dist <= r + tresh && currSphereState == STATEGRID_FAR){
                    newSphereState = STATEGRID_NEAR;
                    std::vector<short int> coord(3,0);
                    coord[0] = i; coord[1] = j; coord[2] = INVALID_THETA_;
                    //near sphere -- modified states only in planning mode
                    if(!trackMode && modStates!=NULL && this->ExistsHashEntry(coord)){
                        //not near or in sphere already
                        int stateID = this->GetHashEntryFromCoord(coord)->stateID;
                        if(stateID != this->EnvNAVXYTHETALAT.startstateid){
                            modStates->push_back(stateID);
                        }
                    }
                    this->set_stategrid(i, j, newSphereState);
                    continue;
                }
            }
        }
        print_stategrid();
    }

    bool isInBounds(std::vector<short int> *coords){
        return (coords->at(0) >= 0) &&
                (coords->at(0) < this->EnvNAVXYTHETALATCfg.EnvWidth_c) &&
                (coords->at(1) >= 0) &&
                (coords->at(1) < this->EnvNAVXYTHETALATCfg.EnvHeight_c) &&
                (coords->at(2) >= 0) &&
                (coords->at(2) <= INVALID_THETA_);
    }

    void moveStartTo(int StateID) {
        SBPL_ERROR("ERROR in EnvAdaptive3D2D... moveStartTo not implemented!");
        throw new SBPL_Exception();
    }

    int getTrackingFailStateID() {
        SBPL_ERROR("ERROR in EnvAdaptive3D2D... getTrackingFailStateID not implemented!");
        throw new SBPL_Exception();
        return false;
    }

    void addSphereWhereTrackingFailed(double rad) {
        SBPL_ERROR("ERROR in EnvAdaptive3D2D... addSphereWhereTrackingFailed not implemented!");
        throw new SBPL_Exception();
    }

    void reset() {
        //reset the state-grid
        spheres_.clear();
        for(int x = 0; x < this->EnvNAVXYTHETALATCfg.EnvWidth_c; x++){
            for(int y = 0; y < this->EnvNAVXYTHETALATCfg.EnvHeight_c; y++){
                grid2D[x][y].sphereState = STATEGRID_DEFAULT;
                grid2D[x][y].distance = 0;
                grid2D[x][y].sphereDist = 0;
                grid2D[x][y].sphereIndex = -1;
                grid2D[x][y].stateInfo = (EnvNAVXYTHETALATCfg.Grid2D[x][y] > 0)?1:0;
            }
        }
        for(unsigned int i = 0; i < StateID2CoordTable.size(); i++){
            delete StateID2CoordTable[i];
        }
        this->StateID2CoordTable.clear();
        for(unsigned int i = 0; i < StateID2IndexMapping.size(); i++){
            delete [] StateID2IndexMapping[i];
        }
        this->StateID2IndexMapping.clear();
        if(this->isUsingHashTable){
            delete [] Coord2StateIDHashTable;
            Coord2StateIDHashTable = new std::vector<EnvHashEntry_t*>[HashTableSize];
        } else {
            for(int i = 0; i < (EnvNAVXYTHETALATCfg.EnvHeight_c * EnvNAVXYTHETALATCfg.EnvWidth_c * (INVALID_THETA_+1)); i++){
                this->Coord2StateIDHashTable_lookup[i] = NULL;
            }
        }
        trackMode = false;
    }

private:

    std::vector<tAction*>* getActionsForState(EnvHashEntry_t* state) {
        if(this->lookup_stategrid(&(state->coord)) == STATEGRID_IN){
            return &(actions_[state->coord[2]]);
        }
        return &(actions_[INVALID_THETA_]);
    }

    void coords_disc2cont_h(std::vector<short int> *in, std::vector<double> *out) {
        for(int i = 0; i < (int)h_angle_mask_.size(); i++){
            if(h_angle_mask_[i] > 0){
                out->at(i) = disc2cont_angle(in->at(i));
            } else {
                out->at(i) = disc2cont(in->at(i));
            }
        }
    }

    void coords_disc2cont_l(std::vector<short int> *in, std::vector<double> *out) {
        for(int i = 0; i < (int)l_angle_mask_.size(); i++){
            if(l_angle_mask_[i] > 0){
                out->at(i) = disc2cont_angle(in->at(i));
            } else {
                out->at(i) = disc2cont(in->at(i));
            }
        }
    }

    char lookup_stategrid(std::vector<short int> *coords) {
        return lookup_stategrid(coords->at(0), coords->at(1));
    }

    void set_stategrid(int x, int y, char state){
        char sphereState_p = (grid2D[x][y].sphereState % 4);
        char sphereState_t = (grid2D[x][y].sphereState >> 2);
        if(!trackMode) {
            //in plan mode can set both planning and tracking
            sphereState_p = state;
            sphereState_t = state;
        } else {
            //in track mode only set tracking
            sphereState_t = state;
        }
        grid2D[x][y].sphereState = 4*sphereState_t + sphereState_p;
    }

    void print_stategrid(){
        for(int i = 0; i < this->EnvNAVXYTHETALATCfg.EnvWidth_c; i++){
            for(int j = 0; j < this->EnvNAVXYTHETALATCfg.EnvHeight_c; j++){
                printf("%d", lookup_stategrid(i,j));
            }
            printf("\n");
        }
    }

    char lookup_stategrid(int x, int y){
        char sphereState = grid2D[x][y].sphereState;
        char sphereState_t = sphereState >> 2;
        char sphereState_p = sphereState - (sphereState_t << 2);
        if(trackMode){
            return min(sphereState_t, sphereState_p);
        } else {
            return sphereState_p;
        }
    }

    void projectToLowDCoords(std::vector<short int> *h_coords) {
        h_coords->at(2) = INVALID_THETA_;
    }

    void projectToHighD(int StateID, std::vector<int> *StateIDsV) {
        EnvHashEntry_t* state = GetHashEntry(StateID);
        for(int th = 0; th < INVALID_THETA_; th++){
            std::vector<short int> *coords = new std::vector<short int>(state->coord);
            coords->at(2) = th;
            if(!isValidHighDState(coords)) continue;
            if(ExistsHashEntry(*coords)){
                StateIDsV->push_back(GetHashEntryFromCoord(*coords)->stateID);
            } else {
                StateIDsV->push_back(CreateHashEntry(*coords, NULL)->stateID);
            }
        }
    }

    int projectToLowD(int StateID) {
        EnvHashEntry_t* state = GetHashEntry(StateID);
        std::vector<short int> *coords = new std::vector<short int>(state->coord);
        coords->at(2) = INVALID_THETA_;
        if(ExistsHashEntry(*coords)){
            return (GetHashEntryFromCoord(*coords)->stateID);
        } else {
            return (CreateHashEntry(*coords, NULL)->stateID);
        }
    }

    bool isValidLowDState(std::vector<short int> *coords) {
        if(trackMode) return false;
        if(!isInBounds(coords)) return false;
        if(this->lookup_stategrid(coords) != STATEGRID_IN){
            return true;
        }
        return false;
    }

    bool isValidHighDState(std::vector<short int> *coords) {
        if(!isInBounds(coords)) return false;
        if(this->lookup_stategrid(coords) == STATEGRID_IN){
            return true;
        }
        return false;
    }

    double disc2cont(int coord){
        return coord * EnvNAVXYTHETALATCfg.cellsize_m;
    }

    int cont2disc(double coord){
        return (int) (coord / EnvNAVXYTHETALATCfg.cellsize_m);
    }

    double disc2cont_angle(int coord){
        return 2.0 * M_PI * coord / (double) INVALID_THETA_;
    }

    int cont2disc_angle(double coord){
        return (int)(INVALID_THETA_ * coord / (2.0 * M_PI));
    }

    double EuclideanDistance(EnvHashEntry_t* S1, EnvHashEntry_t* S2){
        double d_x = disc2cont(S1->coord[0]) - disc2cont(S2->coord[0]);
        double d_y = disc2cont(S1->coord[1]) - disc2cont(S2->coord[1]);
        return sqrt(d_x*d_x + d_y*d_y);
    }

    int HashTableSize;
    bool isUsingHashTable;
    std::vector<EnvHashEntry_t*>* Coord2StateIDHashTable;
    //vector that maps from stateID to coords
    std::vector<EnvHashEntry_t*> StateID2CoordTable;
    EnvHashEntry_t** Coord2StateIDHashTable_lookup;

    unsigned int GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta){
        return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Theta)<<2)) & (HashTableSize-1);
    }

    static unsigned int inthash(unsigned int key)
    {
        key += (key << 12);
        key ^= (key >> 22);
        key += (key << 4);
        key ^= (key >> 9);
        key += (key << 10);
        key ^= (key >> 2);
        key += (key << 7);
        key ^= (key >> 12);
        return key;
    }

    EnvHashEntry_t* GetHashEntry_hash(int StateID) {
        return StateID2CoordTable[StateID];
    }

    EnvHashEntry_t* GetHashEntryFromCoord_hash(std::vector<short int> coords) {
        short int X = coords[0];
        short int Y = coords[1];
        short int Theta = coords[2];
        int binid = GETHASHBIN(X, Y, Theta);

        //iterate over the states in the bin and select the perfect match
        std::vector<EnvHashEntry_t*>* binV = &Coord2StateIDHashTable[binid];
        for(int ind = 0; ind < (int)binV->size(); ind++)
        {
            EnvHashEntry_t* hashentry = binV->at(ind);
            if( hashentry->coord[0] == X  && hashentry->coord[1] == Y && hashentry->coord[2] == Theta)
            {
                return hashentry;
            }
        }

        return NULL;
    }

    bool ExistsHashEntry_hash(std::vector<short int> coords){
        short int X = coords[0];
        short int Y = coords[1];
        short int Theta = coords[2];
        int binid = GETHASHBIN(X, Y, Theta);

        std::vector<EnvHashEntry_t*>* binV = &Coord2StateIDHashTable[binid];
        for(int ind = 0; ind < (int)binV->size(); ind++)
        {
            EnvHashEntry_t* hashentry = binV->at(ind);
            if( hashentry->coord[0] == X  && hashentry->coord[1] == Y && hashentry->coord[2] == Theta)
            {
                return true;
            }
        }

        return false;
    }

    EnvHashEntry_t* CreateNewHashEntry_hash(std::vector<short int> coords, tAction* action){

        int idx;

        EnvHashEntry_t* HashEntry = new EnvHashEntry_t;

        HashEntry->coord = coords;
        HashEntry->stateID = StateID2CoordTable.size();

        //insert into the tables
        StateID2CoordTable.push_back(HashEntry);

        //get the hash table bin
        idx = GETHASHBIN(HashEntry->coord[0], HashEntry->coord[1], HashEntry->coord[2]);

        //insert the entry into the bin
        Coord2StateIDHashTable[idx].push_back(HashEntry);

        //insert into and initialize the mappings
        int* entry = new int [NUMOFINDICES_STATEID2IND];
        StateID2IndexMapping.push_back(entry);
        for(int i = 0; i < NUMOFINDICES_STATEID2IND; i++)
        {
            StateID2IndexMapping[HashEntry->stateID][i] = -1;
        }

        if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
        {
            printf("ERROR in Env... CreateNewHashEntry_hash: last state has incorrect stateID\n");
            exit(1);
        }

        return HashEntry;
    }

    EnvHashEntry_t* GetHashEntry_lookup(int StateID){
        return StateID2CoordTable[StateID];
    }
    EnvHashEntry_t* GetHashEntryFromCoord_lookup(std::vector<short int> coords){
        int index = COORD2INDEX(coords);
        return Coord2StateIDHashTable_lookup[index];
    }
    bool ExistsHashEntry_lookup(std::vector<short int> coords){
        int index = COORD2INDEX(coords);
        if(Coord2StateIDHashTable_lookup[index] == NULL){
            return false;
        }
        return true;
    }

    EnvHashEntry_t* CreateNewHashEntry_lookup(std::vector<short int> coords, tAction* action){
        int i;

#if TIME_DEBUG
        clock_t currenttime = clock();
#endif

        EnvHashEntry_t* HashEntry = new EnvHashEntry_t;

        HashEntry->coord = coords;
        HashEntry->stateID = StateID2CoordTable.size();

        //insert into the tables
        StateID2CoordTable.push_back(HashEntry);

        int index = COORD2INDEX(coords);

#if DEBUG
        if(Coord2StateIDHashTable_lookup[index] != NULL)
        {
            SBPL_ERROR("ERROR: creating hash entry for non-NULL hashentry\n");
            throw new SBPL_Exception();
        }
#endif

        Coord2StateIDHashTable_lookup[index] = 	HashEntry;

        //insert into and initialize the mappings
        int* entry = new int [NUMOFINDICES_STATEID2IND];
        StateID2IndexMapping.push_back(entry);
        for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
        {
            StateID2IndexMapping[HashEntry->stateID][i] = -1;
        }

        if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
        {
            SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
            printf("%d vs. %d", HashEntry->stateID, (int)StateID2IndexMapping.size()-1);
            throw new SBPL_Exception();
        }

#if TIME_DEBUG
        time_createhash += clock()-currenttime;
#endif

        return HashEntry;
    }

    //pointers to functions
    EnvHashEntry_t* GetHashEntry(int StateID){
        if(isUsingHashTable){
            return GetHashEntry_hash(StateID);
        } else {
            return GetHashEntry_lookup(StateID);
        }
    }
    EnvHashEntry_t* GetHashEntryFromCoord(std::vector<short int> coords){
        if(isUsingHashTable){
            return GetHashEntryFromCoord_hash(coords);
        } else {
            return GetHashEntryFromCoord_lookup(coords);
        }
    }
    EnvHashEntry_t* CreateHashEntry(std::vector<short int> coords, tAction* action){
        if(isUsingHashTable){
            return CreateNewHashEntry_hash(coords, action);
        } else {
            return CreateNewHashEntry_lookup(coords, action);
        }
    }
    bool ExistsHashEntry(std::vector<short int> coords){
        if(isUsingHashTable){
            return ExistsHashEntry_hash(coords);
        } else {
            return ExistsHashEntry_lookup(coords);
        }
    }

    bool InitializeEnvFromCFGFile(const char* sEnvFile){
        FILE* fCfg = fopen(sEnvFile, "r");
        if(fCfg == NULL)
        {
            printf("ERROR: unable to open %s\n", sEnvFile);
            exit(1);
        }
        return ReadConfiguration(fCfg);
    }

    bool ReadConfiguration(FILE* fCfg){
        {
            //read in the configuration of environment and initialize  EnvNAVXYTHETALATCfg structure
            char sTemp[1024], sTemp1[1024];
            int dTemp;
            int x, y;

            //discretization(cells)
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            strcpy(sTemp1, "discretization(cells):");
            if(strcmp(sTemp1, sTemp) != 0)
            {
                SBPL_ERROR("ERROR: configuration file has incorrect format\n");
                SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
                throw new SBPL_Exception();
            }
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.EnvWidth_c = atoi(sTemp);
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.EnvHeight_c = atoi(sTemp);

            //obsthresh:
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            strcpy(sTemp1, "obsthresh:");
            if(strcmp(sTemp1, sTemp) != 0)
            {
                SBPL_ERROR("ERROR: configuration file has incorrect format\n");
                SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
                SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
                throw new SBPL_Exception();
            }
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.obsthresh = atoi(sTemp);
            SBPL_PRINTF("obsthresh = %d\n", EnvNAVXYTHETALATCfg.obsthresh);

            //cost_inscribed_thresh:
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            strcpy(sTemp1, "cost_inscribed_thresh:");
            if(strcmp(sTemp1, sTemp) != 0)
            {
                SBPL_ERROR("ERROR: configuration file has incorrect format\n");
                SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
                SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
                throw new SBPL_Exception();
            }
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.cost_inscribed_thresh = atoi(sTemp);
            SBPL_PRINTF("cost_inscribed_thresh = %d\n", EnvNAVXYTHETALATCfg.cost_inscribed_thresh);


            //cost_possibly_circumscribed_thresh:
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            strcpy(sTemp1, "cost_possibly_circumscribed_thresh:");
            if(strcmp(sTemp1, sTemp) != 0)
            {
                SBPL_ERROR("ERROR: configuration file has incorrect format\n");
                SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
                SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
                throw new SBPL_Exception();
            }
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh = atoi(sTemp);
            SBPL_PRINTF("cost_possibly_circumscribed_thresh = %d\n", EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh);


            //cellsize
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            strcpy(sTemp1, "cellsize(meters):");
            if(strcmp(sTemp1, sTemp) != 0)
            {
                SBPL_ERROR("ERROR: configuration file has incorrect format\n");
                SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
                throw new SBPL_Exception();
            }
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.cellsize_m = atof(sTemp);

            //speeds
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            strcpy(sTemp1, "nominalvel(mpersecs):");
            if(strcmp(sTemp1, sTemp) != 0)
            {
                SBPL_ERROR("ERROR: configuration file has incorrect format\n");
                SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
                throw new SBPL_Exception();
            }
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.nominalvel_mpersecs = atof(sTemp);
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            strcpy(sTemp1, "timetoturn45degsinplace(secs):");
            if(strcmp(sTemp1, sTemp) != 0)
            {
                SBPL_ERROR("ERROR: configuration file has incorrect format\n");
                SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
                throw new SBPL_Exception();
            }
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs = atof(sTemp);

            //start(meters,rads):
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.StartX_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETALATCfg.cellsize_m);
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.StartY_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETALATCfg.cellsize_m);
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.StartTheta = ContTheta2Disc(atof(sTemp), NAVXYTHETALAT_THETADIRS);


            if(EnvNAVXYTHETALATCfg.StartX_c < 0 || EnvNAVXYTHETALATCfg.StartX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c)
            {
                SBPL_ERROR("ERROR: illegal start coordinates\n");
                throw new SBPL_Exception();
            }
            if(EnvNAVXYTHETALATCfg.StartY_c < 0 || EnvNAVXYTHETALATCfg.StartY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c)
            {
                SBPL_ERROR("ERROR: illegal start coordinates\n");
                throw new SBPL_Exception();
            }
            if(EnvNAVXYTHETALATCfg.StartTheta < 0 || EnvNAVXYTHETALATCfg.StartTheta >= NAVXYTHETALAT_THETADIRS) {
                SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
                throw new SBPL_Exception();
            }

            //end(meters,rads):
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.EndX_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETALATCfg.cellsize_m);
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.EndY_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETALATCfg.cellsize_m);
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.EndTheta = ContTheta2Disc(atof(sTemp), NAVXYTHETALAT_THETADIRS);;

            if(EnvNAVXYTHETALATCfg.EndX_c < 0 || EnvNAVXYTHETALATCfg.EndX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c)
            {
                SBPL_ERROR("ERROR: illegal end coordinates\n");
                throw new SBPL_Exception();
            }
            if(EnvNAVXYTHETALATCfg.EndY_c < 0 || EnvNAVXYTHETALATCfg.EndY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c)
            {
                SBPL_ERROR("ERROR: illegal end coordinates\n");
                throw new SBPL_Exception();
            }
            if(EnvNAVXYTHETALATCfg.EndTheta < 0 || EnvNAVXYTHETALATCfg.EndTheta >= NAVXYTHETALAT_THETADIRS) {
                SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
                throw new SBPL_Exception();
            }

            if(EnvNAVXYTHETALATCfg.EnvWidth_c * EnvNAVXYTHETALATCfg.EnvHeight_c * (INVALID_THETA_+1) > 100000){
                this->isUsingHashTable = true;
                HashTableSize = 4*1024*1024; //should be power of two
                Coord2StateIDHashTable = new std::vector<EnvHashEntry_t*>[HashTableSize];
                Coord2StateIDHashTable_lookup = NULL;
            } else {
                this->isUsingHashTable = false;
                Coord2StateIDHashTable_lookup = new EnvHashEntry_t*[EnvNAVXYTHETALATCfg.EnvWidth_c * EnvNAVXYTHETALATCfg.EnvHeight_c * (INVALID_THETA_+1)];
                Coord2StateIDHashTable = NULL;
                for(int i = 0; i < EnvNAVXYTHETALATCfg.EnvWidth_c * EnvNAVXYTHETALATCfg.EnvHeight_c * (INVALID_THETA_+1); i++){
                    Coord2StateIDHashTable_lookup[i] = NULL;
                }
            }

            //allocate the 2D environment
            grid2D = new tGridCell* [EnvNAVXYTHETALATCfg.EnvWidth_c];
            EnvNAVXYTHETALATCfg.Grid2D = new unsigned char* [EnvNAVXYTHETALATCfg.EnvWidth_c];
            for (x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++)
            {
                grid2D[x] = new tGridCell[EnvNAVXYTHETALATCfg.EnvHeight_c];
                EnvNAVXYTHETALATCfg.Grid2D[x] = new unsigned char [EnvNAVXYTHETALATCfg.EnvHeight_c];
            }

            //environment:
            if(fscanf(fCfg, "%s", sTemp) != 1){
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            for (y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++)
                for (x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++)
                {
                    if(fscanf(fCfg, "%d", &dTemp) != 1)
                    {
                        SBPL_ERROR("ERROR: incorrect format of config file\n");
                        throw new SBPL_Exception();
                    }
                    EnvNAVXYTHETALATCfg.Grid2D[x][y] = dTemp;
                    grid2D[x][y].distance = 0;
                    grid2D[x][y].sphereDist = 0;
                    grid2D[x][y].sphereIndex = -1;
                    grid2D[x][y].sphereState = STATEGRID_DEFAULT;
                    grid2D[x][y].stateInfo = (dTemp > 0)?1:0;
                }
        }
        return true;
    }

    bool Initialize3DActionsFromFile(const char* mPrimFile){
        h_angle_mask_.resize(3, 0);
        h_angle_mask_[0] = 0;
        h_angle_mask_[1] = 0;
        h_angle_mask_[2] = 0;
        if(mPrimFile != NULL)
        {
            FILE* fMotPrim = fopen(mPrimFile, "r");
            if(fMotPrim == NULL)
            {
                printf("ERROR: unable to open %s\n", mPrimFile);
                exit(1);
            }
            return ReadMotionPrimitives(fMotPrim);
        }
        else {
            printf("ERROR: no motion primitives file specified!");
            exit(1);
        }
        return false;
    }

    bool ReadMotionPrimitives(FILE* fMotPrims)
    {
        char sTemp[1024], sExpected[1024];
        float fTemp;
        int dTemp;
        int totalNumofActions = 0;

        printf("Reading in motion primitives...");

        //read in the resolution
        strcpy(sExpected, "resolution_m:");

        if(fscanf(fMotPrims, "%s", sTemp) == 0)
            return false;

        if(strcmp(sTemp, sExpected) != 0){
            printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
            return false;
        }

        if(fscanf(fMotPrims, "%f", &fTemp) == 0)
            return false;

        if(fabs(fTemp-EnvNAVXYTHETALATCfg.cellsize_m) > ERR_EPS){
            printf("ERROR: invalid resolution %f (instead of %f) in the dynamics file\n",
                    fTemp, EnvNAVXYTHETALATCfg.cellsize_m);
            return false;
        }

        //read in the angular resolution
        strcpy(sExpected, "numberofangles:");

        if(fscanf(fMotPrims, "%s", sTemp) == 0)
            return false;

        if(strcmp(sTemp, sExpected) != 0){
            printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
            return false;
        }
        if(fscanf(fMotPrims, "%d", &dTemp) == 0)
            return false;

        h_angle_mask_[2] = dTemp;
        INVALID_THETA_ = dTemp;
        actions_.resize(dTemp+1);

        //read in the total number of actions
        strcpy(sExpected, "totalnumberofprimitives:");

        if(fscanf(fMotPrims, "%s", sTemp) == 0)
            return false;

        if(strcmp(sTemp, sExpected) != 0){
            printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
            return false;
        }

        if(fscanf(fMotPrims, "%d", &totalNumofActions) == 0){
            return false;
        }

        this->NEARTRESH3D_ = 0;

        for(int i = 0; i < totalNumofActions; i++){
            tAction* motprim = new tAction();

            if(ReadinMotionPrimitive(motprim, fMotPrims) == false)
                return false;

            actions_[motprim->aid].push_back(motprim);

            double dx = disc2cont(motprim->d_[0]);
            double dy = disc2cont(motprim->d_[1]);
            double len = sqrt(dx*dx + dy*dy);
            if(len > this->NEARTRESH3D_){
                this->NEARTRESH3D_ = len;
            }
        }
        this->NEARTRESH3D_ *= 1.2;
        printf("done!\n");
        return true;
    }

    bool ReadinMotionPrimitive(tAction* pMotPrim, FILE* fIn)
    {
        char sTemp[1024];
        int dTemp;
        float fTemp;
        char sExpected[1024];
        int numofIntermPoses;

        //read in actionID
        strcpy(sExpected, "primID:");
        if(fscanf(fIn, "%s", sTemp) == 0)
            return false;

        if(strcmp(sTemp, sExpected) != 0){
            printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
            return false;
        }

        if(fscanf(fIn, "%d", &dTemp) != 1)
            return false;

        //read in start angle
        strcpy(sExpected, "startangle_c:");
        if(fscanf(fIn, "%s", sTemp) == 0)
            return false;
        if(strcmp(sTemp, sExpected) != 0){
            printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
            return false;
        }
        if(fscanf(fIn, "%d", &dTemp) == 0)
        {
            printf("ERROR reading startangle\n");
            return false;
        }
        int starttheta_c = dTemp;
        pMotPrim->aid = starttheta_c;

        //read in end pose
        strcpy(sExpected, "endpose_c:");
        if(fscanf(fIn, "%s", sTemp) == 0)
            return false;
        if(strcmp(sTemp, sExpected) != 0){
            printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
            return false;
        }

        if(fscanf(fIn, "%d", &dTemp) == 0)
            return false;
        pMotPrim->d_.push_back(dTemp);
        if(fscanf(fIn, "%d", &dTemp) == 0)
            return false;
        pMotPrim->d_.push_back(dTemp);
        if(fscanf(fIn, "%d", &dTemp) == 0)
            return false;
        pMotPrim->d_.push_back(dTemp - starttheta_c);

        //read in action cost
        strcpy(sExpected, "additionalactioncostmult:");
        if(fscanf(fIn, "%s", sTemp) == 0)
            return false;
        if(strcmp(sTemp, sExpected) != 0){
            printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
            return false;
        }
        if(fscanf(fIn, "%d", &dTemp) != 1)
            return false;
        pMotPrim->cost = dTemp;

        //read in intermediate poses
        strcpy(sExpected, "intermediateposes:");
        if(fscanf(fIn, "%s", sTemp) == 0)
            return false;
        if(strcmp(sTemp, sExpected) != 0){
            printf("ERROR: expected %s but got %s\n", sExpected, sTemp);
            return false;
        }
        if(fscanf(fIn, "%d", &numofIntermPoses) != 1)
            return false;
        //all intermposes should be with respect to 0,0 as starting pose since it will be added later and should be done
        //after the action is rotated by initial orientation
        for(int i = 0; i < numofIntermPoses; i++){
            std::vector<double> intermpose(3,0.0);

            if(fscanf(fIn, "%f", &fTemp)==0)
                return false;
            intermpose[0] = fTemp;

            if(fscanf(fIn, "%f", &fTemp)==0)
                return false;
            intermpose[1] = fTemp;

            if(fscanf(fIn, "%f", &fTemp)==0)
                return false;
            intermpose[2] = fTemp;

            pMotPrim->pts.push_back(intermpose);
        }
        return true;
    }

    void LERP(tAction* act, int steps){
        double dx = disc2cont(act->d_[0]);
        double dy = disc2cont(act->d_[1]);
        double delta = 1.0 / (double) (steps);
        for(double t = 0; t < 1; t+=delta){
            std::vector<double> pt(2,0.0);
            pt[0] = t * dx;
            pt[1] = t * dy;
            act->pts.push_back(pt);
        }
    }

    bool Initialize2DActions(double dCostMult){
        l_angle_mask_.resize(2,0);
        l_angle_mask_[0] = 0;
        l_angle_mask_[1] = 0;
        //8-connected 2D grid
        tAction* act1 = new tAction();
        act1->d_.resize(2, 0);
        act1->d_[0] = -1; act1->d_[1] = -1;
        act1->cost = (int) (dCostMult * sqrtf((float)(act1->d_[0]*act1->d_[0] + act1->d_[1]*act1->d_[1])));
        act1->aid = INVALID_THETA_;
        LERP(act1, 10);

        tAction* act2 = new tAction();
        act2->d_.resize(2, 0);
        act2->d_[0] = -1; act2->d_[1] = 0;
        act2->cost = (int) (dCostMult * sqrtf((float)(act2->d_[0]*act2->d_[0] + act2->d_[1]*act2->d_[1])));
        act2->aid = INVALID_THETA_;
        LERP(act2, 10);

        tAction* act3 = new tAction();
        act3->d_.resize(2, 0);
        act3->d_[0] = -1; act3->d_[1] = 1;
        act3->cost = (int) (dCostMult * sqrtf((float)(act3->d_[0]*act3->d_[0] + act3->d_[1]*act3->d_[1])));
        act3->aid = INVALID_THETA_;
        LERP(act3, 10);

        tAction* act4 = new tAction();
        act4->d_.resize(2, 0);
        act4->d_[0] = 0; act4->d_[1] = -1;
        act4->cost = (int) (dCostMult * sqrtf((float)(act4->d_[0]*act4->d_[0] + act4->d_[1]*act4->d_[1])));
        act4->aid = INVALID_THETA_;
        LERP(act4, 10);

        tAction* act5 = new tAction();
        act5->d_.resize(2, 0);
        act5->d_[0] = 0; act5->d_[1] = 1;
        act5->cost = (int) (dCostMult * sqrtf((float)(act5->d_[0]*act5->d_[0] + act5->d_[1]*act5->d_[1])));
        act5->aid = INVALID_THETA_;
        LERP(act5, 10);

        tAction* act6 = new tAction();
        act6->d_.resize(2, 0);
        act6->d_[0] = 1; act6->d_[1] = -1;
        act6->cost = (int) (dCostMult * sqrtf((float)(act6->d_[0]*act6->d_[0] + act6->d_[1]*act6->d_[1])));
        act6->aid = INVALID_THETA_;
        LERP(act6, 10);

        tAction* act7 = new tAction();
        act7->d_.resize(2, 0);
        act7->d_[0] = 1; act7->d_[1] = 0;
        act7->cost = (int) (dCostMult * sqrtf((float)(act7->d_[0]*act7->d_[0] + act7->d_[1]*act7->d_[1])));
        act7->aid = INVALID_THETA_;
        LERP(act7, 10);

        tAction* act8 = new tAction();
        act8->d_.resize(2, 0);
        act8->d_[0] = 1; act8->d_[1] = 1;
        act8->cost = (int) (dCostMult * sqrtf((float)(act8->d_[0]*act8->d_[0] + act8->d_[1]*act8->d_[1])));
        act8->aid = INVALID_THETA_;
        LERP(act8, 10);

        actions_[INVALID_THETA_].push_back(act1);
        actions_[INVALID_THETA_].push_back(act2);
        actions_[INVALID_THETA_].push_back(act3);
        actions_[INVALID_THETA_].push_back(act4);
        actions_[INVALID_THETA_].push_back(act5);
        actions_[INVALID_THETA_].push_back(act6);
        actions_[INVALID_THETA_].push_back(act7);
        actions_[INVALID_THETA_].push_back(act8);
        return true;
    }

    int COORD2INDEX(std::vector<short int> coords){
        return (coords[2] + coords[0]*(INVALID_THETA_+1) + coords[1]*EnvNAVXYTHETALATCfg.EnvWidth_c*(INVALID_THETA_+1));
    }
};

#endif

