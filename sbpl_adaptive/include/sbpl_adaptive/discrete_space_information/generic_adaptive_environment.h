#ifndef __GENERIC_ADAPTIVE_ENVIRONMENT_H_
#define __GENERIC_ADAPTIVE_ENVIRONMENT_H_

#define STATEGRID_FAR  2
#define STATEGRID_NEAR 1
#define STATEGRID_IN   0
#define STATEGRID_DEFAULT 10

namespace adim {

struct tAction
{
    int aid;
    std::vector<short int> d_;
    std::vector<std::vector<double>> pts;
    int cost;
};

struct EnvHashEntry_t
{
    int stateID;                    // hash entry ID number
    std::vector<short int> coord;   // coordinates
};

/*typedef struct
 {
 unsigned char sphereState; //0 means in sphere, 1 means near sphere, 2 means outside sphere
 unsigned char stateInfo; //0th bit set - obstacle, 1st bit set - low-D expansion, 2nd bit set - high-D expansion, 3rd bit set - tracking expansion
 float distance;
 float sphereDist;
 short int sphereIndex;
 } tGridCell;*/

class GenericCollisionChecker
{
public:

    GenericCollisionChecker(DiscreteSpaceInformation* env) : env_(env) { }

    virtual ~GenericCollisionChecker() { }

    virtual bool isValidTransition(std::vector<double> coords, tAction *action)
    {
        return true;
        throw new SBPL_Exception();
        return false;
    }

    virtual bool isValidState(std::vector<double> coords)
    {
        return true;
        throw new SBPL_Exception();
        return false;
    }

private:

    DiscreteSpaceInformation *env_;
};

class GenericAdaptiveEnvironment : public AdaptiveDiscreteSpaceInformation
{
public:

    /** \brief constructor - specify the high-dim size and low-dim size
     */
    GenericAdaptiveEnvironment()
    {
        /*SBPL_ERROR("Trying to use default AdaptiveDiscreteSpaceInformation constructor!");
         throw new SBPL_Exception();*/
    }

    /** \brief destructor
     */
    ~GenericAdaptiveEnvironment()
    {
        spheres_.clear();
        if (l_cspace_ != NULL)
            delete l_cspace_;
        if (h_cspace_ != NULL)
            delete h_cspace_;
    }

    void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
    {
        int start_t = clock();
        if (trackMode) {
            GetPreds_Track(TargetStateID, PredIDV, CostV);
        }
        else {
            GetPreds_Plan(TargetStateID, PredIDV, CostV);
        }
        getPredTime += (clock() - start_t) / (double)CLOCKS_PER_SEC;
    }

    void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
    {
        int start_t = clock();
        if (trackMode) {
            GetSuccs_Track(SourceStateID, SuccIDV, CostV);
        }
        else {
            GetSuccs_Plan(SourceStateID, SuccIDV, CostV);
        }
        getSuccTime += (clock() - start_t) / (double)CLOCKS_PER_SEC;
    }

protected:

    double getSuccTime;
    double getPredTime;

    GenericCollisionChecker* l_cspace_;
    GenericCollisionChecker* h_cspace_;
    std::vector<short int> h_angle_mask_;
    std::vector<short int> l_angle_mask_;

    /* the following functions will have to be defined base on the specific environment requirements */

    virtual EnvHashEntry_t* GetHashEntry(int StateID) = 0;
    virtual bool ExistsHashEntry(std::vector<short int> coord) = 0;
    virtual EnvHashEntry_t* CreateHashEntry(
        std::vector<short int> coord,
        tAction* action) = 0;
    virtual EnvHashEntry_t* GetHashEntryFromCoord(
        std::vector<short int> coord) = 0;

    virtual void coords_disc2cont_h(
        std::vector<short int> *in,
        std::vector<double> *out) = 0;
    virtual void coords_disc2cont_l(
        std::vector<short int> *in,
        std::vector<double> *out) = 0;
    virtual char lookup_stategrid(std::vector<short int> *coords) = 0;
    virtual void projectToLowDCoords(std::vector<short int> *h_coords) = 0;
    virtual void projectToHighD(int StateID, std::vector<int> *StateIDsV) = 0;
    virtual bool isValidLowDState(std::vector<short int> *coords) = 0;
    virtual bool isValidHighDState(std::vector<short int> *coords) = 0;

    virtual std::vector<tAction*>* getActionsForState(
        EnvHashEntry_t* state) = 0;

    /** \brief gets the high-dimensional successors of a state
     */
    void GetSuccs_HighD(
        int SourceStateID,
        vector<int>* SuccIDV,
        vector<int>* CostV)
    {
        EnvHashEntry_t* SourceState = GetHashEntry(SourceStateID);
        if (SourceState == NULL)
            return;
        std::vector<double> SourceStateCoords(SourceState->coord.size(), 0);
        coords_disc2cont_h(&(SourceState->coord), &SourceStateCoords);
        std::vector<tAction*>* h_actions_ = getActionsForState(SourceState);

        //loop through all high-d actions
        for (unsigned int aid = 0; aid < h_actions_->size(); aid++) {
            //collision check action
            if (!h_cspace_->isValidTransition(SourceStateCoords,
                    h_actions_->at(aid)))
                continue;
            //compute new state coords
            std::vector<short int> succ_coords(SourceState->coord);
            for (unsigned int cid = 0; cid < h_actions_->at(aid)->d_.size();
                    cid++) {
                succ_coords[cid] = SourceState->coord[cid]
                        + h_actions_->at(aid)->d_[cid];
                //the following takes care of circular angular values (modulo arithmetic)
                if (h_angle_mask_[cid] > 0) {
                    while (succ_coords[cid] < 0) {
                        succ_coords[cid] += h_angle_mask_[cid];
                    }
                    succ_coords[cid] = succ_coords[cid] % h_angle_mask_[cid];
                }
            }
            //check if this is a valid high-D transition
            bool project_to_lowD = false;
            if (!isValidHighDState(&succ_coords)) {
                if (trackMode) {
                    continue;
                }
                else {
                    project_to_lowD = true;
                }
            }
            //generate new state hash entry if needed
            if (project_to_lowD) {
                projectToLowDCoords(&succ_coords); //makes succ_coords the coordinates of the
            }
            EnvHashEntry_t* SuccState = GetHashEntryFromCoord(succ_coords);
            if (SuccState == NULL) {
                SuccState = CreateHashEntry(succ_coords, h_actions_->at(aid));
            }
            SuccIDV->push_back(SuccState->stateID);
            CostV->push_back(h_actions_->at(aid)->cost);
        }
    }

    /** \brief gets the low-dimensional successors of a state
     */
    void GetSuccs_LowD(
        int SourceStateID,
        vector<int>* SuccIDV,
        vector<int>* CostV)
    {
        EnvHashEntry_t* SourceState = GetHashEntry(SourceStateID);
        if (SourceState == NULL)
            return;
        std::vector<double> SourceStateCoords(SourceState->coord.size(), 0);
        coords_disc2cont_l(&(SourceState->coord), &SourceStateCoords);
        //loop through all low-d actions
        std::vector<tAction*> *l_actions_ = getActionsForState(SourceState);
        for (unsigned int aid = 0; aid < l_actions_->size(); aid++) {
            //compute new state coords
            std::vector<short int> succ_coords(SourceState->coord);
            for (unsigned int cid = 0; cid < l_actions_->at(aid)->d_.size();
                    cid++) {
                succ_coords[cid] = SourceState->coord[cid]
                        + l_actions_->at(aid)->d_[cid];
                //the following takes care of circular angular values (modulo arithmetic)
                if (l_angle_mask_[cid] > 0) {
                    while (succ_coords[cid] < 0) {
                        succ_coords[cid] += l_angle_mask_[cid];
                    }
                    succ_coords[cid] = succ_coords[cid] % l_angle_mask_[cid];
                }
            }
            //check if this is a valid tracking transition
            if (!isValidLowDState(&succ_coords))
                continue;
            //collision check action
            if (!l_cspace_->isValidTransition(SourceStateCoords,
                    l_actions_->at(aid)))
                continue;
            //generate new state hash entry if needed
            EnvHashEntry_t* SuccState = GetHashEntryFromCoord(succ_coords);
            if (SuccState == NULL) {
                SuccState = CreateHashEntry(succ_coords, l_actions_->at(aid));
            }
            SuccIDV->push_back(SuccState->stateID);
            CostV->push_back(l_actions_->at(aid)->cost);
        }
    }

    /** \brief gets the valid low- and high-dimensional successors of a state that is near a sphere
     */
    void GetSuccs_LowDNear(
        int SourceStateID,
        vector<int>* SuccIDV,
        vector<int>* CostV)
    {
        //get the valid low-d successors
        GetSuccs_LowD(SourceStateID, SuccIDV, CostV);

        //then get the valid high-d successors of all high-d projections of SourceStateID
        std::vector<int> HighDImages;
        projectToHighD(SourceStateID, &HighDImages);
        for (unsigned int i = 0; i < HighDImages.size(); i++) {
            GetSuccs_HighD(HighDImages[i], SuccIDV, CostV);
        }
    }

    /** \brief gets successors for tracking mode
     */
    void GetSuccs_Track(
        int SourceStateID,
        vector<int>* SuccIDV,
        vector<int>* CostV)
    {
        if (!trackMode) {
            SBPL_ERROR(
                    "GetSuccs_Track called while environment in planning mode");
            throw new SBPL_Exception();
        }
        GetSuccs_HighD(SourceStateID, SuccIDV, CostV);
    }

    /** \brief gets successors for planning mode
     */
    void GetSuccs_Plan(
        int SourceStateID,
        vector<int>* SuccIDV,
        vector<int>* CostV)
    {
        if (trackMode) {
            SBPL_ERROR(
                    "GetSuccs_Plan called while environment in tracking mode");
            throw new SBPL_Exception();
        }
        EnvHashEntry_t* SourceState = GetHashEntry(SourceStateID);
        std::vector<double> SourceStateCoords(SourceState->coord.size(), 0);
        coords_disc2cont_h(&(SourceState->coord), &SourceStateCoords);

        char stategrid = lookup_stategrid(&SourceState->coord);

        if (stategrid == STATEGRID_FAR) {
            //state is far from all spheres
            GetSuccs_LowD(SourceStateID, SuccIDV, CostV);
        }
        else if (stategrid == STATEGRID_NEAR) {
            //state is near a sphere
            GetSuccs_LowDNear(SourceStateID, SuccIDV, CostV);
        }
        else if (stategrid == STATEGRID_IN) {
            //state is inside a sphere
            GetSuccs_HighD(SourceStateID, SuccIDV, CostV);
        }
    }
};

} // namespace adim

#endif
