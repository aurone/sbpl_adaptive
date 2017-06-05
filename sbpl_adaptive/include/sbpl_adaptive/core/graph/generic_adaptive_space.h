#ifndef SBPL_ADAPTIVE_GENERIC_ADAPTIVE_ENVIRONMENT_H
#define SBPL_ADAPTIVE_GENERIC_ADAPTIVE_ENVIRONMENT_H

// standard includes
#include <vector>

// system includes
#include <sbpl/headers.h>
#include <smpl/time.h>

// project includes
#include <sbpl_adaptive/core/graph/adaptive_discrete_space.h>

#define STATEGRID_FAR  2
#define STATEGRID_NEAR 1
#define STATEGRID_IN   0
#define STATEGRID_DEFAULT 10

namespace adim {

struct tAction
{
    int aid;                                //
    std::vector<int> d_;                    // state coordinate delta
    std::vector<std::vector<double>> pts;   // sequence of intermediate waypoint deltas
    int cost;
};

struct EnvHashEntry_t
{
    int stateID;              // hash entry ID number
    std::vector<int> coord;   // coordinates
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

    GenericCollisionChecker(DiscreteSpaceInformation* env);

    virtual ~GenericCollisionChecker();

    virtual bool isValidTransition(
        std::vector<double> coords,
        tAction *action) = 0;

    virtual bool isValidState(std::vector<double> coords) = 0;

    DiscreteSpaceInformation *space() { return env_; }

private:

    DiscreteSpaceInformation *env_;
};

class GenericAdaptiveSpace : public AdaptiveDiscreteSpace
{
public:

    GenericAdaptiveSpace(
        GenericCollisionChecker *lo_cspace,
        GenericCollisionChecker *hi_cspace);

    ~GenericAdaptiveSpace();

    /// \name Required Functions from AdaptiveDiscreteSpace
    ///@{
    virtual void addSphere(
        int state_id,
        std::vector<int> *mod_states = NULL) override;

    virtual void processCostlyPath(
        const std::vector<int> &planning_path,
        const std::vector<int> &tracking_path,
        std::vector<int> *new_sphere_locations) override;

    void reset() override;

    void addSphere(int state_id, int &first_mod_step) override;
    ///@}

protected:

    GenericCollisionChecker *l_cspace_;
    GenericCollisionChecker *h_cspace_;
    std::vector<short int> h_angle_mask_;
    std::vector<short int> l_angle_mask_;

    /* the following functions will have to be defined base on the specific environment requirements */

    virtual EnvHashEntry_t* GetHashEntry(int StateID) = 0;
    virtual bool ExistsHashEntry(std::vector<int> coord) = 0;
    virtual EnvHashEntry_t* CreateHashEntry(
        std::vector<int> coord,
        tAction* action) = 0;
    virtual EnvHashEntry_t* GetHashEntryFromCoord(
        std::vector<int> coord) = 0;

    /// Convert discrete state coordinate into continuous state coordinates for
    /// high-dimensional states.
    ///
    /// \param in input discrete state coordinates
    /// \param out output continuous state coordinates
    virtual void coords_disc2cont_h(
        const std::vector<int> *in,
        std::vector<double> *out) = 0;

    /// Convert discrete state coordinates into continuous state coordinates for
    /// low-dimensional states
    ///
    /// \param in input discrete state coordinates
    /// \param out output continuous state coordinates
    virtual void coords_disc2cont_l(
        const std::vector<int> *in,
        std::vector<double> *out) = 0;

    /// Test whether a state is within a high-dimensional region, close to a
    /// high-dimensional region such that a high-dimensinal action from a ...
    /// fuck it
    virtual char lookup_stategrid(std::vector<int> *coords) = 0;

    /// Project a high-dimensional state to its corresponding low-dimensional
    /// state. The input state is modified to result in the low-dimensional
    /// state.
    virtual void projectToLowDCoords(std::vector<int> *h_coords) = 0;

    virtual void projectToHighD(int StateID, std::vector<int> *StateIDsV) = 0;

    virtual bool isValidLowDState(std::vector<int> *coords) = 0;
    virtual bool isValidHighDState(std::vector<int> *coords) = 0;

    virtual std::vector<tAction*>* getActionsForState(
        EnvHashEntry_t* state) = 0;

    void GetSuccs_HighD(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void GetSuccs_LowD(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void GetSuccs_LowDNear(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs);

    /// \name Required Functions from AdaptiveDiscreteSpace
    ///@{
    void GetSuccs_Plan(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void GetSuccs_Track(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs);

    virtual void GetPreds_Plan(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs);

    virtual void GetPreds_Track(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs);

    virtual void GetSuccs_Plan(
        int state_id,
        int expansion_step,
        std::vector<int>* succs,
        std::vector<int>* costs);

    virtual void GetSuccs_Track(
        int state_id,
        int expansion_step,
        std::vector<int>* succs,
        std::vector<int>* costs);

    virtual void GetPreds_Plan(
        int state_id,
        int expansion_step,
        std::vector<int>* preds,
        std::vector<int>* costs);

    virtual void GetPreds_Track(
        int state_id,
        int expansion_step,
        std::vector<int>* preds,
        std::vector<int>* costs);
    ///@}
};

} // namespace adim

#endif
