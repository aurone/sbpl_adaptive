#ifndef SBPL_ADAPTIVE_ENVIRONMENT_MHA_H
#define SBPL_ADAPTIVE_ENVIRONMENT_MHA_H

// system includes
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/utils.h>

class EnvironmentMHA : public DiscreteSpaceInformation
{
public:

    using DiscreteSpaceInformation::GetFromToHeuristic;
    using DiscreteSpaceInformation::GetGoalHeuristic;
    using DiscreteSpaceInformation::GetStartHeuristic;

    /**
     * \brief heuristic estimate from state FromStateID to state ToStateID
     */
    virtual int GetFromToHeuristic(int q_id, int FromStateID, int ToStateID)
    {
      return 0;
    }

    /**
     * \brief heuristic estimate from state with stateID to goal state
     */
    virtual int GetGoalHeuristic(int q_id, int stateID)
    {
      return 0;
    }

    /**
     * \brief heuristic estimate from start state to state with stateID
     */
    virtual int GetStartHeuristic(int q_id, int stateID) 
    {
      return 0;
    }

    /**
     * \brief GetSuccs methods that inform the environment which queue the states are being expanded from
     */
    using DiscreteSpaceInformation::GetSuccs;
    using DiscreteSpaceInformation::GetPreds;
    using DiscreteSpaceInformation::GetLazySuccs;
    using DiscreteSpaceInformation::GetLazyPreds;
    using DiscreteSpaceInformation::GetTrueCost;

    virtual void GetSuccs(int q_id, int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV) 
    {
      SBPL_ERROR("ERROR: GetSuccs with q_id is not implemented for this environment!\n");
      throw new SBPL_Exception();
    }

    virtual void GetPreds(int q_id, int SourceStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) 
    {
      SBPL_ERROR("ERROR: GetPreds with q_id is not implemented for this environment!\n");
      throw new SBPL_Exception();
    }

    virtual void GetLazySuccs(int q_id, int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
      SBPL_ERROR("ERROR: GetLazySuccs with q_id is not implemented for this environment!\n");
      throw new SBPL_Exception();
    };

    virtual void GetLazyPreds(int q_id, int SourceStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
      SBPL_ERROR("ERROR: GetLazyPreds with q_id is not implemented for this environment!\n");
      throw new SBPL_Exception();
    };

    virtual int GetTrueCost(int q_id, int parentID, int childID){
      SBPL_ERROR("ERROR: GetTrueCost with q_id is not implemented for this environment!\n");
      throw new SBPL_Exception();
      return -1;
    };
};

#endif

