/*
 * representation.h
 *
 *  Created on: Feb 8, 2016
 *      Author: kalin
 */

#ifndef _ADAPTIVE_REPRESENTATION_H_
#define _ADAPTIVE_REPRESENTATION_H_

namespace adim {

class MultiRepAdaptiveDiscreteSpaceInformation;

class AdaptiveStateRepresentation_t {
public:
    inline AdaptiveStateRepresentation_t(std::shared_ptr<MultiRepAdaptiveDiscreteSpaceInformation> env, bool executable, std::string description) :
        env_(env),
        dimID_(-1),
        bExecutable_(executable),
        sDescription_(description)

    {
        sphere_radius_ = 1.0;
        near_radius_ = 1.0;
        tunnel_radius_ = 1.0;
    }

    virtual inline ~AdaptiveStateRepresentation_t() { };

    virtual int SetStartCoords(const void *disc_data) = 0; //returns the state ID of the created start state

    virtual int SetStartConfig(const void* cont_data) = 0;

    virtual int SetGoalCoords(const void *disc_data) = 0; //returns the state ID of the created start state

    virtual int SetGoalConfig(const void* cont_data) = 0;

    virtual bool isGoalState(int StateID) const = 0;

    virtual void GetSuccs(int stateID, std::vector<int> *SuccV, std::vector<int> *CostV, const void *env_data) = 0;

    virtual void GetPreds(int stateID, std::vector<int> *PredV, std::vector<int> *CostV, const void *env_data) = 0;

    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut=stdout) const = 0;

    virtual void PrintStateData(const void* state_data, bool bVerbose, FILE* fOut=stdout) const = 0;

    virtual int GetGoalHeuristic(int stateID) const =0;

    virtual void VisualizeState(int stateID, int hue, std::string ns, int &viz_idx) const =0;

    virtual bool IsValidStateData(const void* disc_data) const = 0;

    virtual bool IsValidConfig(const void* cont_data) const = 0;

    inline void setFullDRepresentation(std::shared_ptr<AdaptiveStateRepresentation_t> fullD_rep){
        fullD_rep_ = fullD_rep;
    }

    inline bool isExecutable() const {
        return bExecutable_;
    }

    inline int getID() const {
        return dimID_;
    }

    inline void setID(int id){
        dimID_ = id;
    }

    inline const std::string getDescription() const {
        return sDescription_;
    }

    inline void addParentRepresentation(AdaptiveStateRepresentation_t* parent){
        bool found = false;
        for(auto rep : parents){
            if(rep.get() == parent){
                found = true;
                break;
            }
        }
        if(!found){
            SBPL_INFO("Added %s as parent representation to %s!", parent->getDescription().c_str(), this->getDescription().c_str());
            parents.push_back(std::shared_ptr<AdaptiveStateRepresentation_t>(parent));
            parent->addChildRepresentation(this);
        }
    }

    inline void addChildRepresentation(AdaptiveStateRepresentation_t* child){
        bool found = false;
        for(auto rep : children){
            if(rep.get() == child){
                found = true;
                break;
            }
        }
        if(!found){
            SBPL_INFO("Added %s as child representation to %s!", child->getDescription().c_str(), this->getDescription().c_str());
            children.push_back(std::shared_ptr<AdaptiveStateRepresentation_t>(child));
            child->addParentRepresentation(this);
        }
    }

    void GetExecutableParents(std::vector<const adim::AdaptiveStateRepresentation_t*> &executableParents) const {
        if(isExecutable()){
            executableParents.push_back(this);
        } else {
            for(auto parent : parents){
                parent->GetExecutableParents(executableParents);
            }
        }
    }

    virtual bool ProjectToFullD(const void* ld_state_data, std::vector<int> &hd_projStateIDs, int adPathIdx=0) = 0;

    virtual bool ProjectFromFullD(const void* hd_state_data, std::vector<int> &ld_projStateIDs, int adPathIdx=0) = 0;

    void getParentIDs(int stateID, std::vector<int> &IDs) const {
        for(auto parent : parents){
            IDs.push_back(parent->getID());
        }
    }

    void getChildIDs(int stateID, std::vector<int> &IDs) const {
        for(auto child : children){
            IDs.push_back(child->getID());
        }
    }

    virtual void deleteStateData(int stateID) = 0; //free the stateData ptr in the AdaptiveHashEntry

    virtual void toCont(const void* disc_data, void* cont_data) const = 0;

    virtual void toDisc(const void* cont_data, void* disc_data) const = 0;

    inline double getSphereRadius() const { return sphere_radius_; }

    inline double getNearRadius() const { return near_radius_; }

    inline double getTunnelRadius() const { return tunnel_radius_; }

protected:
    double sphere_radius_;
    double near_radius_;
    double tunnel_radius_;

    std::shared_ptr<MultiRepAdaptiveDiscreteSpaceInformation> env_;
    int dimID_;
    bool bExecutable_;
    std::string sDescription_;
    std::shared_ptr<AdaptiveStateRepresentation_t> fullD_rep_;

    //these form the abstraction hierarchy
    std::vector<std::shared_ptr<AdaptiveStateRepresentation_t>> parents; //less abstract representations
    std::vector<std::shared_ptr<AdaptiveStateRepresentation_t>> children; //more abstract representations
};

}


#endif /* ADAPTIVE_PLANNING_SBPL_HUMANOID_PLANNER_INCLUDE_REPRESENTATION_H_ */
