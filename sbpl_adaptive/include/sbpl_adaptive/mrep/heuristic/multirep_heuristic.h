#ifndef SBPL_ADAPTIVE_MULTIREP_HEURISTIC_H
#define SBPL_ADAPTIVE_MULTIREP_HEURISTIC_H

// system includes
#include <sbpl/heuristics/heuristic.h>
#include <smpl/forward.h>

namespace adim {

SBPL_CLASS_FORWARD(MultiRepHeuristic)

class MultiRepHeuristic : public Heuristic
{
public:

    MultiRepHeuristic(MultiRepAdaptiveDiscreteSpace *space) :
        Heuristic(space)
    { }

    virtual bool IsDefinedForRepresentation(int dim_id) const = 0;
};

SBPL_CLASS_FORWARD(MultiRepEmbeddedHeuristic)

class MultiRepEmbeddedHeuristic : public MultiRepHeuristic
{
public:

    MultiRepEmbeddedHeuristic(MultiRepAdaptiveDiscreteSpace *space) :
        MultiRepHeuristic(space),
        space_(space)
    { }

    int GetGoalHeuristic(int state_id)
    { return space_->GetGoalHeuristic(state_id); }

    int GetStartHeuristic(int state_id)
    { return space_->GetStartHeuristic(state_id); }

    int GetFromToHeuristic(int from_id, int to_id)
    { return space_->GetFromToHeuristic(from_id, to_id); }

    bool IsDefinedForRepresentation(int dim_id) const override { return true; }

protected:

    MultiRepAdaptiveDiscreteSpace *space_;
};

} // namespace adim

#endif
