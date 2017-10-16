#ifndef SBPL_ADAPTIVE_SCVSTATE_H
#define SBPL_ADAPTIVE_SCVSTATE_H

// standard includes
#include <stdio.h>
#include <string>

// system includes
#include <smpl/forward.h>

SBPL_CLASS_FORWARD(AdaptivePlannerCSVStat_c)

class AdaptivePlannerCSVStat_c
{
public:

    AdaptivePlannerCSVStat_c()
    {
       	n_expansions = 0;
        n_expansions_low = 0;
        n_expansions_high = 0;
        n_expansions_near = 0;
        n_expansions_track = 0;
        n_iterations = 0;
        t_planning_phase = 0;
        t_tracking_phase = 0;
        t_total = 0;
        d_final_eps = 0;
        d_initial_eps = 0;
        cost_plan = 0;
        cost_track = 0;
        b_success = false;
    }

    ~AdaptivePlannerCSVStat_c(){};

    bool fileExists(const std::string &name)
    {
        if (FILE *file = fopen(name.c_str(), "r")) {
            fclose(file);
            return true;
        }
        else {
            return false;
        }
    }

    void recordLDExpansion() { n_expansions_low++; n_expansions++; }
    void recordTrackExpansion() { n_expansions_track++; n_expansions++; }
    void recordHDExpansion() { n_expansions_high++; n_expansions++; }
    void recordNearHDExpansion() { n_expansions_near++; n_expansions++; }
    void addPlanningPhaseTime(double t) { t_planning_phase += t; }
    void addTrackingPhaseTime(double t) { t_tracking_phase += t; }

    void setTotalPlanningTime(double t) { t_total = t; }
    void setInitialEps(double e) { d_initial_eps = e; }
    void setFinalEps(double e) { d_final_eps = e; }
    void setFinalPlanCost(unsigned long c) { cost_plan = c; }
    void setFinalTrackCost(unsigned long c) { cost_track = c; }
    void setSuccess(bool success) { b_success = success; }
    void setNumIterations(int n_iter) { n_iterations = n_iter; }
    void setPlanSize(unsigned long sz) { plan_size = sz; }

    double getPlanningPhaseTime() { return t_planning_phase; }
    double getTrackingPhaseTime() { return t_tracking_phase; }
    double getTotalTime() { return t_total; }
    unsigned long getPlanCost() { return cost_plan; }
    unsigned long getTrackingCost() { return cost_track; }
    unsigned long getPlanSize() { return plan_size; }

    void reset()
    {
        n_expansions = 0;
        n_expansions_low = 0;
        n_expansions_high = 0;
        n_expansions_near = 0;
        n_expansions_track = 0;
        n_iterations = 0;
        t_planning_phase = 0;
        t_tracking_phase = 0;
        t_total = 0;
        d_final_eps = 0;
        d_initial_eps = 0;
        cost_plan = 0;
        cost_track = 0;
        b_success = false;
        plan_size = 0;
    }

private:
//public:

    // number of expansions
    unsigned long n_expansions;
    unsigned long n_expansions_low;
    unsigned long n_expansions_near;
    unsigned long n_expansions_high;
    unsigned long n_expansions_track;

    // planning times
    double t_planning_phase;
    double t_tracking_phase;
    double t_total;

    // num iterations
    unsigned int n_iterations;
    bool b_success;

    // epsilon
    double d_final_eps;
    double d_initial_eps;

    // cost
    unsigned long cost_track;
    unsigned long cost_plan;

    unsigned long plan_size;

    // planner config ???
};

#endif
