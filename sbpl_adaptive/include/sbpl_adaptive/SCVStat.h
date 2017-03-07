/*
 * SCVStat.h
 *
 *  Created on: Sep 14, 2015
 *      Author: kalin
 */

#ifndef _SBPL_ADAPTIVE_SCVSTAT_H_
#define _SBPL_ADAPTIVE_SCVSTAT_H_

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

    bool writeToFile(const std::string &name)
    {
        if (fileExists(name)) {
            return appendToFile(name);
        }
        else {
            FILE *file = fopen(name.c_str(), "w");
            if (!file) {
                return false;
            }
            fprintf(file, "Success, Initial Eps, Planning Phase Time, Tracking Phase Time, Total Time, Num Iterations, Num Expansions LD, Num Expansions NearHD, Num Expansions HD, Num Expansions Track, Num Expansions Total, Final Plan Cost, Final Track Cost, Final Eps\n");
            fprintf(file, "%s, %.5f, %.5f, %.5f, %.5f, %u, %lu, %lu, %lu, %lu, %lu, %lu, %lu, %.5f\n",
                    (b_success) ? std::string("True").c_str() : std::string("False").c_str(), d_initial_eps, t_planning_phase, t_tracking_phase, t_total, n_iterations, n_expansions_low, n_expansions_near, n_expansions_high, n_expansions_track, n_expansions, cost_plan, cost_track, d_final_eps);
            fclose(file);
            return true;
        }
    }

    bool appendToFile(const std::string &name)
    {
        if (!fileExists(name)) {
            return writeToFile(name);
        }
        else {
            FILE* file = fopen(name.c_str(), "a");
            if (!file) {
                return false;
            }
//            fprintf(file, "Success, Initial Eps, Planning Phase Time, Tracking Phase Time, Total Time, Num Iterations, Num Expansions LD, Num Expansions NearHD, Num Expansions HD, Num Expansions Track, Num Expansions Total, Final Plan Cost, Final Track Cost, Final Eps\n");
            fprintf(file, "%s, %.5f, %.5f, %.5f, %.5f, %u, %lu, %lu, %lu, %lu, %lu, %lu, %lu, %.5f\n",
                    (b_success) ? std::string("True").c_str() : std::string("False").c_str(), d_initial_eps, t_planning_phase, t_tracking_phase, t_total, n_iterations, n_expansions_low, n_expansions_near, n_expansions_high, n_expansions_track, n_expansions, cost_plan, cost_track, d_final_eps);
            fclose(file);
            return true;
        }
    }

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
