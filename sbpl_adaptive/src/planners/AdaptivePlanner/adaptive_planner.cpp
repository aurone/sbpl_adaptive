
#include <sbpl_adaptive/headers.h>

#define TRACK_RETRIES 0

using namespace std;

//====================================== PUBLIC ====================================================
namespace sbpl_adaptive {

AdaptivePlanner::AdaptivePlanner(AdaptiveDiscreteSpaceInformation* environment, bool bSearchForward)
{
	logstream_ = stdout;
	SBPL_INFO("Creating adaptive planner...");
	bforwardsearch = bSearchForward;
	adaptive_environment_ = environment;

	StartStateID = environment->StartStateID;
	GoalStateID = environment->GoalStateID;

	final_eps_planning_time = -1.0;
	final_eps = -1.0;

	timePerRetryPlan_ = 5.0;
	timePerRetryTrack_ = 5.0;

	planningEPS = -1.0;
	trackingEPS = -1.0;
	targetEPS = -1.0;

	nIterations = 0;
	plan_time_total_s = 0;
	track_time_total_s = 0;
	bsearchuntilfirstsolution = true;
	repair_time = 0;
	searchexpands = 0;

	stat_.reset(new AdaptivePlannerCSVStat_c());
	//adaptive_environment_->setStat(stat_.get());

	SBPL_INFO("Initializing planners...");

	plan_heur_ = new EmbeddedHeuristic(adaptive_environment_);
	track_heur_ = new EmbeddedHeuristic(adaptive_environment_);
	num_heur_ = 1;

	planner.reset(new MHAPlanner_AD(adaptive_environment_, plan_heur_, &plan_heur_, num_heur_));
	planner->set_search_mode(false);
	tracker.reset(new MHAPlanner_AD(adaptive_environment_, track_heur_, &track_heur_, num_heur_));
	tracker->set_search_mode(false);
	SBPL_INFO("done!");
}


AdaptivePlanner::~AdaptivePlanner()
{

}

//----------------------------- Interface functions ----------------------------------------------

//returns 1 if found a solution, and 0 otherwise
int AdaptivePlanner::replan(double allocated_time_secs, double allocated_time_per_retry_plan_, double allocated_time_per_retry_track_, std::vector<int>* solution_stateIDs_V, int* psolcost)
{
	SBPL_INFO("Adaptive planning begins...");
	adaptive_environment_->setPlanMode();
	//DO THE ADAPTIVE PLANNING LOOP HERE
	std::vector<int> planning_stateV;
	std::vector<int> tracking_stateV;

	MY_TIME_TYPE start_t = MY_TIME_NOW;
	track_time_total_s = 0;
	plan_time_total_s = 0;

	int round = 0;

	double ad_plan_time_alloc = allocated_time_per_retry_plan_;
	double ad_track_time_alloc = allocated_time_per_retry_track_;

	SBPL_INFO("Retry time limits: (Planning: %.4f) (Tracking: %.4f)", ad_plan_time_alloc, ad_track_time_alloc);

	int planning_bRet;
	int tracking_bRet;

	SBPL_INFO("Setting start and goal for planner and tracker...");

	if(planner->set_start(StartStateID) == 0)
	{
		SBPL_ERROR("ERROR: planner failed to set start state");
		throw SBPL_Exception();
	}

	if(tracker->set_start(StartStateID) == 0){
		SBPL_ERROR("ERROR: tracker failed to set start state");
		throw SBPL_Exception();
	}

	if(planner->set_goal(GoalStateID) == 0)
	{
		SBPL_ERROR("ERROR: planner failed to set goal state");
		throw SBPL_Exception();
	}

	if(tracker->set_goal(GoalStateID) == 0){
		SBPL_ERROR("ERROR: tracker failed to set goal state");
		throw SBPL_Exception();
	}

	SBPL_INFO("Setting environment in planning mode...");
	adaptive_environment_->setPlanMode();

	SBPL_INFO("Adding start (%d) and goal (%d) spheres...", StartStateID, GoalStateID);
	adaptive_environment_->addSphere(GoalStateID);
	adaptive_environment_->addSphere(StartStateID);

	std::vector<int> ModifiedStates;
	std::vector<int> TrkModifiedStates;
	repair_time = 0.0;

	SBPL_INFO("Forcing planning from scratch for planner and tracker...");
	planner->force_planning_from_scratch();
	tracker->force_planning_from_scratch();

	std::vector<int> LastTrackPath;
	bool LastTrackSuccess = false;
	double t_elapsed_s;

	stat_->setInitialEps(planningEPS*trackingEPS);

	std::vector<int> new_sphere_locations; //as stateIDs
	do {
		if(MY_TIME_DIFF_S(MY_TIME_NOW, start_t) > allocated_time_secs){
			SBPL_INFO("Search ran out of time!");
			solution_stateIDs_V->clear();
			for(unsigned int i = 0; i < LastTrackPath.size(); i++){
				solution_stateIDs_V->push_back(LastTrackPath[i]);
			}
			SBPL_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			nIterations = round;

			stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			stat_->setFinalEps(-1.0);
			return LastTrackSuccess;
		}

		SBPL_INFO("=======================================");
		SBPL_INFO("||          Iteration %03d            ||", round);
		SBPL_INFO("=======================================");
		MY_TIME_TYPE iter_start = MY_TIME_NOW;
		MY_TIME_TYPE plan_start = MY_TIME_NOW;
		MY_TIME_TYPE track_start;

		double plan_time_s = 0;
		double track_time_s = 0;

		int p_Cost;		  //planning solution cost
		int t_Cost;		  //tracking solution cost

		//==================================== PLANNING =================================
		SBPL_INFO("\t=======================================");
		SBPL_INFO("\t||          Planning Phase           ||");
		SBPL_INFO("\t=======================================");

		planner->force_planning_from_scratch();
		planner->set_initialsolution_eps(planningEPS);
		planner->set_search_mode(false);

		adaptive_environment_->setPlanMode();
		//add pending new spheres
		SBPL_INFO("Adding %zd pending spheres...", new_sphere_locations.size());
		for(int stateID : new_sphere_locations){
			adaptive_environment_->addSphere(stateID, &ModifiedStates);
		}
		new_sphere_locations.clear();
		planning_stateV.clear();
		planning_bRet = 0;

		//PLANNING HERE!
		t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
		while(t_elapsed_s < allocated_time_secs){
			SBPL_INFO("Still have time (%.3fs)...planning", allocated_time_secs - t_elapsed_s);
			MY_TIME_TYPE p_start = MY_TIME_NOW;
			planning_bRet = planner->replan(ad_plan_time_alloc, &planning_stateV, &p_Cost);
			t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
			if(!planning_bRet && MY_TIME_DIFF_S(MY_TIME_NOW, p_start) < 0.1*ad_plan_time_alloc){
			    SBPL_WARN("Planning too quick! Probably stuck!");
			    break;
			}
			if(planning_bRet) break;
		}

		plan_time_s = MY_TIME_DIFF_S(MY_TIME_NOW, plan_start);
		plan_time_total_s += plan_time_s;
		stat_->addPlanningPhaseTime(plan_time_s);

		SBPL_INFO("Planner done in %.3fs...", plan_time_s);

		if(!planning_bRet || planning_stateV.size() == 0){
			SBPL_ERROR("Solution could not be found within the allowed time (%.3fs.) after %d iterations", ad_plan_time_alloc, round);
			#ifdef ADP_GRAPHICAL
			adaptive_environment_->visualizeEnvironment();
			#endif
			nIterations = round;
			stat_->setFinalEps(-1.0);
			stat_->setSuccess(false);
			stat_->setNumIterations(nIterations+1);
			return planning_bRet;
		}
		if(adaptive_environment_->isExecutablePath(planning_stateV)){
			solution_stateIDs_V->clear();
			for(unsigned int i = 0; i < planning_stateV.size(); i++){
				solution_stateIDs_V->push_back(planning_stateV[i]);
			}
			SBPL_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
			SBPL_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			nIterations = round;
			stat_->setFinalEps(planner->get_final_epsilon());
			stat_->setFinalPlanCost(p_Cost);
			stat_->setFinalTrackCost(p_Cost);
			stat_->setNumIterations(nIterations+1);
			stat_->setSuccess(true);
			stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			return true;
		}
		#ifdef ADP_GRAPHICAL
		adaptive_environment_->visualizeEnvironment();
		adaptive_environment_->visualizeStatePath(&planning_stateV, 0, 120, "planning_path");
		#endif

		//==================================== TRACKING ====================================
		SBPL_INFO("\t=======================================");
		SBPL_INFO("\t||          Tracking Phase           ||");
		SBPL_INFO("\t=======================================");
		track_start = MY_TIME_NOW;
		adaptive_environment_->setTrackMode(planning_stateV, p_Cost, &TrkModifiedStates);
		tracker->force_planning_from_scratch();
		tracker->set_initialsolution_eps(targetEPS / planner->get_final_epsilon());
		tracker->set_search_mode(true);
		tracking_stateV.clear();
		tracking_bRet = 0;

		//TRACKING HERE!!!
		t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
		int last_bestTrackedID = -1;
		int retries = 0;
		while(t_elapsed_s < allocated_time_secs){
			SBPL_INFO("Still have time (%.3fs)...tracking", allocated_time_secs - t_elapsed_s);
			MY_TIME_TYPE p_start = MY_TIME_NOW;
			tracking_bRet = tracker->replan(ad_track_time_alloc, &tracking_stateV, &t_Cost);
			t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
            if(!tracking_bRet && MY_TIME_DIFF_S(MY_TIME_NOW, p_start) < 0.1*ad_track_time_alloc){
                SBPL_WARN("Tracking too quick! Probably stuck!");
                break;
            }

			if(tracking_bRet) {
				break;
			} else {
				int new_bestTrackedID = adaptive_environment_->getBestSeenState();
				if(new_bestTrackedID != -1 && new_bestTrackedID == last_bestTrackedID){
					if(retries>=TRACK_RETRIES){
						SBPL_WARN("No progress...giving up this tracking iteration");
						break;
					}
					retries++;
				} else {
					last_bestTrackedID = new_bestTrackedID;
					retries=0;
				}
			}
		}

		track_time_s = MY_TIME_DIFF_S(MY_TIME_NOW, track_start);
		track_time_total_s += track_time_s;
		stat_->addTrackingPhaseTime(track_time_s);

		LastTrackPath.clear();
		for(unsigned int i = 0; i < tracking_stateV.size(); i++){
			LastTrackPath.push_back(tracking_stateV[i]);
		}
		LastTrackSuccess = tracking_bRet;

		SBPL_INFO("Tracker done in %.3fs...", track_time_s);

		ModifiedStates.clear();
		SBPL_INFO("[Planning] Time: %.3fs (%.1f%% of iter time)", plan_time_s, 100*plan_time_s / (plan_time_s + track_time_s));
		SBPL_INFO("[Tracking] Time: %.3fs (%.1f%% of iter time)", track_time_s, 100*track_time_s / (plan_time_s + track_time_s));

		#ifdef ADP_GRAPHICAL
		adaptive_environment_->visualizeEnvironment();
		adaptive_environment_->visualizeStatePath(&tracking_stateV, 240, 300, "tracking_path");
		#endif

		if(tracking_bRet && (t_Cost / (1.0f * p_Cost)) > trackingEPS * planningEPS){
			//tracking found a costly path
			SBPL_INFO("Tracking succeeded - costly path found! tCost %d / pCost %d (eps: %.3f, target: %.3f)", t_Cost, p_Cost, (t_Cost/(double)p_Cost), trackingEPS * planningEPS);
			adaptive_environment_->processCostlyPath(planning_stateV, tracking_stateV, &new_sphere_locations);
			if(new_sphere_locations.size() == 0){
				SBPL_ERROR("No new spheres added during this planning episode!!!");
				throw SBPL_Exception();
			}
		} else if (tracking_bRet && (t_Cost / (1.0f * p_Cost)) <= trackingEPS * planningEPS ) {
			SBPL_INFO("Tracking succeeded! - good path found! tCost %d / pCost %d (eps: %.3f, target: %.3f)", t_Cost, p_Cost, (t_Cost/(double)p_Cost), trackingEPS * planningEPS);
			#ifdef ADP_GRAPHICAL
			adaptive_environment_->visualizeStatePath(&tracking_stateV, 0, 240, "tracking_path");
			#endif
			solution_stateIDs_V->clear();
			for(unsigned int i = 0; i < tracking_stateV.size(); i++){
				solution_stateIDs_V->push_back(tracking_stateV[i]);
			}
			SBPL_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
			SBPL_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			nIterations = round;
			stat_->setFinalEps(planner->get_final_epsilon() * tracker->get_final_epsilon());
			stat_->setFinalPlanCost(p_Cost);
			stat_->setFinalTrackCost(t_Cost);
			stat_->setNumIterations(nIterations+1);
			stat_->setSuccess(true);
			stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			return true;
		} else {
			SBPL_WARN("Tracking Failed!");
			//since tracking failed -- introduce new spheres
			if(tracking_stateV.size() > 0){
				//get the point of failure
				int TrackFail_StateID = tracking_stateV[tracking_stateV.size()-1];
				new_sphere_locations.push_back(TrackFail_StateID);
			} else {
				SBPL_ERROR("No new spheres added during this planning episode!!!");
				throw SBPL_Exception();
			}
		}
		SBPL_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
		SBPL_INFO("Total Time so far: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
		round++;
	} while (true);

	stat_->setFinalEps(-1);
	stat_->setFinalPlanCost(INFINITECOST);
	stat_->setFinalTrackCost(INFINITECOST);
	stat_->setNumIterations(round+1);
	stat_->setSuccess(false);
	stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));

	return tracking_bRet;
}


int AdaptivePlanner::set_goal(int goal_stateID)
{
	GoalStateID = goal_stateID;
	SBPL_INFO("goal set (StateID: %d)", goal_stateID);
	return 1;
}


int AdaptivePlanner::set_start(int start_stateID)
{
	StartStateID = start_stateID;
	SBPL_INFO("start set (StateID: %d)", start_stateID);
	return 1;
}


int AdaptivePlanner::force_planning_from_scratch()
{
	if(planner != NULL && tracker != NULL){
		SBPL_INFO("Resetting planner and tracker!");
		planner->force_planning_from_scratch();
		tracker->force_planning_from_scratch();
	}
	adaptive_environment_->reset();
	stat_->reset();
	return 1;
}


int AdaptivePlanner::set_search_mode(bool bSearchUntilFirstSolution)
{
	bsearchuntilfirstsolution = bSearchUntilFirstSolution;
	//set search mode
	planner->set_search_mode(bsearchuntilfirstsolution);
	tracker->set_search_mode(bsearchuntilfirstsolution);
	return 1;
}


void AdaptivePlanner::print_searchpath(FILE* fOut)
{
	SBPL_WARN("print_searchpath() NOT IMPLEMENTED YET!");
	return;
}

bool AdaptivePlanner::set_time_per_retry(double t_plan, double t_track){
	timePerRetryPlan_ = t_plan;
	timePerRetryTrack_ = t_track;
	return true;
}

void AdaptivePlanner::set_initialsolution_eps(double initialsolution_eps){
	targetEPS = initialsolution_eps;
	planningEPS = sqrt(initialsolution_eps);
	trackingEPS = sqrt(initialsolution_eps);
	planner->set_initialsolution_eps(planningEPS);
	tracker->set_initialsolution_eps(trackingEPS);
}

int AdaptivePlanner::replan(vector<int>* solution_stateIDs_V, ReplanParams params, int* solcost){
  set_initialsolution_eps(params.initial_eps);
  bsearchuntilfirstsolution = params.return_first_solution;
  return replan(params.max_time, params.repair_time, params.repair_time, solution_stateIDs_V, solcost);
}

}
