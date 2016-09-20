/*
 * state.h
 *
 *  Created on: Jan 28, 2016
 *      Author: kalin
 */

#ifndef ADAPTIVE_PLANNING_SBPL_HUMANOID_PLANNER_INCLUDE_SBPL_HUMANOID_PLANNER_STATE_H_
#define ADAPTIVE_PLANNING_SBPL_HUMANOID_PLANNER_INCLUDE_SBPL_HUMANOID_PLANNER_STATE_H_

#include <tf/tf.h>

namespace adim {

typedef struct {
    size_t stateID;         //the state ID
    size_t heur;            //the heuristic value
    char dimID;             //the dimensionality ID
    void* stateData;        //the state data specific to this dimensionality
} AdaptiveHashEntry_t;

inline static size_t intHash(size_t key)
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

inline static size_t dblHash(double d, double res){
    return intHash((d / res));
}

}

#endif /* ADAPTIVE_PLANNING_SBPL_HUMANOID_PLANNER_INCLUDE_SBPL_HUMANOID_PLANNER_STATE_H_ */
