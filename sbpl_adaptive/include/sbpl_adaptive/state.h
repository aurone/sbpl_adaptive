/*
 * state.h
 *
 *  Created on: Jan 28, 2016
 *      Author: kalin
 */

#ifndef SBPL_ADAPTIVE_STATE_H
#define SBPL_ADAPTIVE_STATE_H

#include <stdlib.h>

namespace adim {

struct AdaptiveHashEntry
{
    size_t stateID;         // the state ID
    size_t heur;            // the heuristic value
    char dimID;             // the dimensionality ID
    void *stateData;        // the state data specific to this dimensionality

    template <class T> T *dataAs() {
        return static_cast<T *>(stateData);
    }

    template <class T> const T *dataAs() const {
        return static_cast<const T *>(stateData);
    }
};

inline
size_t intHash(size_t key)
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

inline
size_t dblHash(double d, double res)
{
    return intHash((d / res));
}

} // namespace adim

#endif
