/*
 * state.h
 *
 *  Created on: Jan 28, 2016
 *      Author: kalin
 */

#ifndef SBPL_ADAPTIVE_STATE_H
#define SBPL_ADAPTIVE_STATE_H

#include <stdlib.h>
#include <stdexcept>

namespace adim {

#define STATE_CAST_DEBUG 0

struct AdaptiveState
{
#if STATE_CAST_DEBUG
  // virtual destructor to force dynamic type information
  virtual ~AdaptiveState() { }
#endif
};

template <typename T>
T *state_cast(AdaptiveState *state)
{
#if STATE_CAST_DEBUG
    T *s = dynamic_cast<T *>(state);
    if (!s) {
        throw std::runtime_error("bad cast");
    }
    return s;
#else
    return static_cast<T *>(state);
#endif
}

template <typename T>
const T *state_cast(const AdaptiveState *state)
{
    return state_cast<T>(const_cast<AdaptiveState *>(state));
}

struct AdaptiveHashEntry
{
    size_t stateID;           // the state ID
    size_t heur;              // the heuristic value
    char dimID;               // the dimensionality ID
    AdaptiveState *stateData; // the state data specific to this dimensionality

    template <class T>
    T *dataAs() {
        return state_cast<T>(stateData);
    }

    template <class T>
    const T *dataAs() const {
        return state_cast<T>(const_cast<const AdaptiveState *>(stateData));
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
