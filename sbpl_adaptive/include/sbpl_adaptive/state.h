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

#define STATE_CAST_DEBUG 1

struct AdaptiveState
{
#if STATE_CAST_DEBUG
  // virtual destructor to force dynamic type information
  virtual ~AdaptiveState() { }
#endif
};

struct AdaptiveHashEntry
{
    size_t stateID;           // the state ID
    size_t heur;              // the heuristic value
    char dimID;               // the dimensionality ID
    AdaptiveState *stateData; // the state data specific to this dimensionality

    template <class T>
    T *dataAs()
    {
#if STATE_CAST_DEBUG
        T *state = dynamic_cast<T *>(stateData);
        if (!state) {
          throw std::runtime_error("bad cast");
        }
        return state;
#else
        return static_cast<T *>(stateData);
#endif
    }

    template <class T>
    const T *dataAs() const
    {
#if STATE_CAST_DEBUG
        const T *state = dynamic_cast<const T *>(stateData);
        if (!state) {
          throw std::runtime_error("bad cast");
        }
        return state;
#else
        return static_cast<const T *>(stateData);
#endif
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
