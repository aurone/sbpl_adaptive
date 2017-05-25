#ifndef SBPL_ADAPTIVE_STATE_H
#define SBPL_ADAPTIVE_STATE_H

// standard includes
#include <stdlib.h>
#include <memory>
#include <stdexcept>

// system includes
#include <smpl/forward.h>

namespace adim {

#ifndef STATE_CAST_DEBUG
#define STATE_CAST_DEBUG 0
#endif

SBPL_CLASS_FORWARD(AdaptiveState);

/// Base class for a discrete state in a given Representation
struct AdaptiveState
{
#if STATE_CAST_DEBUG
    // virtual destructor to force dynamic type information
    virtual ~AdaptiveState() { }
#endif
};

SBPL_CLASS_FORWARD(ModelCoords);

/// Base class for the continuous state represented by a discrete state in a
/// given Representation
struct ModelCoords
{
#if STATE_CAST_DEBUG
    // virtual destructor to force dynamic type information
    virtual ~ModelCoords() { }
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

template <typename T>
T *coords_cast(ModelCoords *coords)
{
#if STATE_CAST_DEBUG
    T *s = dynamic_cast<T *>(coords);
    if (!s) {
        throw std::runtime_error("bad cast");
    }
    return s;
#else
    return static_cast<T *>(coords);
#endif
}

template <typename T>
const T *coords_cast(const ModelCoords *coords)
{
    return coords_cast<T>(const_cast<ModelCoords *>(coords));
}

struct AdaptiveHashEntry
{
    int stateID;              // the state ID
    int dimID;                // the representation ID
    AdaptiveState *stateData; // the state data specific to this dimensionality

    template <class T>
    T *dataAs() { return state_cast<T>(stateData); }

    template <class T>
    const T *dataAs() const { return state_cast<T>(stateData); }
};

struct TrajectoryWaypoint
{
    int dimID;
    std::shared_ptr<ModelCoords> coords;
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
