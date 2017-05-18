#ifndef SBPL_ADAPTIVE_EXPANSION_GRID_3D_H
#define SBPL_ADAPTIVE_EXPANSION_GRID_3D_H

// standard includes
#include <string>

// system includes
#include <sbpl_adaptive_collision_checking/sbpl_collision_space.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_adaptive/common.h>

namespace adim {

class ExpansionGrid3D
{
public:

    /// Construct an expansion grid from collision space, with a corresponding
    /// collision model used to calculate the voxels for each state
    ExpansionGrid3D(adim::SBPLCollisionSpace* cspace);

    ~ExpansionGrid3D();

    bool inGrid(int x, int y, int z);

    /// \brief resets all expansion counts in the grid
    inline void reset();

    /// \brief records the expansion step for the specified model coords
    void setExpansionStep(
        const adim::ModelCoords &model_coords,
        unsigned int exp_step);

    /// \brief returns the earliest expansion step for a given set of voxels --
    /// pass in all modified cells in the environment to get the earliest
    /// expansion step the changes affect
    unsigned int getEarliestExpansionStep(const std::vector<Cell3D> &voxels);

    visualization_msgs::MarkerArray getVoxelVisualization(
        const adim::ModelCoords &model_coords,
        std::string ns,
        Color color);

protected:

    Cell3D size_;
    std::vector<std::vector<std::vector<unsigned int>>> expands_grid_;

private:

    SBPLCollisionSpacePtr cspace_;
};

} // namespace adim

#endif
