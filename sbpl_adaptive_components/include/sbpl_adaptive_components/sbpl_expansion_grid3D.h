/*
 * sbpl_TRA_expansion_grid3D.h
 *
 *  Created on: Sep 22, 2015
 *      Author: kalin
 */

#ifndef _SBPL_EXPANSION_GRID3D_H_
#define _SBPL_EXPANSION_GRID3D_H_

#include <sbpl/headers.h>
#include <sbpl_adaptive_components/common.h>
#include <sbpl_adaptive_collision_checking/sbpl_collision_space.h>
#include <visualization_msgs/MarkerArray.h>

namespace adim {

class ExpansionGrid3D
{
private:
	std::shared_ptr<adim::SBPLCollisionSpace> cspace_;

protected:
	Cell3D size_;
	std::vector<std::vector<std::vector<unsigned int>>> expands_grid_;

public:

    /** @brief constructor - uses a pointer to a collision space (with a corresponding collision model used to calculate the voxels for each state) */
	ExpansionGrid3D(adim::SBPLCollisionSpace* cspace);

    /** @brief destructor */
    ~ExpansionGrid3D();

    /** @brief checks if given coords are inside the grid bounds */
    bool inGrid(int x, int y, int z);

    /** @brief resets all expansion counts in the grid */
    inline void reset();

    /** @brief records the expansion step for the specified model coords */
    void setExpansionStep(const adim::ModelCoords &model_coords, unsigned int exp_step);

    /** @brief returns the earliest expansion step for a given set of voxels -- pass in all modified cells in the environment to get the earliest expansion step the changes affect */
    unsigned int getEarliestExpansionStep(std::vector<Cell3D> voxels);

    visualization_msgs::MarkerArray getVoxelVisualization(const adim::ModelCoords &model_coords, std::string ns, Color color);

};

} //namespace

#endif 
