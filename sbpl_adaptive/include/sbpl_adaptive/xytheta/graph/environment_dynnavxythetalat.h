#ifndef SBPL_ADAPTIVE_ENVIRONMENT_DYNNAVXYTHETALAT_H
#define SBPL_ADAPTIVE_ENVIRONMENT_DYNNAVXYTHETALAT_H

#include <stdio.h>
#include <vector>

#include <sbpl/headers.h>

#define SIM_VISIBILITY
#define SIM_DISTANCE

#define SIM_DIST_RAD 1.0

class DynamicEnvironmentNAVXYTHETALATTICE : public EnvironmentNAVXYTHETALAT
{
public:

    DynamicEnvironmentNAVXYTHETALATTICE();

    virtual bool SimDetect(int currStateID, double det_rad) = 0; //detects / updates obstacle grid and modified states

    std::vector<int> *GetModifiedStates() { return &ModifiedStateIDs; };

    void ReadMapData(FILE *fCfg);
    void StoreMapData(const char *fname);
    void GenerateRandomMap(double prob_free, int size1, double prob_obs, int size2);

    bool DynInitializeEnv(
        const char *sEnvFile,
        const std::vector<sbpl_2Dpt_t> &perimeterptsV,
        const char *sMotPrimFile,
        const char *sMapFile);

protected:

    //Grid2D contains belief values at first
    //Grid2D will be updated with correct / detected values
    unsigned char **bDetected2D;
    unsigned char **TrueGrid2D; //contains the true environment data (for the simulation)
    unsigned char **expandsGrid;
    std::vector<int> ModifiedStateIDs;

    void ComputeModifiedCells(int x, int y, std::vector<nav2dcell_t> *modCells);
};
#endif
