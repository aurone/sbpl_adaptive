/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __SBPL_ADAPTIVE_HEADERS_H_
#define __SBPL_ADAPTIVE_HEADERS_H_

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <string>

#ifndef ROS
	#define ROS
#endif

#include <sbpl/headers.h>

#include <sbpl_adaptive/SCVStat.h>
#include <sbpl_adaptive/common.h>

#include <sbpl_adaptive/discrete_space_information/adaptive_environment.h>

#include <sbpl_adaptive/state.h>
#include <sbpl_adaptive/representation.h>
#include <sbpl_adaptive/discrete_space_information/multirep_adaptive_environment.h>

#include <sbpl_adaptive/representation3D.h>
#include <sbpl_adaptive/discrete_space_information/multirep_adaptive_3d/multirep_adaptive_3d.h>

#include <sbpl_adaptive/planners/TRAStar/traplanner.h>
#include <sbpl_adaptive/planners/AdaptivePlanner/araplanner_ad.h>
#include <sbpl_adaptive/planners/AdaptivePlanner/mhaplanner_ad.h>
#include <sbpl_adaptive/planners/AdaptivePlanner/improved_mhaplanner_ad.h>
#include <sbpl_adaptive/planners/AdaptivePlanner/adaptive_planner.h>

#endif
