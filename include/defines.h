#ifndef DEFINES_H
#define DEFINES_H

#include <obstacle_detector/Point2D.h>

namespace obstacle_detector
{

// Scans to PCL conversion
const bool   OMIT_OVERLAPPING_SCANS = true;
const int    MAX_UNRECEIVED_SCANS = 1;      // If for some reason one of the scanner does not provide # of consecutive scans: start sending PCL from single scan
const double ALERT_DISTANCE = 1.0;          // Lowest allowable range measurement before sending alert

// Segmentation constants
const int    MIN_NUM_OF_PTS = 5;            // Miminal number of points to make a group
const double GROUPING_D_MAX = 0.080;        // Maximal distance between two points in the group
const double GROUPING_K = 0.006;

const float SEGMENTATION_D_MAX = 0.065;     // Maximal distance between a point and a line in the segment
const float SEGMENTATION_K = 0.006;

const float ENCHANCEMENT_D_MAX = 0.030;     // Maximal distance between a point and a line to merge segments

const double SCANNERS_SEPARATION = 0.45;    // [m] Separation between centers of two laser scanners


// Inlines
inline int signum(double f) { if (f<0) return -1; else return 1; }
inline double absolute(double f) { if (f<0) return -f; else return f; }

}

#endif // DEFINES_H
