#include "computeGPSDistance.h"
#include <math.h>
double computeDistance(double Northing, double lNorthing, double Easting, double lEasting){

  double distance = 0.0; 
  distance = sqrt(pow(lNorthing-Northing,2) + pow(lEasting-Easting,2));
  return distance;
}