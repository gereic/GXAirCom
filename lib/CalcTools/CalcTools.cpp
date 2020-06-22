#include "CalcTools.h"


/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double deg) {
  return (deg * pi / 180);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double rad2deg(double rad) {
  return (rad * 180 / pi);
}


double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
  
  double theta, dist;
  theta = lon1 - lon2;
  dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515;
  switch(unit) {
    case 'M':
      break;
    case 'K':
      dist = dist * 1.609344;
      break;
    case 'N':
      dist = dist * 0.8684;
      break;
  }
  return (dist);
}

double dtorA(double fdegrees)
{
  return(fdegrees * PI / 180);
}

//Convert radians to degrees
double rtodA(double fradians)
{
  return(fradians * 180.0 / PI);
}


int CalcBearingA(double lat1, double lon1, double lat2, double lon2)
{

  
 
  lat1 = dtorA(lat1);
  lon1 = dtorA(lon1);
  lat2 = dtorA(lat2);
  lon2 = dtorA(lon2);
  
  //determine angle
  double bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)));
  //convert to degrees
  bearing = rtodA(bearing);
  //use mod to turn -90 = 270
  //bearing = fmod((bearing + 360.0), 360);
  //return (int) bearing + 0.5;
  return ((int) bearing + 360) % 360;
}

