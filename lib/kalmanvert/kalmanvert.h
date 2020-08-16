/* kalmanvert -- Compute vertical acceleration with Kalman filter
 *
 * Copyright 2016-2019 Baptiste PELLEGRIN
 * 
 * This file is part of GNUVario.
 *
 * GNUVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNUVario is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef KALMANVERT_H
#define KALMANVERT_H

#include <Arduino.h>

/*********************************************************/
/* compute velocity from known position and acceleration */
/* p = position, v = velocity, a = acceleration          */
/*********************************************************/

class kalmanvert {

 public:
  /**********************************************************/
  /*           init with your first measured values         */
  /*  !!! sigmap and sigmaa are very important values !!!   */
  /* make experimentations : ex sigmap = 0.1 , sigmaa = 0.3 */
  /**********************************************************/
  void init(double startp, double starta, double sigmap, double sigmaa, unsigned long timestamp);

  /* run each time you get new values */
  void update(double mp, double ma, unsigned long timestamp);

  /* at any time get result */
  double getPosition();
  double getCalibratedPosition();
  double getVelocity();
  double getAcceleration();
  unsigned long getTimestamp();

  /* reset the current position without changing velocity and acceleration */
  void calibratePosition(double newPosition);

 private:
  /* position variance, acceleration variance */
  double varp, vara;
  
  /* position, velocity, acceleration, timestamp */
  double p, v, a;
  unsigned long t;

  /* calibration */
  double calibrationDrift;

  /* covariance matrix */
  double p11, p21, p12, p22;
  
};

#endif
