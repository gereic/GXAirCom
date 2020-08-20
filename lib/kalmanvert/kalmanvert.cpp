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

#include <kalmanvert.h>

#include <Arduino.h>

void kalmanvert::init(double startp, double starta, double sigmap, double sigmaa, unsigned long timestamp) {

  /* init base values */
  p = startp;
  v = 0;
  a = starta;
  t = timestamp;
  calibrationDrift = 0.0;
    
  /* init variance */
  varp = sigmap * sigmap;
  vara = sigmaa * sigmaa;

  /* init covariance matrix */
  p11 = 0;
  p12 = 0;
  p21 = 0;
  p22 = 0;
}

void kalmanvert::update(double mp, double ma, unsigned long timestamp) {

  /**************/
  /* delta time */
  /**************/
  unsigned long deltaTime = timestamp - t;
  double dt = ((double)deltaTime)/1000.0;
  t = timestamp;

  /**************/
  /* prediction */
  /**************/

  /* values */
  a = ma;  // we use the last acceleration value for prediction 
  double dtPower = dt * dt; //dt^2
  p += dt*v + dtPower*a/2;
  v += dt*a;
  //a = ma; // uncomment to use the previous acceleration value 

  /* covariance */
  double inc;
  
  dtPower *= dt;  // now dt^3
  inc = dt*p22+dtPower*vara/2;
  dtPower *= dt; // now dt^4
  p11 += dt*(p12 + p21 + inc) - (dtPower*vara/4);
  p21 += inc;
  p12 += inc;
  p22 += dt*dt*vara;

  /********************/
  /* gaussian product */
  /********************/

  /* kalman gain */
  double s, k11, k12, y;

  s = p11 + varp;
  k11 = p11/s;
  k12 = p12/s;
  y = mp - p;

  /* update */
  p += k11 * y;
  v += k12 * y;
  p22 -= k12 * p21;
  p12 -= k12 * p11;
  p21 -= k11 * p21;
  p11 -= k11 * p11;
 
}

double kalmanvert::getPosition() {

  return p;
}

double kalmanvert::getCalibratedPosition() {

  return (p + calibrationDrift);
}

double kalmanvert::getVelocity() {

  return v;
}

double kalmanvert::getAcceleration() {

  return a;
}

unsigned long kalmanvert::getTimestamp() {

  return t;
}

void kalmanvert::calibratePosition(double newPosition) {

  calibrationDrift = newPosition - p;
}
