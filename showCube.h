/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/


#ifndef _SHOWCUBE_H_
#define _SHOWCUBE_H_

void showCube(struct world * jello);

void showBoundingBox();

void showInclinedPlane(struct world *jello);
bool rayPlaneIntersect(vec ro, vec rd, double a, double b, double c, double d, vec *hit);

#endif
