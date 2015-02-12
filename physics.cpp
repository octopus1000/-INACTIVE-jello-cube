/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"
#include <vector>

void forceSpring(vec *L, vec *LV, vec *f, double R, double k, double kd);
void collideBox(struct point *p, vec v, double k, double kd, vec *force);
void forceField(struct world *jello, vec *f, struct point p);
/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
	vec p, v, *pa = NULL;//p = jello->p[i][j][k], v = jello->v[i][j][k], pa = &(a[i][j][k])
	vec L = { 0, 0, 0 }, LV = { 0, 0, 0 }, f = { 0, 0, 0 };//L = posA - posB, LV = velocityA - velocityB
	double R = 0;//rest length
	vec fStruct, fShear, fBend;
	
	for (int i = 0; i < 8; i++)
		for (int j = 0; j < 8; j++)
			for (int k = 0; k < 8; k++)
			{
				pa = &(a[i][j][k]);
				p = jello->p[i][j][k], v = jello->v[i][j][k];
				
				memset(pa, 0, sizeof(vec));//clear memory for accumulating
				memset(&fStruct, 0, sizeof(vec));
				memset(&fShear, 0, sizeof(vec));
				memset(&fBend, 0, sizeof(vec));

				/*compute structural and shear spring force*/
				for (int l = -1; l < 2; l++){
					for (int m = -1; m < 2; m++){
						for (int n = -1; n < 2; n++)
						{
							R = sqrt(abs(l) + abs(m) + abs(n));//length depends
							if (i + l < 0 || i + l >7 || j + m < 0 || j + m > 7 || k + n < 0 || k + n > 7|| R == 0)
								continue;
							pDIFFERENCE(jello->v[i][j][k], jello->v[i + l][j + m][k + n], LV);
							pDIFFERENCE(jello->p[i][j][k], jello->p[i + l][j + m][k + n], L);
							forceSpring(&L, &LV, &f, 1.0 / 7 * R, jello->kElastic, jello->dElastic);
							if (R > 1.001){//shear spring force 0.001 error tolerance
								pSUM(fShear, f, fShear);
							}
							else{//structural spring force
								pSUM(fStruct, f, fStruct);
							}								
						}
					}
				}
				
				/*compute bend spring force*/
				R = 2;//length equals 2 units
#define COMPUTEBEND(di,dj,dk) \
	if(!(i + di < 0 || i + di >7 || j + dj < 0 || j + dj > 7 || k + dk < 0 || k + dk > 7)){\
	pDIFFERENCE(jello->v[i][j][k], jello->v[i+di][j+dj][k+dk], LV);\
	pDIFFERENCE(jello->p[i][j][k], jello->p[i+di][j+dj][k+dk], L);\
	forceSpring(&L, &LV, &f, 1.0 / 7 * R, jello->kElastic, jello->dElastic);\
	pSUM(fBend, f, fBend);\
	}
				COMPUTEBEND(0, 2, 0);
				COMPUTEBEND(0, 0, 2);
				COMPUTEBEND(0, 0, -2);
				COMPUTEBEND(2, 0, 0);
				COMPUTEBEND(-2, 0, 0);
				COMPUTEBEND(0, -2, 0);				

				//add structural, shear and bend force to accumulated force
				pSUM(*pa, fStruct, *pa);
				pSUM(*pa, fShear, *pa);
				pSUM(*pa, fBend, *pa);
				
				/*compute collision penalty spring force*/
				collideBox(&p, v, jello->kCollision, jello->dCollision, &f);
				pSUM(*pa, f, *pa);
				if (jello->incPlanePresent && collidePlane(jello, p, v, &f)){
					pSUM(*pa, f, *pa);
				}
				
				/*compute forcefield force*/
				forceField(jello, &f, p);
				pSUM(*pa, f, *pa);
	
				/*compute mouse drag force*/
				vec dpos;//user point and current point position delta 
				pMAKE(i, j, k, dpos);
				pDIFFERENCE(dpos,z_hitPos,dpos);
				if (z_hit && pLENGTH(dpos) <= 3 ){
					pMULTIPLY(z_userForce, 1 - pLENGTH(dpos)/4, f);
					//pMULTIPLY(f, 9.8, f);
					pSUM(*pa, f, *pa);
					if (pLENGTH(z_userForce) < 1){
						pMAKE(0., 0., 0., z_userForce);
					}
						
				}
				pMULTIPLY(*pa, 1.0 / jello->mass, *pa);
			}
}

void forceSpring(vec *L, vec *LV, vec *f, double R, double k, double kd){
	double len, temp;
	vec fh = { 0, 0, 0 }, fd = { 0, 0, 0 };
	len = pLENGTH(*L);
	memset(f, 0, sizeof(vec));
	if (len == 0) return;
	//Hook
	pMULTIPLY(*L, -k * (len - R) / len, fh);
	//damping
	temp = pDOTPRODUCT(*LV, *L);
	pMULTIPLY(*L, -kd * temp / len / len, fd);
	pSUM(fh, fd, *f);
}
void forceField(struct world *jello, vec *f,struct point p){
	const double boxd = 4;
	struct point pd,value[8];
	struct point i1, i2, j1, j2, w1, w2;//interpolate coordinate
	memset(f, 0, sizeof(vec));

#define RESETBOUNDE(p)\
	if(p >= 1)\
	p = 0.99;\
	if(p < 0)\
	p = 0;
	
	//scale to (0,resolution -1)
	pMULTIPLY(p, 1. / boxd, p);//scale to (-0.5 ,0.5)
	p.x += .5; 
	p.y += .5; 
	p.z += .5;//adjust to (0, 1);

	RESETBOUNDE(p.x);
	RESETBOUNDE(p.y);
	RESETBOUNDE(p.z);

	pMULTIPLY(p, jello->resolution - 1, p);//adjust to (0, resolution - 1)
	
	pd.x = floor(p.x); 
	pd.y = floor(p.y); 
	pd.z = floor(p.z);//closest lattice point bellow p
	
	//the ratio to left bottom point
	pDIFFERENCE(p, pd, p);
	double resolution = jello->resolution;

#define GETFORCEFIELD(value, x,y,z)\
	value = jello->forceField[int(x*resolution*resolution + y*resolution + z)];
	//get adjecent points in forcefield
	GETFORCEFIELD(value[0], pd.x, pd.y, pd.z);
	GETFORCEFIELD(value[1], pd.x, pd.y, pd.z + 1);
	GETFORCEFIELD(value[2], pd.x, pd.y + 1, pd.z);
	GETFORCEFIELD(value[3], pd.x, pd.y + 1, pd.z + 1);
	GETFORCEFIELD(value[4], pd.x + 1, pd.y, pd.z);
	GETFORCEFIELD(value[5], pd.x + 1, pd.y, pd.z + 1);
	GETFORCEFIELD(value[6], pd.x + 1, pd.y + 1, pd.z);
	GETFORCEFIELD(value[7], pd.x + 1, pd.y + 1, pd.z + 1);
	
	struct point temp1;
	
#define INTERPOLATE3D(value1,value2, d,dest) \
	pMULTIPLY(value1, 1 - d, dest);\
	pMULTIPLY(value2, d, temp1);\
	pSUM(dest, temp1, dest);
	
	//interpolate at z direction
	INTERPOLATE3D(value[0], value[1], p.z, i1);
	INTERPOLATE3D(value[2], value[3], p.z, i2);
	INTERPOLATE3D(value[4], value[5], p.z, j1);
	INTERPOLATE3D(value[6], value[7], p.z, j2);
	//y direction
	INTERPOLATE3D(i1, i2, p.y, w1);
	INTERPOLATE3D(j1, j2, p.y, w2);
	//x direction
	INTERPOLATE3D(w1, w2, p.x, *f);
}

void collideBox(struct point *p,vec v,double k,double kd,vec *force){
	vec LV = v, L = { 0, 0, 0 }, f = { 0, 0, 0 };
	memset(force, 0, sizeof(vec));//clear output  force in order that no force to be set

#define COLLIDE(obj, wall, dir, l)\
	memset(&L, 0. , sizeof(vec));\
	l = obj - wall;\
	if((l) * dir < 0){\
	forceSpring(&L, &LV, &f, 0, k, kd);\
	pSUM(*force, f, *force);\
	}

	COLLIDE(p->x, -2, 1, L.x);
	COLLIDE(p->x, 2, -1, L.x);
	COLLIDE(p->y, -2, 1, L.y);
	COLLIDE(p->y, 2, -1, L.y);
	COLLIDE(p->z, -2, 1, L.z);
	COLLIDE(p->z, 2, -1, L.z);
}

/*detect which side the jello is on*/
/*if return value is true then the jello is on the other side*/
bool collidePlane(struct world *jello, vec p, vec v, vec *f){
	bool collide = false;
	double length,distance,a = jello->a,b = jello->b,c = jello->c,d = jello->d;
	vec planeNorm,L, LV = v;
	memset(f, 0, sizeof(vec));
	if (jello->incPlanePresent){
		//assume that result > 0 if on the initial side, result < 0 when collide the plane
		distance = (a*p.x + b*p.y + c*p.z + d) / sqrt(a*a + b*b + c*c);
		collide = (distance <= 0);
		if (collide){
			pMAKE(a, b, c, planeNorm);
			pNORMALIZE(planeNorm);
			pMULTIPLY(planeNorm, distance, L);
			forceSpring(&L, &LV, f, 0, jello->kCollision, jello->dCollision);
		}
	}
	return collide;
}
/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
