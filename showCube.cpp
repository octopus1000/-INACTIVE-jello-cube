/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "showCube.h"

int pointMap(int side, int i, int j)
{
  int r;

  switch (side)
  {
  case 1: //[i][j][0] bottom face
    r = 64 * i + 8 * j;
    break;
  case 6: //[i][j][7] top face
    r = 64 * i + 8 * j + 7;
    break;
  case 2: //[i][0][j] front face
    r = 64 * i + j;
    break;
  case 5: //[i][7][j] back face
    r = 64 * i + 56 + j;
    break;
  case 3: //[0][i][j] left face
    r = 8 * i + j;
    break;
  case 4: //[7][i][j] right face
    r = 448 + 8 * i + j;
    break;
  }

  return r;
}

void showCube(struct world * jello)
{
  int i,j,k,ip,jp,kp;
  point r1,r2,r3; // aux variables
  
  /* normals buffer and counter for Gourad shading*/
  struct point normal[8][8];
  int counter[8][8];

  int face;
  double faceFactor, length;

  if (fabs(jello->p[0][0][0].x) > 10)
  {
    printf ("Your cube somehow escaped way out of the box.\n");
    exit(0);
  }

  
  #define NODE(face,i,j) (*((struct point * )(jello->p) + pointMap((face),(i),(j))))

  
  #define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if\
    (!( (ip>7) || (ip<0) ||\
      (jp>7) || (jp<0) ||\
    (kp>7) || (kp<0) ) && ((i==0) || (i==7) || (j==0) || (j==7) || (k==0) || (k==7))\
       && ((ip==0) || (ip==7) || (jp==0) || (jp==7) || (kp==0) || (kp==7))) \
    {\
      glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
      glVertex3f(jello->p[ip][jp][kp].x,jello->p[ip][jp][kp].y,jello->p[ip][jp][kp].z);\
    }\

  if (mode == GL_SELECT){
#define ISSURFACE(x,y,z)\
(x >= 0 && y >= 0 && z >= 0 && x <= 7 && y <= 7 && z <= 7 && (x == 0 || y == 0 || z == 0 || x == 7 || y == 7 || z == 7))
#define DRAWQUAD(p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z)\
if(ISSURFACE(p1x,p1y,p1z) && ISSURFACE(p2x,p2y,p2z) && ISSURFACE(p3x,p3y,p3z)){\
glBegin(GL_TRIANGLE_STRIP);\
glColor3f(0.1, 0.2, 0.3); \
glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
	  glVertex3f(jello->p[p1x][p1y][p1z].x,jello->p[p1x][p1y][p1z].y,jello->p[p1x][p1y][p1z].z);\
	  glVertex3f(jello->p[p2x][p2y][p2z].x,jello->p[p2x][p2y][p2z].y,jello->p[p2x][p2y][p2z].z);\
	  glVertex3f(jello->p[p3x][p3y][p3z].x,jello->p[p3x][p3y][p3z].y,jello->p[p3x][p3y][p3z].z);\
glEnd();\
}
	  glDisable(GL_CULL_FACE);
	  glDisable(GL_LIGHTING);
	  for (i = 0; i <= 7; i++){
		  for (j = 0; j <= 7; j++){
			  for (k = 0; k <= 7; k++){
				  if (i*j*k*(7 - i)*(7 - j)*(7 - k) != 0) // not surface point
					  continue;
				  glLoadName(i * 8 * 8 + j * 8 + k);
				  DRAWQUAD(i, j, k + 1, i, j + 1, k, i, j + 1, k + 1);
				  DRAWQUAD(i, j, k + 1, i + 1, j, k, i + 1, j, k + 1);
				  DRAWQUAD(i, j + 1, k, i + 1, j, k, i + 1, j + 1, k);
			  }
		  }
	  }
	  glEnable(GL_LIGHTING);
	  glEnable(GL_CULL_FACE);

  }
  
  else if (viewingMode==0) // render wireframe
  {
    glLineWidth(1);
    glPointSize(5);
    glDisable(GL_LIGHTING);
    for (i=0; i<=7; i++)
      for (j=0; j<=7; j++)
        for (k=0; k<=7; k++)
        {
          if (i*j*k*(7-i)*(7-j)*(7-k) != 0) // not surface point
            continue;
		  if (mode == GL_SELECT)
			  glLoadName(i * 8 * 8 + j * 8 + k);
          glBegin(GL_POINTS); // draw point
            glColor4f(0,0,0,0);  
            glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);        
          glEnd();


		  //if (mode == GL_SELECT)
			 // continue;//select mode do not render springs
          //
          //if ((i!=7) || (j!=7) || (k!=7))
          //  continue;

          glBegin(GL_LINES);      
          // structural
          if (structural == 1)
          {
            glColor4f(0,0,1,1);
            PROCESS_NEIGHBOUR(1,0,0);
            PROCESS_NEIGHBOUR(0,1,0);
            PROCESS_NEIGHBOUR(0,0,1);
            PROCESS_NEIGHBOUR(-1,0,0);
            PROCESS_NEIGHBOUR(0,-1,0);
            PROCESS_NEIGHBOUR(0,0,-1);
          }
          
          // shear
          if (shear == 1)
          {
            glColor4f(0,1,0,1);
            PROCESS_NEIGHBOUR(1,1,0);
            PROCESS_NEIGHBOUR(-1,1,0);
            PROCESS_NEIGHBOUR(-1,-1,0);
            PROCESS_NEIGHBOUR(1,-1,0);
            PROCESS_NEIGHBOUR(0,1,1);
            PROCESS_NEIGHBOUR(0,-1,1);
            PROCESS_NEIGHBOUR(0,-1,-1);
            PROCESS_NEIGHBOUR(0,1,-1);
            PROCESS_NEIGHBOUR(1,0,1);
            PROCESS_NEIGHBOUR(-1,0,1);
            PROCESS_NEIGHBOUR(-1,0,-1);
            PROCESS_NEIGHBOUR(1,0,-1);

            PROCESS_NEIGHBOUR(1,1,1)
            PROCESS_NEIGHBOUR(-1,1,1)
            PROCESS_NEIGHBOUR(-1,-1,1)
            PROCESS_NEIGHBOUR(1,-1,1)
            PROCESS_NEIGHBOUR(1,1,-1)
            PROCESS_NEIGHBOUR(-1,1,-1)
            PROCESS_NEIGHBOUR(-1,-1,-1)
            PROCESS_NEIGHBOUR(1,-1,-1)
          }
          
          // bend
          if (bend == 1)
          {
            glColor4f(1,0,0,1);
            PROCESS_NEIGHBOUR(2,0,0);
            PROCESS_NEIGHBOUR(0,2,0);
            PROCESS_NEIGHBOUR(0,0,2);
            PROCESS_NEIGHBOUR(-2,0,0);
            PROCESS_NEIGHBOUR(0,-2,0);
            PROCESS_NEIGHBOUR(0,0,-2);
          }           
          glEnd();
        }
    glEnable(GL_LIGHTING);
  }
  
  else
  {
    glPolygonMode(GL_FRONT, GL_FILL); 
    
    for (face=1; face <= 6; face++) 
      // face == face of a cube
      // 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
    {
      
      if ((face==1) || (face==3) || (face==5))
        faceFactor=-1; // flip orientation
      else
        faceFactor=1;
      

      for (i=0; i <= 7; i++) // reset buffers
        for (j=0; j <= 7; j++)
        {
          normal[i][j].x=0;normal[i][j].y=0;normal[i][j].z=0;
          counter[i][j]=0;
        }

      /* process triangles, accumulate normals for Gourad shading */
  
      for (i=0; i <= 6; i++)
        for (j=0; j <= 6; j++) // process block (i,j)
        {
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i,j),r1); // first triangle
          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i,j),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i][j],r3,normal[i][j]);
          counter[i][j]++;

          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i+1,j+1),r1); // second triangle
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i+1,j+1),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i+1][j+1],r3,normal[i+1][j+1]);
          counter[i+1][j+1]++;
        }

      
        /* the actual rendering */
        for (j=1; j<=7; j++) 
        {

          if (faceFactor  > 0)
            glFrontFace(GL_CCW); // the usual definition of front face
          else
            glFrontFace(GL_CW); // flip definition of orientation
         
          glBegin(GL_TRIANGLE_STRIP);
          for (i=0; i<=7; i++)
          {
            glNormal3f(normal[i][j].x / counter[i][j],normal[i][j].y / counter[i][j],
              normal[i][j].z / counter[i][j]);
            glVertex3f(NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z);
            glNormal3f(normal[i][j-1].x / counter[i][j-1],normal[i][j-1].y/ counter[i][j-1],
              normal[i][j-1].z / counter[i][j-1]);
            glVertex3f(NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z);
          }
          glEnd();
        }
        
        
    }  
  } // end for loop over faces
  glFrontFace(GL_CCW);
}

/*these two vector is for sort vertices*/
vec z_planeNorm, z_origin;
int vertex3Dcmp(const void *a, const void *b){
	vec v, v1,v2;
	pDIFFERENCE(*(vec*)a, z_origin, v1);
	pDIFFERENCE(*(vec*)b, z_origin, v2);
	CROSSPRODUCTp(v1, v2, v);
	return pDOTPRODUCT(v, z_planeNorm) < 0;
}

bool rayPlaneIntersect(vec ro, vec rd, double a, double b, double c, double d,vec *hit){
	double t;
	if ((a*rd.x + b*rd.y + c*rd.z) == 0)
		return false;
	t = -(a*ro.x + b*ro.y + c*ro.z + d) / (a*rd.x + b*rd.y + c*rd.z);
	pMAKE(ro.x + rd.x* t, ro.y + rd.y*t, ro.z + rd.z *t, *hit);
	return true;
}
/*idea of render plane in AABB box refer to http://www.asawicki.info/news_1428_finding_polygon_of_plane-aabb_intersection.html*/
void showInclinedPlane(struct world *jello){
	vec vertices[12], ro, rd, hit;//at most 6 points intersect with plane
	int c = 0;//counter for vertices

	if (!jello->incPlanePresent)
		return;

#define INSIDEBOX(axis)\
	(axis >= -2 && axis <= 2)

#define RAYPLANEINTERSECT(x1,y1,z1,x2,y2,z2)\
	pMAKE(x1,y1,z1,ro);\
	pMAKE(x2,y2,z2,rd);\
	pDIFFERENCE(rd,ro,rd);\
	if(rayPlaneIntersect(ro,rd,jello->a,jello->b,jello->c,jello->d,&hit) && INSIDEBOX(hit.x)&&INSIDEBOX(hit.y)&&INSIDEBOX(hit.z)){\
	pCPY(hit,vertices[c]);\
	c++;\
	}
	
	/*x axis*/
	RAYPLANEINTERSECT(-2, -2, -2, 2, -2, -2);
	RAYPLANEINTERSECT(-2, 2, -2, 2, 2, -2);
	RAYPLANEINTERSECT(-2, -2, 2, 2, -2, 2);
	RAYPLANEINTERSECT(-2, 2, 2, 2, 2, 2);
	/*y axis*/
	RAYPLANEINTERSECT(-2,-2,-2,-2,2,-2);
	RAYPLANEINTERSECT(2, -2, -2, 2, 2, -2);
	RAYPLANEINTERSECT(-2, -2, 2, -2, 2, 2);
	RAYPLANEINTERSECT(2, -2, 2, 2, 2, 2);
	/*z axis*/
	RAYPLANEINTERSECT(-2, -2, -2, -2, -2, 2);
	RAYPLANEINTERSECT(2, -2, -2, 2, -2, 2);
	RAYPLANEINTERSECT(-2, 2, -2, -2, 2, 2);
	RAYPLANEINTERSECT(2, 2, -2, 2, 2, 2);

	pMAKE(jello->a, jello->b, jello->c, z_planeNorm);
	pCPY(vertices[0], z_origin);

	qsort(vertices, c, sizeof(vec), vertex3Dcmp);

	glColor4f(0.6, 0.6, 0.6, 0);
	
	glDisable(GL_CULL_FACE);
	glBegin(GL_POLYGON);
	for (int i = 0; i < c; i++){
		glVertex3f(vertices[i].x, vertices[i].y, vertices[i].z);
	}
	glEnd();
	glEnable(GL_CULL_FACE);

}

void showBoundingBox()
{
  int i,j;

  glColor4f(0.6,0.6,0.6,0);

  glBegin(GL_LINES);

  // front face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,-2,-2);
    glVertex3f(i,-2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(2,-2,j);
  }

  // back face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,2,-2);
    glVertex3f(i,2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,2,j);
    glVertex3f(2,2,j);
  }

  // left face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(-2,i,-2);
    glVertex3f(-2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(-2,2,j);
  }

  // right face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(2,i,-2);
    glVertex3f(2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(2,-2,j);
    glVertex3f(2,2,j);
  }
  
  glEnd();

  return;
}

