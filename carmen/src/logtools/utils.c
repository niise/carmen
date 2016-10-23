#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/times.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <zlib.h>
#include <stdarg.h>

#include <carmen/logtools.h>
#include "defines.h"

double
fsgn( double val )
{
  if (val>0.0) {
    return(1);
  } else if (val<0.0) {
    return(-1);
  } else {
    return(0);
  }
}

int
sgn( int val )
{
  if (val>0) {
    return(1);
  } else if (val<0) {
    return(-1);
  } else {
    return(0);
  }
}

double
rad2Deg(double x)
{
  return x * 57.29577951308232087679;
}

double
deg2Rad(double x)
{
  return x * 0.01745329251994329576;
}


double
normalize_theta(double theta) 
{
  int multiplier;

  if (theta >= -M_PI && theta < M_PI)
    return theta;

  multiplier = theta / (2*M_PI);
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
} 

int
timeCompare ( struct timeval time1, struct timeval time2 )
{
  if (time1.tv_sec<time2.tv_sec) {
    return(-1);
  } else if (time1.tv_sec>time2.tv_sec) {
    return(1);
  } else if (time1.tv_usec<time2.tv_usec) {
    return(-1);
  } else if (time1.tv_usec>time2.tv_usec) {
    return(1);
  } else {
    return(0);
  }
}

double
timeDiff( struct timeval  t1, struct timeval t2)
{
   double diff;
   diff =  (double) (t2.tv_usec - t1.tv_usec) / 1000000.0;
   diff += (double) (t2.tv_sec  - t1.tv_sec);
   return(diff);
}


VECTOR3
translate_and_rotate_vector3( VECTOR3 p, VECTOR3 rot, VECTOR3 trans )
{
  VECTOR3 p2;
  p.x += trans.x;
  p.y += trans.y;
  p.z += trans.z;
  p2.x =
    p.x * (cos(rot.y)*cos(rot.z)) +
    p.y * (sin(rot.x)*sin(rot.y)*cos(rot.z)-cos(rot.x)*sin(rot.z)) +
    p.z * (cos(rot.x)*sin(rot.y)*cos(rot.z)+sin(rot.x)*sin(rot.z));
  p2.y = 
    p.x * (cos(rot.y)*sin(rot.z)) +
    p.y * (sin(rot.x)*sin(rot.y)*sin(rot.z)+cos(rot.x)*cos(rot.z)) +
    p.z * (cos(rot.x)*sin(rot.y)*sin(rot.z)-sin(rot.x)*cos(rot.z));
  p2.z = 
    p.x * (-sin(rot.y)) +
    p.y * (sin(rot.x)*cos(rot.y)) +
    p.z * (cos(rot.x)*cos(rot.y));
  return(p2);
}

VECTOR3
rotate_and_translate_vector3( VECTOR3 p, VECTOR3 rot, VECTOR3 trans )
{
  VECTOR3 p2;
  p2.x = trans.x +
    p.x * (cos(rot.y)*cos(rot.z)) +
    p.y * (sin(rot.x)*sin(rot.y)*cos(rot.z)-cos(rot.x)*sin(rot.z)) +
    p.z * (cos(rot.x)*sin(rot.y)*cos(rot.z)+sin(rot.x)*sin(rot.z));
  p2.y = trans.y +
    p.x * (cos(rot.y)*sin(rot.z)) +
    p.y * (sin(rot.x)*sin(rot.y)*sin(rot.z)+cos(rot.x)*cos(rot.z)) +
    p.z * (cos(rot.x)*sin(rot.y)*sin(rot.z)-sin(rot.x)*cos(rot.z));
  p2.z = trans.z +
    p.x * (-sin(rot.y)) +
    p.y * (sin(rot.x)*cos(rot.y)) +
    p.z * (cos(rot.x)*cos(rot.y));
  return(p2);
}

VECTOR3
rotate_vector3( VECTOR3 p, VECTOR3 rot )
{
  VECTOR3 p2;
  double s1, s2, s3, c1, c2, c3;

  s1 = sin(rot.y); s2 = sin(rot.z); s3 = sin(rot.x);
  c1 = cos(rot.y); c2 = cos(rot.z); c3 = cos(rot.x);

  p2.x =
    p.x * (s1 * s2 * s3 + c1 * c3) +
    p.y * (c2 * s3) +
    p.z * (c1 * s2 * s3 - c3 * s1);
  p2.y =
    p.x * (c3 * s1 * s2 - c1 * s3) +
    p.y * (c2 * c3) +
    p.z * (c1 * c3 * s2 + s1 * s3);
  p2.z =
    p.x * (c1 * s2 * s3 - c1 * s1) +
    p.y * (-s2) +
    p.z * (c1 * c2);
  
    /*
  VECTOR3 p2;
  p2.x = 
    p.x * (cos(rot.y)*cos(rot.z)) +
    p.y * (sin(rot.x)*sin(rot.y)*cos(rot.z)-cos(rot.x)*sin(rot.z)) +
    p.z * (cos(rot.x)*sin(rot.y)*cos(rot.z)+sin(rot.x)*sin(rot.z));
  p2.y = 
    p.x * (cos(rot.y)*sin(rot.z)) +
    p.y * (sin(rot.x)*sin(rot.y)*sin(rot.z)+cos(rot.x)*cos(rot.z)) +
    p.z * (cos(rot.x)*sin(rot.y)*sin(rot.z)-sin(rot.x)*cos(rot.z));
  p2.z = 
    p.x * (-sin(rot.y)) +
    p.y * (sin(rot.x)*cos(rot.y)) +
    p.z * (cos(rot.x)*cos(rot.y));
    */
  return(p2);
}

VECTOR3
rotate3_x( VECTOR3 p, double rot )
{
  double   s = sin(rot), c = cos(rot);
  VECTOR3  r;
  r.x = p.x;
  r.y = p.y *  c  +  p.z *  s;
  r.z = p.y * -s  +  p.z *  c;
  return(r);
}

VECTOR3
rotate3_y( VECTOR3 p, double rot )
{
  double   s = sin(rot), c = cos(rot);
  VECTOR3  r;
  r.x = p.x *  c  +  p.z * -s;
  r.y = p.y;
  r.z = p.x *  s  +  p.z *  c;
  return(r);
}

VECTOR3
rotate3_z( VECTOR3 p, double rot )
{
  double   s = sin(rot), c = cos(rot);
  VECTOR3  r;
  r.x = p.x *  c  +  p.y *  s;
  r.y = p.x * -s  +  p.y *  c;
  r.z = p.z;
  return(r);
}

VECTOR2
rotate_vector2( VECTOR2 p, double rot )
{
  VECTOR2 v;
  v.x = p.x*cos(rot)-p.y*sin(rot);
  v.y = p.x*sin(rot)+p.y*cos(rot);
  return(v);
}

VECTOR2
rotate_and_translate_vector2( VECTOR2 p, double rot, VECTOR2 trans )
{
  VECTOR2 v;
  v.x = p.x*cos(rot)-p.y*sin(rot)+trans.x;
  v.y = p.x*sin(rot)+p.y*cos(rot)+trans.y;
  return(v);
}

double
vector2_distance( VECTOR2 p1, VECTOR2 p2 ) {
  return sqrt( (p1.x-p2.x)*(p1.x-p2.x) +
	       (p1.y-p2.y)*(p1.y-p2.y) );
}
  
double
vector3_distance( VECTOR3 v1, VECTOR3 v2 )
{
  return( sqrt( (v1.x-v2.x)*(v1.x-v2.x) +
		(v1.y-v2.y)*(v1.y-v2.y) +
		(v1.z-v2.z)*(v1.z-v2.z) ) );
}


double
vector3_length( VECTOR3 v1 )
{
  return( sqrt( (v1.x*v1.x) + (v1.y*v1.y) + (v1.z*v1.z) ) );
}

double
vector2_length( VECTOR2 v1 )
{
  return( sqrt( (v1.x*v1.x) + (v1.y*v1.y) ) );
}


VECTOR3
vector3_add( VECTOR3 v1, VECTOR3 v2 )
{
  VECTOR3 v;
  v.x = v1.x + v2.x;
  v.y = v1.y + v2.y;
  v.z = v1.z + v2.z;
  return(v);
}

VECTOR3
vector3_diff( VECTOR3 v1, VECTOR3 v2 )
{
  VECTOR3 v;
  v.x = v1.x - v2.x;
  v.y = v1.y - v2.y;
  v.z = v1.z - v2.z;
  return(v);
}

VECTOR3
vector3_cross( VECTOR3 v1, VECTOR3 v2 )
{
  VECTOR3 v;
  v.x = ( v1.y * v2.z ) - ( v1.z * v2.y );
  v.y = ( v1.z * v2.x ) - ( v1.x * v2.z );
  v.z = ( v1.x * v2.y ) - ( v1.y * v2.x );
  return(v);
}

VECTOR3
vector3_scalar_mult( VECTOR3 v1, double m )
{
  VECTOR3 v;
  v.x = v1.x * m;
  v.y = v1.y * m;
  v.z = v1.z * m;
  return(v);
}

double
vector3_dot_mult( VECTOR3 v1, VECTOR3 v2 )
{
  double val;
  val = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
  return(val);
}


double
random_gauss( void )
{
  static int iset = 0;
  static double gset;
  float fac, rsq, v1, v2;
  if(iset == 0) {
    do {
      v1 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;
      v2 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;         
      rsq = v1*v1 + v2*v2;
    } while(rsq >= 1.0 || rsq == 0.0);
    fac = sqrt(-2.0*log(rsq)/rsq);
    gset = v1*fac;
    iset = 1;
    return v2*fac;
  }
  else {
    iset = 0;
    return gset;
  }
}

void
init_bounding_box( BOUNDING_BOX3 *box )
{
  VECTOR3 min, max;

  min.x = MAXFLOAT;
  min.y = MAXFLOAT;
  min.z = MAXFLOAT;
  max.x = -MAXFLOAT;
  max.y = -MAXFLOAT;
  max.z = -MAXFLOAT;

  box->min.x = min.x;       box->max.x = max.x;
  box->min.y = min.y;       box->max.y = max.y;
  box->min.z = min.z;       box->max.z = max.z;

  box->p[0].x = min.x;      box->p[1].x = min.x;
  box->p[0].y = min.y;      box->p[1].y = max.y;
  box->p[0].z = min.z;      box->p[1].z = min.z;

  box->p[2].x = max.x;      box->p[3].x = max.x;
  box->p[2].y = max.y;      box->p[3].y = min.y;
  box->p[2].z = min.z;      box->p[3].z = min.z;

  box->p[4].x = min.x;      box->p[5].x = min.x;
  box->p[4].y = min.y;      box->p[5].y = max.y;
  box->p[4].z = max.z;      box->p[5].z = max.z;

  box->p[6].x = max.x;      box->p[7].x = max.x;
  box->p[6].y = max.y;      box->p[7].y = min.y;
  box->p[6].z = max.z;      box->p[7].z = max.z;
}

void
update_bounding_box( VECTOR3 vec, BOUNDING_BOX3 *box )
{
  VECTOR3 min, max;

  min.x = box->min.x;
  min.y = box->min.y;
  min.z = box->min.z;
  max.x = box->max.x;
  max.y = box->max.y;
  max.z = box->max.z;
  
  if (vec.x<min.x) min.x=vec.x;
  if (vec.y<min.y) min.y=vec.y;
  if (vec.z<min.z) min.z=vec.z;
  if (vec.x>max.x) max.x=vec.x;
  if (vec.y>max.y) max.y=vec.y;
  if (vec.z>max.z) max.z=vec.z;

  box->min.x = min.x;       box->max.x = max.x;
  box->min.y = min.y;       box->max.y = max.y;
  box->min.z = min.z;       box->max.z = max.z;

  box->p[0].x = min.x;      box->p[1].x = min.x;
  box->p[0].y = min.y;      box->p[1].y = max.y;
  box->p[0].z = min.z;      box->p[1].z = min.z;

  box->p[2].x = max.x;      box->p[3].x = max.x;
  box->p[2].y = max.y;      box->p[3].y = min.y;
  box->p[2].z = min.z;      box->p[3].z = min.z;

  box->p[4].x = min.x;      box->p[5].x = min.x;
  box->p[4].y = min.y;      box->p[5].y = max.y;
  box->p[4].z = max.z;      box->p[5].z = max.z;

  box->p[6].x = max.x;      box->p[7].x = max.x;
  box->p[6].y = max.y;      box->p[7].y = min.y;
  box->p[6].z = max.z;      box->p[7].z = max.z;

}

void
compute_bounding_box( VECTOR3_SET vset, BOUNDING_BOX3 *box )
{
  int i;
  VECTOR3 min, max;

  min.x = MAXFLOAT;
  min.y = MAXFLOAT;
  min.z = MAXFLOAT;
  max.x = -MAXFLOAT;
  max.y = -MAXFLOAT;
  max.z = -MAXFLOAT;
  
  for (i=0;i<vset.numvectors;i++) {
    if (vset.vec[i].x<min.x)
      min.x = vset.vec[i].x;
    if (vset.vec[i].y<min.y)
      min.y = vset.vec[i].y;
    if (vset.vec[i].z<min.z)
      min.z = vset.vec[i].z;
    if (vset.vec[i].x>max.x)
      max.x = vset.vec[i].x;
    if (vset.vec[i].y>max.y)
      max.y = vset.vec[i].y;
    if (vset.vec[i].z>max.z)
      max.z = vset.vec[i].z;
  }

  box->min.x = min.x;       box->max.x = max.x;
  box->min.y = min.y;       box->max.y = max.y;
  box->min.z = min.z;       box->max.z = max.z;

  box->p[0].x = min.x;      box->p[1].x = min.x;
  box->p[0].y = min.y;      box->p[1].y = max.y;
  box->p[0].z = min.z;      box->p[1].z = min.z;

  box->p[2].x = max.x;      box->p[3].x = max.x;
  box->p[2].y = max.y;      box->p[3].y = min.y;
  box->p[2].z = min.z;      box->p[3].z = min.z;

  box->p[4].x = min.x;      box->p[5].x = min.x;
  box->p[4].y = min.y;      box->p[5].y = max.y;
  box->p[4].z = max.z;      box->p[5].z = max.z;

  box->p[6].x = max.x;      box->p[7].x = max.x;
  box->p[6].y = max.y;      box->p[7].y = min.y;
  box->p[6].z = max.z;      box->p[7].z = max.z;

}

void
compute_bounding_box_with_min_max( VECTOR3 min, VECTOR3 max,
				   BOUNDING_BOX3 *box )
{
  box->p[0].x = min.x;      box->p[1].x = min.x;
  box->p[0].y = min.y;      box->p[1].y = max.y;
  box->p[0].z = min.z;      box->p[1].z = min.z;

  box->p[2].x = max.x;      box->p[3].x = max.x;
  box->p[2].y = max.y;      box->p[3].y = min.y;
  box->p[2].z = min.z;      box->p[3].z = min.z;

  box->p[4].x = min.x;      box->p[5].x = min.x;
  box->p[4].y = min.y;      box->p[5].y = max.y;
  box->p[4].z = max.z;      box->p[5].z = max.z;

  box->p[6].x = max.x;      box->p[7].x = max.x;
  box->p[6].y = max.y;      box->p[7].y = min.y;
  box->p[6].z = max.z;      box->p[7].z = max.z;

}

void
compute_bounding_box_plus_dist( VECTOR3_SET vset,
				BOUNDING_BOX3 *box,
				double dist)
{
  compute_bounding_box( vset, box );

  box->p[0].x -= dist;      box->p[1].x -= dist;
  box->p[0].y -= dist;      box->p[1].y += dist;
  box->p[0].z -= dist;      box->p[1].z -= dist;

  box->p[2].x += dist;      box->p[3].x += dist;
  box->p[2].y += dist;      box->p[3].y -= dist;
  box->p[2].z -= dist;      box->p[3].z -= dist;

  box->p[4].x -= dist;      box->p[5].x -= dist;
  box->p[4].y -= dist;      box->p[5].y += dist;
  box->p[4].z += dist;      box->p[5].z += dist;

  box->p[6].x += dist;      box->p[7].x += dist;
  box->p[6].y += dist;      box->p[7].y -= dist;
  box->p[6].z += dist;      box->p[7].z += dist;
  
}

double
compute_triangle3_area( TRIANGLE3 tri )
{
  double a = vector3_distance( tri.point0, tri.point1 );
  double b = vector3_distance( tri.point1, tri.point2 );
  double c = vector3_distance( tri.point2, tri.point0 );
  double s = (a+b+c/2.0);
  return( sqrt( s*(s-a)*(s-b)*(s-c) ) );
}

LINE2_LSQFT
compute_lsqf_line( VECTOR2_SET p )
{
  int i;
  LINE2_LSQFT line;
  double oxm, oym, Sx, Sy, Sxy, cphi, sphi;
  
  line.numpoints  = p.numvectors;
  line.xm         = 0;
  line.ym         = 0;
  line.error      = 0;
  line.phi        = 0;
  line.ndist      = 0;
  
  if (line.numpoints<2)
    return(line);
  
  for (i=0; i<p.numvectors; i++) {
    line.xm += p.vec[i].x; 
    line.ym += p.vec[i].y; 
  }
  
  oxm = line.xm / (double) p.numvectors;
  oym = line.ym / (double) p.numvectors;

  Sx = 0; Sy = 0; Sxy = 0;
  for (i=0; i<p.numvectors; i++) {
    Sx  += (p.vec[i].x-oxm) * (p.vec[i].x-oxm);
    Sy  += (p.vec[i].y-oym) * (p.vec[i].y-oym);
    Sxy += (p.vec[i].x-oxm) * (p.vec[i].y-oym);
  }
  line.phi = 0.5 * atan2( (-2*Sxy), (Sy-Sx) );
  cphi = cos(line.phi);
  sphi = sin(line.phi);
  line.ndist = oxm * cphi + oym * sphi;
  for (i=0; i<p.numvectors; i++) {
    line.error += 
      ( ( p.vec[i].x * cphi + p.vec[i].y * sphi - line.ndist ) *
	( p.vec[i].x * cphi + p.vec[i].y * sphi - line.ndist ) );
  }
  return(line);
}  

double
gauss_function( double x, double mu, double sigma )
{
 return( (1/sqrt(2.0*M_PI*sigma*sigma)) *
	 exp(-(((x-mu)*(x-mu))/(2*sigma*sigma))) );  
}

GAUSS_KERNEL
compute_gauss_kernel_with_function( int length )
{
  GAUSS_KERNEL kernel;
  
  int center = (length-1) / 2;
  double sigma = sqrt( length/(2*M_PI) );
  int i;

  kernel.len = length;
  kernel.val = (double *) malloc( length * sizeof(double) );
  for (i=0;i<length;i++) {
    kernel.val[i] = gauss_function( (i-center), 0.0, sigma );
  }
  return(kernel);
}

GAUSS_KERNEL
compute_gauss_kernel(int length )
{
  GAUSS_KERNEL  kernel;
  int i, j, *store, sum = 0;
  store = (int *) malloc( length*sizeof(int) );
  store[0] = 1;
  for ( i=0; i<length-1; i++ ) {
    store[i+1] = 1;
    for ( j=i; j>0; j-- ) {
      store[j] = store[j] + store[j-1];
    }
  }
  for ( i=0; i<length; i++ ) {
    sum += store[i];
  }
  kernel.len = length;
  kernel.val = (double *) malloc( length * sizeof(double) );
  for (i=0;i<length;i++) {
    kernel.val[i] = store[i] / (double) sum;
  }
  free( store );
  return(kernel);
}


double
convert_orientation_to_range( double angle ) {
  if (angle<-M_PI) {
    return(2*M_PI -  fmod( fabs(angle),2*M_PI ));
  } else if (angle>M_PI) {
    return(-2*M_PI + fmod( angle,2*M_PI ));
  } else {
    return(angle);
  }
}

double
compute_orientation_diff( double start, double end ) {
  double diff;
  diff =
    convert_orientation_to_range( start ) -
    convert_orientation_to_range( end );
  if (diff<-M_PI) {
    return(2*M_PI+diff);
  } else if (diff>M_PI) {
    return(-2*M_PI+diff);
  } else {
    return(diff);
  }
}

RMOVE2
compute_movement2_between_rpos2( RPOS2 start, RPOS2 end )
{
  RMOVE2 move;

  /* compute forward and sideward sensing_MOVEMENT */
  move.forward =
    + (end.y - start.y) * sin(start.o)
    + (end.x - start.x) * cos(start.o);
  
  move.sideward =
    - (end.y - start.y) * cos(start.o)
    + (end.x - start.x) * sin(start.o);
  
  move.rotation = compute_orientation_diff( end.o, start.o );

  return( move );
}

RPOS2
compute_rpos2_with_movement2( RPOS2 start, RMOVE2 move ) {
  RPOS2 end;
  if ( (move.forward==0.0) && (move.sideward==0.0) && (move.rotation==0.0) )
    return (start);
  end.x = start.x + cos(start.o) * move.forward + sin(start.o) * move.sideward;
  end.y = start.y + sin(start.o) * move.forward - cos(start.o) * move.sideward;
  end.o = convert_orientation_to_range( start.o + move.rotation );
  return(end);
}

RPOS2
compute_rpos2_backwards_with_movement2( RPOS2 start, RMOVE2 move ) {
  RPOS2 end;
  if ( (move.forward==0.0) && (move.sideward==0.0) && (move.rotation==0.0) )
    return (start);
  end.o = convert_orientation_to_range( start.o - move.rotation );
  end.x = start.x - cos(end.o) * move.forward - sin(end.o) * move.sideward;
  end.y = start.y - sin(end.o) * move.forward + cos(end.o) * move.sideward;
  return(end);
}

int
inside_polygon( POLYGON polygon, POINT2 p )
{
  int counter = 0;
  int i;
  double xinters;
  POINT2 p1,p2;

  p1 = polygon.pt[0];
  for (i=1;i<=polygon.numpoints;i++) {
    p2 = polygon.pt[i % polygon.numpoints];
    if (p.y > MIN(p1.y,p2.y)) {
      if (p.y <= MAX(p1.y,p2.y)) {
        if (p.x <= MAX(p1.x,p2.x)) {
          if (p1.y != p2.y) {
            xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
            if (p1.x == p2.x || p.x <= xinters)
              counter++;
          }
        }
      }
    }
    p1 = p2;
  }

  if (counter % 2 == 0)
    return( FALSE);
  else
    return( TRUE );
}

double
get_user_time( void )
{
  struct tms     tms_time;
  if ((int)times(&tms_time)==-1) {
    return(-1);
  }
  return( (double)tms_time.tms_utime / (double)CLK_TCK );
}
  
void
convert_time( double tval, struct timeval *time )
{
  time->tv_sec = (long) floor(tval);
  time->tv_usec = (long) ( (tval-(double) time->tv_sec) * 1000000 );
}

double
double_time( struct timeval time )
{
  return( (double) (time.tv_sec) + (double) (time.tv_usec/1000000.0) );
}

double
sick_laser_fov( int num )
{
  switch( num ) {
  case(180):
    /* 180 degrees */
    return(M_PI);
    break;
  case(181):
    return(M_PI);
    break;
  case(360):
    return(M_PI);
    break;
  case(361):
    return(M_PI);
    break;
  case(401):
    /* 100 degrees */
    return(1.74532925199432957692);
    break;
  case(721):
    return(M_PI);
    break;
  default:
    return(M_PI);
    break;
  }
}

double
sick_laser_angle_diff( int num, double fov )
{
  switch( num ) {
  case(180):
    return(fov / (double) num);
    break;
  case(360):
    return(fov / (double) num);
    break;
  case(400):
    return(fov / (double) num);
    break;
  default:
    return(fov / (double) (num-1));
    break;
  }
}

double
kl_distance( GAUSSIAN g1, GAUSSIAN g2 )
{
  return( 0.5 * ( log( g2.sigma / g1.sigma ) - 1 + ( g1.sigma / g2.sigma ) +
		 ( (g1.mu-g2.mu)*(g1.mu-g2.mu) / g2.sigma ) ) );
}

double
rpos2_distance( RPOS2 p1, RPOS2 p2 )
{
  return sqrt( (p1.x-p2.x)*(p1.x-p2.x) +
	       (p1.y-p2.y)*(p1.y-p2.y) );
}
  
char *
extended_filename( char * filename )
{
  struct timeval     time;
  struct tm        * date;
  struct utsname     info;

  static char        fname[MAX_NAME_LENGTH];
  static char        addstr[MAX_NAME_LENGTH];

  int                i, j, ctr;

  ctr = 0;
  for (i=0;i<(int)strlen(filename);i++) {
    if (filename[i]=='%') {
      switch (filename[i+1]) {
      case 'd':	
	gettimeofday( &time, NULL );
	date = localtime( &(time.tv_sec) );
	snprintf( addstr, MAX_NAME_LENGTH, "%d-%s%d-%s%d",
		  1900+date->tm_year,
		  (date->tm_mon<10)?"0":"",
		  date->tm_mon,
		  (date->tm_mday<10)?"0":"",
		  date->tm_mday );
	
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 't':	
	gettimeofday( &time, NULL );
	date = localtime( &(time.tv_sec) );
	snprintf( addstr, MAX_NAME_LENGTH, "%s%d:%s%d",
		  (date->tm_hour<10)?"0":"",
		  date->tm_hour,
		  (date->tm_min<10)?"0":"",
		  date->tm_min );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 'h':	
	if (!uname( &info )) {
	  for (j=0;j<(int)strlen(info.nodename);j++) {
	    if (ctr<MAX_NAME_LENGTH)
	      fname[ctr++]=info.nodename[j];
	  }
	}
	i++;
	break;
      case 'n':	
	if (!uname( &info )) {
	  for (j=0;j<(int)strlen(info.nodename);j++) {
	    if (ctr<MAX_NAME_LENGTH)
	      fname[ctr++]=info.nodename[j];
	  }
	}
	i++;
	break;
      case 's':	
	if (!uname( &info )) {
	  for (j=0;j<(int)strlen(info.sysname);j++) {
	    if (ctr<MAX_NAME_LENGTH)
	      fname[ctr++]=info.sysname[j];
	  }
	}
	i++;
	break;
      case 'r':	
	if (!uname( &info )) {
	  for (j=0;j<(int)strlen(info.release);j++) {
	    if (ctr<MAX_NAME_LENGTH)
	      fname[ctr++]=info.release[j];
	  }
	}
	i++;
	break;
      case 'm':	
	if (!uname( &info )) {
	  for (j=0;j<(int)strlen(info.machine);j++) {
	    if (ctr<MAX_NAME_LENGTH)
	      fname[ctr++]=info.machine[j];
	  }
	}
	i++;
	break;
      default:
	if (ctr<MAX_NAME_LENGTH)
	  fname[ctr++]=filename[i];
	break;
      }
    } else {
      if (ctr<MAX_NAME_LENGTH)
	fname[ctr++]=filename[i];
    }
  }
  if (ctr<MAX_NAME_LENGTH) {
    fname[ctr++]='\0';
  } else {
    fname[MAX_NAME_LENGTH-1]='\0';
  }
  return(fname);
}

char *
printable_filename( char * filename )
{
  static char        fname[MAX_NAME_LENGTH];
  static char        addstr[MAX_NAME_LENGTH];

  int                i, j, ctr;

  ctr = 0;
  for (i=0;i<(int)strlen(filename);i++) {
    if (filename[i]=='%') {
      switch (filename[i+1]) {
      case 'd':	
	snprintf( addstr, MAX_NAME_LENGTH, "<DATE>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 't':	
	snprintf( addstr, MAX_NAME_LENGTH, "<TIME>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 'h':	
 	snprintf( addstr, MAX_NAME_LENGTH, "<HOST>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 'n':	
 	snprintf( addstr, MAX_NAME_LENGTH, "<NODENAME>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 's':	
 	snprintf( addstr, MAX_NAME_LENGTH, "<SYSNAME>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 'r':	
 	snprintf( addstr, MAX_NAME_LENGTH, "<RELEASE>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 'm':	
 	snprintf( addstr, MAX_NAME_LENGTH, "<MACHINE>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      default:
	if (ctr<MAX_NAME_LENGTH)
	  fname[ctr++]=filename[i];
	break;
      }
    } else {
      if (ctr<MAX_NAME_LENGTH)
	fname[ctr++]=filename[i];
    }
  }
  if (ctr<MAX_NAME_LENGTH) {
    fname[ctr++]='\0';
  } else {
    fname[MAX_NAME_LENGTH-1]='\0';
  }
  return(fname);
}

FILE *
fopen_filename( char * filename, char * mode )
{
  char fname[MAX_NAME_LENGTH];
  strncpy( fname, extended_filename( filename ), MAX_NAME_LENGTH );
  return(fopen( fname, mode));
}
    
  
double
gps_degree_abs_decimal_100( double degree_minute )
{
  double gps_deg, gps_min, gps_sec;
  gps_deg = floor( degree_minute/100.0 );
  gps_min = floor( degree_minute )-gps_deg*100;
  gps_sec = degree_minute-floor(degree_minute);
  return(gps_deg + (gps_min / 60.0) + (gps_sec / 60.0)); 
}

double
gps_degree_abs_decimal( double degree_minute )
{
  double gps_deg, gps_min, gps_sec;
  gps_deg = floor( degree_minute );
  gps_min = floor( degree_minute )-gps_deg;
  gps_sec = degree_minute-floor(degree_minute);
  return(gps_deg + (gps_min / 60.0) + (gps_sec / 60.0)); 
}

double
gps_degree_decimal( double degree_minute, char orient )
{
  if ( (orient=='w') || (orient=='s') ||
       (orient=='W') || (orient=='S') ) {
    return(-gps_degree_abs_decimal(degree_minute));
  } else {
    return(gps_degree_abs_decimal(degree_minute));
  }
}

VECTOR2
compute_laser_abs_point( RPOS2 rpos, double val, RMOVE2 offset, double angle )
{
  VECTOR2 abspt;
  abspt.x =
    rpos.x + 
    cos( angle+offset.rotation+rpos.o ) * val;
  abspt.y =
    rpos.y + 
    sin( angle+offset.rotation+rpos.o ) * val;
  return(abspt);
}



LASER_COORD2
compute_laser2d_coord( LASERSENS2_DATA lsens, int i )
{
  double        val;
  LASER_COORD2  coord;
  RPOS2         rpos, npos;

  rpos  = compute_rpos2_with_movement2( lsens.estpos,
					lsens.laser.offset );
  npos.x = 0.0;
  npos.y = 0.0;
  npos.o = 0.0;

  val = lsens.laser.val[i];
  coord.relpt = compute_laser_abs_point( npos, val,
					 lsens.laser.offset,
					 lsens.laser.angle[i] );
  coord.abspt = compute_laser_abs_point( rpos, val,
					 lsens.laser.offset,
					 lsens.laser.angle[i] );
  return(coord);
}

void
compute_rec2d_coordpts( REC2_DATA *rec )
{
  int i, j;
  for (j=0;j<rec->numlaserscans;j++) {
    if (rec->lsens[j].coord==NULL) {
      rec->lsens[j].coord =
	(LASER_COORD2 *) malloc( rec->lsens[j].laser.numvalues *
				 sizeof(LASER_COORD2) );
      for (i=0;i<rec->lsens[j].laser.numvalues;i++) {
	rec->lsens[j].coord[i] = compute_laser2d_coord(rec->lsens[j], i);
      }
    }
  }
}

