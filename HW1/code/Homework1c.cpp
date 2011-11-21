#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <time.h>

float const L1 = 3.0;
float const L2 = 4.0;
float const L3 = 1.0;

float const OriginX = 5;
float const OriginY = 2;

float cf = 3.1416/180;
int const nc = 6;
float q1w[nc + 1] = {    75.522*cf,       90*cf,       90*cf,       90*cf,    150*cf,        150*cf,      97.028*cf   };
float q2w[nc + 1] = {  -122.090*cf, -122.090*cf, -122.090*cf,     -150*cf,    -150*cf,   -82.218*cf,      -82.218*cf   };
float q3w[nc + 1] = {   -43.432*cf, -43.432*cf,  -104.811*cf, -104.811*cf, -104.811*cf,  -104.811*cf,     -104.811*cf   };



float q1PosX; float q1PosY;
float q2PosX; float q2PosY;
float q3PosX; float q3PosY;

float GridX = 10;
float GridY = 10;

float deltaDist = 0.02;
float dt = 1/30.0;
int numWaypoints;
double L = 1.0;

#define PI 3.1416

void getJointsValue( const float &EEX, const float &EEY, float &q1, float &q2, float &q3 );
void getJointsPosition( const float &q1, const float &q2, const float &q3 );
void setPlotSettings();
void plotWorld( const float &q1, const float &q2, const float &q3 );
void plotRobot();

/**
 * @function main
 */
int main( int argc, char* argv[] )
{
   float q1,q2,q3;
   float dq1, dq2, dq3,dq;
   float ex, ey;
   int num;

   setPlotSettings(); 
   
   FILE* pJoints; FILE* pEE; FILE* pJoint2; FILE* pJoint3;
   pJoints = fopen( "Jointsc.dat", "w" );
   pEE = fopen( "EndEffectorc.dat", "w" );
   pJoint2 = fopen("Joint2c.dat", "w");
   pJoint3 = fopen("Joint3c.dat", "w");

   float t = 0;

   int count = 0; 
   char name[50];

   for( int i = 0; i < nc; i++ )
   {
  
      dq1 = q1w[i] - q1w[i+1];
      dq2 = q2w[i] - q2w[i+1];
      dq3 = q3w[i] - q3w[i+1];

      dq = fabs(dq1) + fabs(dq2) + fabs(dq3);
      num = dq/(3.14/180.0);

      for( int j = 0; j <= num; j++ )
      { 
         q1 = q1w[i] - j*dq1/num;
         q2 = q2w[i] - j*dq2/num;
         q3 = q3w[i] - j*dq3/num;

         if( count < 10 )
         { sprintf( name, "00000%d.png", count); }        
         else if( count < 100 )
         { sprintf( name, "0000%d.png", count); }
         else if( count < 1000 ) 
         { sprintf( name, "000%d.png", count); }               

         getJointsPosition( q1, q2, q3 );
         printf("set output '%s' \n", name);
         printf("set multiplot \n");
         plotWorld(q1,q2,q3);
         plotRobot();
         printf("unset multiplot \n");
         fprintf( pJoints, "%.3f %.3f %.3f %.3f \n", t, q1, q2, q3 );
         fprintf( pJoint2, "%.3f %.3f \n", q1PosX, q1PosY );
         fprintf( pJoint3, "%.3f %.3f \n", q2PosX, q2PosY );
         fprintf( pEE, "%.3f %.3f \n", q3PosX, q3PosY );
         t+=dt;
         count++;
      }
   }

   printf("print 'done' \n");
   fclose( pJoints ); fclose( pEE ); fclose( pJoint2 ); fclose( pJoint3 );
   return 0; 
}

/**
 * @function setPlotSettings
 */
void setPlotSettings()
{
   printf("set xtics 1 \n");
   printf("set ytics 1 \n");
   printf("set xrange [%f:%f] \n", 0.0, GridX );
   printf("set yrange [%f:%f] \n", 0.0, GridY );
   printf("set grid \n");
   printf("set terminal png \n");
}

/**
 * @function calculateJoints
 */
void getJointsValue( const float &EEX, const float &EEY, float &q1, float &q2, float &q3 )
{
   float hyp = sqrt( pow( (EEX - OriginX),2) + pow( (L3 + EEY - OriginY),2) );

   q2 = acos(  ( pow(L1,2) + pow(L2,2) - pow(hyp,2) )/(2*L1*L2) ) - PI;
   q1 = acos( ( pow(L1,2) + pow(hyp,2) - pow(L2,2) )/(2*L1*hyp) ) + atan2( L3 + EEY - OriginY, EEX - OriginX );
   q3 = -PI/2.0 - q1 - q2;
}

/**
 */
void getJointsPosition( const float &q1, const float &q2, const float &q3 )
{
   q1PosX = OriginX + L1*cos(q1);
   q1PosY = OriginY + L1*sin(q1); 

   q2PosX = q1PosX + L2*cos(q1+q2);
   q2PosY = q1PosY + L2*sin(q1+q2);

   q3PosX = q2PosX + L3*cos(q1+q2+q3);
   q3PosY = q2PosY + L3*sin(q1+q2+q3);
}

/**
 * @function plotWorld
 */
void plotWorld( const float &q1, const float &q2, const float &q3 )
{

   printf("plot '-' with lines lc 7 lw 4 \n" );  
   printf("%.3f  %.3f \n", 5.0, 0.0 );
   printf("%.3f  %.3f \n", 5.0, 2.0 );
   printf("e \n");

   printf("plot '-' with lines lc 7 lw 4 \n" );  
   printf("%.3f  %.3f \n", 7.5, 0.5 );
   printf("%.3f  %.3f \n", 7.5, 0.0 );
   printf("%.3f  %.3f \n", 9.5, 0.0 );
   printf("%.3f  %.3f \n", 9.5, 0.5 );
   printf("e \n");

   printf("plot '-' with lines lc 7 lw 4 \n" );  
   printf("%.3f  %.3f \n", 7.5, 4.5 );
   printf("%.3f  %.3f \n", 7.5, 4.0 );
   printf("%.3f  %.3f \n", 9.5, 4.0 );
   printf("%.3f  %.3f \n", 9.5, 4.5 );
   printf("e \n");

   printf("plot '-' with lines lc 3 lw 4 \n" );  
   printf("%.3f  %.3f \n", q3PosX + L/2.0*sin( q1 + q2 + q3 ), q3PosY - L/2.0*cos( q1 + q2 + q3 ) );
   printf("%.3f  %.3f \n", q3PosX - L/2.0*sin( q1 + q2 + q3 ), q3PosY + L/2.0*cos( q1 + q2 + q3 ) );
   printf("%.3f  %.3f \n", q3PosX + L*cos(q1+q2+q3) - L/2.0*sin( q1 + q2 + q3 ), q3PosY + L*sin(q1+q2+q3) + L/2.0*cos( q1 + q2 + q3 ) );
   printf("%.3f  %.3f \n", q3PosX + L*cos(q1+q2+q3) + L/2.0*sin( q1 + q2 + q3 ), q3PosY + L*sin(q1+q2+q3) - L/2.0*cos( q1 + q2 + q3 ) );
   printf("%.3f  %.3f \n", q3PosX + L/2.0*sin( q1 + q2 + q3 ), q3PosY - L/2.0*cos( q1 + q2 + q3 ) );
   printf("e \n");
}

void plotRobot()
{
         printf("plot '-' with linespoints lc 1 lw 4 pt 7 ps 2 \n" );  
         printf("%.3f  %.3f \n", OriginX, OriginY );
         printf("%.3f  %.3f \n", q1PosX, q1PosY );
         printf("%.3f  %.3f \n", q2PosX, q2PosY );
         printf("%.3f  %.3f \n", q3PosX, q3PosY );
         printf("e \n");

}
