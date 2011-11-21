#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <time.h>

float const L1 = 3.0;
float const L2 = 4.0;
float const L3 = 1.0;
float const L = 1.0;

float const OriginX = 5;
float const OriginY = 2;

float EndEffectorX[6] = { 8.5, 8.5, 6.5, 6.5, 8.5, 8.5 };
float EndEffectorY[6] = { 1.0, 2.0, 2.0, 6.0, 6.0, 5.0 };

float q1PosX; float q1PosY;
float q2PosX; float q2PosY;
float q3PosX; float q3PosY;

float GridX = 10;
float GridY = 10;

float deltaDist = 0.02;
float dt = 0.02;
int numWaypoints;

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
   float distx, disty, dist;
   float ex, ey;
   int num;

   setPlotSettings(); 
   
   FILE* pJoints; FILE* pEE; FILE* pJoint2; FILE* pJoint3;
   pJoints = fopen( "Joints.dat", "w" );
   pEE = fopen( "EndEffector.dat", "w" );
   pJoint2 = fopen("Joint2.dat", "w");
   pJoint3 = fopen("Joint3.dat", "w");

   float t = 0;
   int count = 0; 
   char name[50];

   for( int i = 0; i < 5; i++ )
   {
      distx = EndEffectorX[i+1] - EndEffectorX[i];
      disty = EndEffectorY[i+1] - EndEffectorY[i];
      dist = sqrt( distx*distx + disty*disty );
      num = dist/deltaDist;

      for( int j = 0; j <= num; j++ )
      { 

         ex = EndEffectorX[i] + j*distx/num;
         ey = EndEffectorY[i]+ j*disty/num;                 

         getJointsValue( ex, ey, q1, q2, q3 );
         getJointsPosition( q1, q2, q3 );


         if( count < 10 )
         { sprintf( name, "00000%d.png", count); }        
         else if( count < 100 )
         { sprintf( name, "0000%d.png", count); }
         else if( count < 1000 ) 
         { sprintf( name, "000%d.png", count); }   
         printf("set output '%s' \n", name);
         printf("set multiplot \n");
   
         plotWorld(q1,q2,q3);
         plotRobot();
         printf("unset multiplot \n");

         for( int k = 0; k < 1000; k++ );
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

   q3PosX = q2PosX;
   q3PosY = q2PosY - L3;
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

/**
 */
void plotRobot()
{
         printf("plot '-' with linespoints lc 1 lw 4 pt 7 ps 2 \n" );  
         printf("%.3f  %.3f \n", OriginX, OriginY );
         printf("%.3f  %.3f \n", q1PosX, q1PosY );
         printf("%.3f  %.3f \n", q2PosX, q2PosY );
         printf("%.3f  %.3f \n", q3PosX, q3PosY );
         printf("e \n");

}
