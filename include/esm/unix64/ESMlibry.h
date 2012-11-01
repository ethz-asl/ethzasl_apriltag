/*#============================================================================
//#
//#       Filename:  ESMlibry.h
//#
//#    Description:  Tracking structures and functions prototypes 
//#
//#
//#         Author:  Selim BENHIMANE & Ezio MALIS
//#        Company:  INRIA, Sophia-Antipolis, FRANCE
//#          Email:  Selim.Benhimane@in.tum.de
//#          Email:  Ezio.Malis@inria.fr
//#
//#
//#==========================================================================*/

#ifndef __ESMLIBRY_H
#define __ESMLIBRY_H

// #####   HEADER FILE INCLUDES   #############################################

// #####   MACROS  -  LOCAL TO THIS SOURCE FILE   #############################

// #####   EXPORTED MACROS   ##################################################

// #####   EXPORTED DATA TYPES   ##############################################

// #####   EXPORTED TYPE DEFINITIONS   ########################################

// Image Structure

typedef struct imageStruct {
  int    cols;  //!< width  / cols 
  int    rows;  //!< height / rows
  float *data;  //!< data
} imageStruct;

// Tracking Structure

typedef struct trackStruct {
  imageStruct** images; //!< various images  
  float**    trackdata; //!< various data
  int            miter; //!< iteration number 
  int            mprec; //!< tracking precision 
  float       homog[9]; //!< the homography computed 
} trackStruct;

// #####   EXPORTED VARIABLES   ###############################################

// #####   EXPORTED FUNCTION DECLARATIONS   ##################################

// Image Functions

int    MallImage    (imageStruct *image, int sizx, int sizy);
int    FreeImage    (imageStruct *image);
int    GetImageRows (imageStruct *image);
int    GetImageCols (imageStruct *image);
float* GetImageData (imageStruct *image);

// Input / Output Image Functions

int SavePgm (char *filename, imageStruct *image);
int ReadPgm (char *filename, imageStruct *image);

// Tracking Functions

int MallTrack     (trackStruct *trackArgs, imageStruct *image, int posx, int posy, int sizx, int sizy, int miter, int mprec);
int MakeTrack     (trackStruct *trackArgs, imageStruct *image);
int FreeTrack     (trackStruct *trackArgs);

int MallTrackMask (trackStruct *trackArgs, imageStruct *image, imageStruct *mask, int posx, int posy, int sizx, int sizy, int miter, int mprec);
int MakeTrackMask (trackStruct *trackArgs, imageStruct *image);
int FreeTrackMask (trackStruct *trackArgs);

imageStruct* GetPatm (trackStruct *trackArgs); //!< the mask pattern
imageStruct* GetPatr (trackStruct *trackArgs); //!< the reference pattern
imageStruct* GetPatc (trackStruct *trackArgs); //!< the current pattern
float        GetZNCC (trackStruct *trackArgs); //!< the zncc

#endif
