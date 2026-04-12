//file for all the camber position data and lookup table for servo positions based on angle of attack

#ifndef CAMBER_POSITION_H
#define CAMBER_POSITION_H

// struct for aoa and respective position
struct ShapeData {
  float angleOfAttack; // degrees
  int servoPosition;   // 0-180 degrees
};

// dataset of optimized shapes dataset
const int NUM_SHAPES = 35;  //


//angle rangles 0-180
ShapeData lookupTable[NUM_SHAPES] = {
    {-20, -30},
    {-19, -30},
    {-18, -30},
    {-17, -28},
    {-16, -28},
    {-15, -30},
    {-13, -26},
    {-12, -36},
    {-11, -40},
    {-10, -40},
    {-9, -40},
    {-8, -40},
    {-7, -40},
    {-6, -36},
    {-5, -30},
    {-4, -30},
    {-3, -16},
    {-2, -14},
    {-1, -10},
    {0, -6}, //-6
    {1, -6},
    {2, -8},
    {3, -10},
    {4, -10},
    {5, -6},
    {6, -4},
    {7, 0},
    {8, 0},
    {9, 8},
    {10, 10},
    {11, 10},
    {12, 12},
    {13, 14},
    {14, 16},
    {15, 20}
};

#endif