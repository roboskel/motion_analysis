/* Algorithm parameters :
 *
 * COMMAND                  : Number of operation to be executed (default for RADIO algorithm: 5)
 * ALGO_?                   : If defined, algorithm No ? is enabled in command 5
 * SENSITIVITY              : How many changed pixels in each block will mean the block is chamged
 * STANDING_PERSON_HEIGHT   : If we detect changes above this line, the person is standing-up
 * OUTOFBED_LEFT / RIGHT    : If centre of activity is outside these columns, the person is off-bed
 * CUPX / CUPY              : Coordinates of the centre of the medication cup region
 * CUPR                     : Size (in pixels) of the region where the medication cup is placed
 * CUPTHRESHOLD             : Threshold in difference in pixel intensityto detect a chnage in the cup region
 * CUPTHRSCOUNT             : Count of changed pixels that mean a cup has been moved   
 *
 */ 
#ifndef PROCESS_H
#define PROCESS_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define COMMAND 5
#define ALGO_1 //bed detection
#define ALGO_2 //cup detection
#define SENSITIVITY 30
//#define STANDING_PERSON_HEIGHT 150
//#define OUTOFBED_LEFT 150
//#define OUTOFBED_RIGHT 450

//#define CUPX 500
//#define CUPY 200
//#define CUPR 40
#define CUPTHRESHOLD 80
#define CUPTHRSCOUNT 30

#define REDV   0
#define GREENV 1
#define BLUEV  2

// Detect Human Movement
#define MODE_HUMAN_MOVEMENT 0
// Store Initial Object Position
#define MODE_INIT_OBJECT 1
// Detect Object Movement
#define MODE_DETECT_OBJECT_MOVEMENT 2


/* Global variables
 *
 */
 
//unsigned int cup[CUPR*CUPR*4];
int *cup;
unsigned char tempframe[640*480*3];
unsigned int STANDING_PERSON_HEIGHT = 0;
unsigned int OUTOFBED_LEFT = 0;
unsigned int OUTOFBED_RIGHT = 0;

unsigned int CUPX = 0;
unsigned int CUPY = 0;
unsigned int CUPR = 0;

unsigned int BOUNDARIES_CONFIG = 0;
unsigned int CONFIGURATION_DONE = 0;
unsigned int FIRST_ROUND = 1;




//We need a node that publishes an object_state topic with values MODE_HUMAN_MOVEMENT, MODE_INIT_OBJECT, MODE_DETECT_OBJECT_MOVEMENT for the states 
// for definitions see above.
int placed = MODE_HUMAN_MOVEMENT;

unsigned int x1p,x2p,y1p,y2p,cxp, cyp;
unsigned int top, centerx, centery;
unsigned int gotup, cupmoved;

int process_function (unsigned char *rgb_a, unsigned char *rgb_b,unsigned char command, unsigned int showanno, unsigned char *unedited, int *shapesxy);
void process(unsigned char *rgb_a, unsigned char *rgb_b,unsigned int index, unsigned int showanno, int &bed_ans, int &cup_ans, unsigned char *unedited, int *shapesxy);

#endif
