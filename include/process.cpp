#include "process.h"

/* traffic_light(rgb_a,state)
 *
 * shows a red,blue,green traffic light bottom right
 *
 * rgb_a     = the image frame  ( array[640*480*3] )
 * state     = 0: all off, 1: red, 2: blue, 4:green. Binary combined
 *
 */

int traffic_light(unsigned char *rgb_a, unsigned int state){

    unsigned int x,y,cyp,cxp;
    int tmpi;

    if (state & 0x01) {
        cxp = 610;
        cyp = 360;
        for (x=cxp-30;x<cxp+30;x++) {
            for (y=cyp-30;y<cyp+30;y++) {
                tmpi = 255 - ((x-cxp)*(x-cxp)+(y-cyp)*(y-cyp))/3;
                if (tmpi>0) rgb_a[(x*480+y)*3+BLUEV ] = tmpi;
            }
        }
    }
    if (state & 0x02) {
        cxp = 610;
        cyp = 400;
        for (x=cxp-30;x<cxp+30;x++) {
            for (y=cyp-30;y<cyp+30;y++) {
                tmpi = 255 - ((x-cxp)*(x-cxp)+(y-cyp)*(y-cyp))/3;
                if (tmpi>0) rgb_a[(x*480+y)*3+REDV ] = tmpi;
            }
        }
    }
    if (state & 0x04) {
        cxp = 610;
        cyp = 440;
        for (x=cxp-30;x<cxp+30;x++) {
            for (y=cyp-30;y<cyp+30;y++) {
                tmpi = 255 - ((x-cxp)*(x-cxp)+(y-cyp)*(y-cyp))/3;
                if (tmpi>0) rgb_a[(x*480+y)*3+GREENV] = tmpi;
            }
        }
    }
}



/* process_function(grb_a, rgb_b,command, showanno)
 *
 * processes two frames
 *
 * rgb_a     = the first frame  ( array[640*480*3] )
 * rgb_b     = the second frame ( array[640*480*3] )
 * command   = a specific operation to be applied on the frame
 * showanno  = 0 : no image annotations, 1 : show chnaged blocks, 2 : show all annotations
 * unedited  = a masked frame, containing only the area inside the rectangle
 *
 */
int process_function (unsigned char *rgb_a, unsigned char *rgb_b,unsigned char command, unsigned int showanno, unsigned char *unedited, int *shapesxy) {


    unsigned int x,y;
                     
    unsigned long int cx, cy, x1,x2,y1,y2,n,nn;

	switch(command) {
		case 0 : // do nothing
			 break;		
		case 1 : // copy a to b
                {
                for (y=0;y<480;y++) {
                  for (x=0;x<640;x++) {
                    rgb_b[(x*480+y)*3+REDV  ] = rgb_a[(x*480+y)*3+REDV  ];
                    rgb_b[(x*480+y)*3+GREENV] = rgb_a[(x*480+y)*3+GREENV];
                    rgb_b[(x*480+y)*3+BLUEV ] = rgb_a[(x*480+y)*3+BLUEV ];
                  }
                }
                }
                break;
        case 2 : // diff with b and store in a
                {
                for (y=0;y<480;y++) {
                  for (x=0;x<640;x++) {
                    rgb_a[(x*480+y)*3+REDV  ] = abs(rgb_a[(x*480+y)*3+REDV  ] - rgb_b[(x*480+y)*3+REDV  ] );
                    rgb_a[(x*480+y)*3+GREENV] = abs(rgb_a[(x*480+y)*3+GREENV] - rgb_b[(x*480+y)*3+GREENV] );
                    rgb_a[(x*480+y)*3+BLUEV ] = abs(rgb_a[(x*480+y)*3+BLUEV ] - rgb_b[(x*480+y)*3+BLUEV ] );
                  }
                }
                }
                break;
        case 3 : // diff with b and store in a, store previous a in b
                 // copy a to tempframe
                 {
                for (y=0;y<480;y++) {
                  for (x=0;x<640;x++) {
                    tempframe[(x*480+y)*3+REDV  ] = rgb_a[(x*480+y)*3+REDV  ];
                    tempframe[(x*480+y)*3+GREENV] = rgb_a[(x*480+y)*3+GREENV];
                    tempframe[(x*480+y)*3+BLUEV ] = rgb_a[(x*480+y)*3+BLUEV ];
                  }
                }
                // copy diff(a,b) to a
                for (y=0;y<480;y++) {
                  for (x=0;x<640;x++) {
                    rgb_a[(x*480+y)*3+REDV  ] = abs(rgb_a[(x*480+y)*3+REDV  ] - rgb_b[(x*480+y)*3+REDV  ] );
                    rgb_a[(x*480+y)*3+GREENV] = abs(rgb_a[(x*480+y)*3+GREENV] - rgb_b[(x*480+y)*3+GREENV] );
                    rgb_a[(x*480+y)*3+BLUEV ] = abs(rgb_a[(x*480+y)*3+BLUEV ] - rgb_b[(x*480+y)*3+BLUEV ] );
                  }
                }
                // copy tempframe to b
                for (y=0;y<480;y++) {
                  for (x=0;x<640;x++) {
                    rgb_b[(x*480+y)*3+REDV  ]  = tempframe[(x*480+y)*3+REDV  ];
                    rgb_b[(x*480+y)*3+GREENV]  = tempframe[(x*480+y)*3+GREENV];
                    rgb_b[(x*480+y)*3+BLUEV ]  = tempframe[(x*480+y)*3+BLUEV ];
                  }
                }
                }
                break;       
        case 4 : // show centre of activity
                {
                
                cx=0;
                cy=0;     
                n = 0;
                 
                for (y=0;y<480;y++) {
                  for (x=0;x<640;x++) {
                    tempframe[(x*480+y)*3+REDV  ] = rgb_a[(x*480+y)*3+REDV  ];
                    tempframe[(x*480+y)*3+GREENV] = rgb_a[(x*480+y)*3+GREENV];
                    tempframe[(x*480+y)*3+BLUEV ] = rgb_a[(x*480+y)*3+BLUEV ];
                  }
                }
                // copy diff(a,b) to a
                for (y=0;y<480;y++) {
                  for (x=0;x<640;x++) {
                    if ((abs(rgb_a[(x*480+y)*3+REDV  ] - rgb_b[(x*480+y)*3+REDV  ] ))>40) { 
                        rgb_a[(x*480+y)*3+REDV  ] = 70; 
                        cx = cx + x;
                        cy = cy + y;
                        n = n + 1;
                    }
                    else 
                        rgb_a[(x*480+y)*3+REDV  ] = 0;
                    
                    if ((abs(rgb_a[(x*480+y)*3+GREENV] - rgb_b[(x*480+y)*3+GREENV] ))>40) {
                        rgb_a[(x*480+y)*3+GREENV] = 70; 
                        cx = cx + x;
                        cy = cy + y;
                        n = n + 1;
                    }
                    else 
                        rgb_a[(x*480+y)*3+GREENV] = 0;
                        
                    if ((abs(rgb_a[(x*480+y)*3+BLUEV ] - rgb_b[(x*480+y)*3+BLUEV ] ))>40) {
                        rgb_a[(x*480+y)*3+BLUEV ] = 70; 
                        cx = cx + x;
                        cy = cy + y;
                        n = n + 1;
                    }
                    else 
                        rgb_a[(x*480+y)*3+BLUEV ] = 0;
                  }
                }
                
                if (n>0) {
                    cx = cx/n;
                    cy = cy/n;
                }
                
                if ((cx<6)||(cx>634)||(cy<6)||(cy>474)) {
                // do nothing
                } else {
                     for (x=cx-5;x<cx+5;x++) {
                       for (y=cy-5;y<cy+5;y++) {
                          rgb_a[(x*480+y)*3+BLUEV ] = 255; 
                       }
                     }
                }
                
                // copy tempframe to b
                for (y=0;y<480;y++) {
                  for (x=0;x<640;x++) {
                    rgb_b[(x*480+y)*3+REDV  ]  = tempframe[(x*480+y)*3+REDV  ];
                    rgb_b[(x*480+y)*3+GREENV]  = tempframe[(x*480+y)*3+GREENV];
                    rgb_b[(x*480+y)*3+BLUEV ]  = tempframe[(x*480+y)*3+BLUEV ];
                  }
                }
                }
                break; 
        case 5 : // Main RADIO algorithm for events 1 and 2
                 // Event 1 : Time to get out of bed
                 // Event 2 : Medication cup chnage
                {
                
                                
                unsigned int xx,yy;   
                unsigned int s,ss;
                int tmpi;
                
                x1=638; x2=1;
                y1=438; y2=1;
                 
                cx = 0; cy = 0; nn = 0;
                 
                // Store rgb in tempframe (make sure we do not loose it due to annotation)
                for (y=0;y<480;y++) {
                  for (x=0;x<640;x++) {
                    tempframe[(x*480+y)*3+REDV  ] = rgb_a[(x*480+y)*3+REDV  ];
                    tempframe[(x*480+y)*3+GREENV] = rgb_a[(x*480+y)*3+GREENV];
                    tempframe[(x*480+y)*3+BLUEV ] = rgb_a[(x*480+y)*3+BLUEV ];
                  }
                }

                #ifdef ALGO_1
		
		// Define ss to any value >0 and <(480/4). 
		// Reasonable results expected only for values in between 2 and 10.
                ss= 4;           // +/- from center of each block
		
		s = ss + ss + 1; // size of each block
                
                // find diff in squares of size s, and find top,bot,right,left
                for (yy=s;yy<480-s;yy+=s) {
                  for (xx=s;xx<640-s;xx+=s) {
                                    
                    n=0;
                    
                    for (x=xx-ss;x<xx+s+1;x++) {
                    for (y=yy-ss;y<yy+ss+1;y++) {

                
                    if ((abs(rgb_a[(x*480+y)*3+REDV  ] - rgb_b[(x*480+y)*3+REDV  ] ))>40) { 
                        if (showanno>0) rgb_a[(x*480+y)*3+REDV  ] = 70; 
                        n = n + abs(rgb_a[(x*480+y)*3+REDV  ] - rgb_b[(x*480+y)*3+REDV  ] );
                    }
                    else 
                        if (showanno>0) rgb_a[(x*480+y)*3+REDV  ] = rgb_a[(x*480+y)*3+REDV  ] /4;
                    
                    if ((abs(rgb_a[(x*480+y)*3+GREENV] - rgb_b[(x*480+y)*3+GREENV] ))>40) {
                        if (showanno>0) rgb_a[(x*480+y)*3+GREENV] = 70; 
                        n = n + abs(rgb_a[(x*480+y)*3+GREENV ] - rgb_b[(x*480+y)*3+GREENV ] );
                    }
                    else 
                        if (showanno>0) rgb_a[(x*480+y)*3+GREENV] = rgb_a[(x*480+y)*3+GREENV] /4;
                        
                    if ((abs(rgb_a[(x*480+y)*3+BLUEV ] - rgb_b[(x*480+y)*3+BLUEV ] ))>40) {
                        if (showanno>0) rgb_a[(x*480+y)*3+BLUEV ] = 70; 
                        n = n + abs(rgb_a[(x*480+y)*3+BLUEV  ] - rgb_b[(x*480+y)*3+BLUEV ] );
                    }
                    else 
                        if (showanno>0) rgb_a[(x*480+y)*3+BLUEV ] = rgb_a[(x*480+y)*3+BLUEV ] /4;
                        
                    }
                    }
                                    
                    n = n / (s*s);
                                    
                    if (n>SENSITIVITY) {
                        if (x<x1) x1=x;
                        if (x>x2) x2=x;
                        if (y<y1) y1=y;
                        if (y>y2) y2=y;
                        
                        cx = cx + x;
                        cy = cy + y;
                        nn = nn + 1;
                        
                        if (showanno>0) {
                            for (x=xx-ss;x<xx+ss;x++) {
                            for (y=yy-ss;y<yy+ss;y++) {
                                if (rgb_a[(x*480+y)*3+BLUEV ]<128)
                                    rgb_a[(x*480+y)*3+BLUEV ] = rgb_a[(x*480+y)*3+BLUEV ] + 128;
                                else
                                    rgb_a[(x*480+y)*3+BLUEV ] = 255;
                            }
                            }
                        }
                    }
                  }
                }

                
           
                // highlight bounding rectangle (top left , bot right)
                
                if ((x2>x1)&&(y2>y1)) //valid rectangle
                {
                    x1p=x1; y1p=y1; x2p=x2; y2p=y2;  top = y1p;
                    if (nn>0) { cxp=cx/nn; cyp=cy/nn; centerx=cxp; centery=cyp;}
                    
                }
                //else use previous

                //make sure we will not get crazy values for edges of rect
                if ((x1p<1)||(x2p>639)||(y1p<1)||(y2p>479))
                {
                    x1p=1;x2p=639;y1p=1;y2p=439; cxp= 320; cyp= 240;
                }

                //blacken all the unedited image except from the part inside the rectangle
                for(unsigned int x_=0;x_<640;x_++){
                    for(unsigned int y_=0;y_<480;y_++){
                        if((x_<x1p || x_>x2p) || (y_<y1p || y_>y2p)){
                            unedited[(x_*480+y_)*3+0] = 0; 
                            unedited[(x_*480+y_)*3+1] = 0; 
                            unedited[(x_*480+y_)*3+2] = 0; 
                        }
                    }
                }
                shapesxy[0] = x1p;
                shapesxy[1] = y1p;
                shapesxy[2] = x2p;
                shapesxy[3] = y2p;
                shapesxy[4] = cx;
                shapesxy[5] = cy;
                    
                if (showanno>1) {
                    for (x=x1p;x<x2p;x++) rgb_a[(x*480+y1p)*3+REDV ] = 255; 
                    for (x=x1p;x<x2p;x++) rgb_a[(x*480+y2p)*3+REDV ] = 255; 
                    for (y=y1p;y<y2p;y++) rgb_a[(x1p*480+y)*3+REDV ] = 255; 
                    for (y=y1p;y<y2p;y++) rgb_a[(x2p*480+y)*3+REDV ] = 255; 
                
                    for (x=2;x<638;x++) {
                        rgb_a[(x*480+STANDING_PERSON_HEIGHT)*3+GREENV] = 255;
                        //rgb_a[(x*480+top                   )*3+BLUEV] = 255; // uncomment this to show hight in red
                        if (top<STANDING_PERSON_HEIGHT) rgb_a[(x*480+top)*3+REDV] = 255;
                    }
                    for (y=2;y<438;y++) {
                        rgb_a[(OUTOFBED_LEFT*480 +y)*3+GREENV] = 255;
                        rgb_a[(OUTOFBED_RIGHT*480+y)*3+GREENV] = 255;
                    }
                
                
                    if (!((cxp<30)||(cxp>610)||(cyp<30)||(cyp>450))) {
                        for (x=cxp-30;x<cxp+30;x++) {
                            for (y=cyp-30;y<cyp+30;y++) {
                                tmpi = 255 - ((x-cxp)*(x-cxp)+(y-cyp)*(y-cyp))/3;
                                if (tmpi>0) rgb_a[(x*480+y)*3+GREENV ] = tmpi;
                            }
                        }
                    }
                    
                }
                 
                #else
                gotup = 0;
                top = 0;
                #endif
                #ifdef ALGO_2
                // Process the cup region
                int cupi=0;
                int cupd=0;
                int cupdiff=0;
                for (x=CUPX-CUPR; x<CUPX+CUPR; x++) {
                    for (y=CUPY-CUPR; y<CUPY+CUPR; y++) {
                        if (placed==MODE_INIT_OBJECT) { // the moment when the cup is placed
                            cup[cupi]=rgb_a[(x*480 + y)*3+REDV]+rgb_a[(x*480 + y)*3+GREENV]+rgb_a[(x*480 + y)*3+BLUEV];
                            cupi++;
                            cupd = 0;
                        }
                        else if (placed==MODE_DETECT_OBJECT_MOVEMENT) { // when we want to check
                            cupd = rgb_a[(x*480 + y)*3+REDV]+rgb_a[(x*480 + y)*3+GREENV]+rgb_a[(x*480 + y)*3+BLUEV];
                            cupd = abs(cupd - cup[cupi]);
                            if ( cupd>CUPTHRESHOLD ) {
                                cupdiff++;
                                if (showanno>0) rgb_a[(x*480 + y)*3+GREENV] = 255;
                            }
                            cupi++;
                        }
                    }
                }
                if ((placed==MODE_DETECT_OBJECT_MOVEMENT)&&(cupdiff>CUPTHRSCOUNT)) cupmoved=1; else cupmoved=0; 
                
                if (showanno==2) {
                   // Show region for cup
                    for (x=CUPX-CUPR; x<CUPX+CUPR; x++) {
                        rgb_a[(x*480 + CUPY + CUPR)*3+GREENV] = 255;
                        rgb_a[(x*480 + CUPY - CUPR)*3+GREENV] = 255;
                    }
                    for (y=CUPY-CUPR; y<CUPY+CUPR; y++) {
                        rgb_a[((CUPX+CUPR)*480 + y)*3+GREENV] = 255;
                        rgb_a[((CUPX-CUPR)*480 + y)*3+GREENV] = 255;
                    }
                    
                }
                
                #else
                cupmoved=0;
                #endif
                
                // copy tempframe to b
                for (y=0;y<480;y++) {
                  for (x=0;x<640;x++) {
                    rgb_b[(x*480+y)*3+REDV  ]  = tempframe[(x*480+y)*3+REDV  ];
                    rgb_b[(x*480+y)*3+GREENV]  = tempframe[(x*480+y)*3+GREENV];
                    rgb_b[(x*480+y)*3+BLUEV ]  = tempframe[(x*480+y)*3+BLUEV ];
                  }
                }
                }
                break;                 
 	}

	return 0;
}


void process(unsigned char *rgb_a, unsigned char *rgb_b,unsigned int index, unsigned int showanno, int &bed_ans, int &cup_ans, unsigned char *unedited, int *shapesxy) {
    
    unsigned int height, isup, oob;

    char str[1];

    cup = new int[CUPR*CUPR*4];    

    process_function (rgb_a, rgb_b,COMMAND, showanno, unedited, shapesxy);

    height = top;

    #ifdef ALGO_1
        if (height<STANDING_PERSON_HEIGHT) isup=1; else isup=0;
        if ((centerx<OUTOFBED_LEFT)||(centerx>OUTOFBED_RIGHT)) oob=1; else oob=0;

        //if ((isup==1)&&(gotup==0)&&(index>9))  { 
        //    gotup=1; 
        if (isup==1)  { 
            //printf("Standing [after %05d Frames]\n",index); 
            bed_ans = 1;
            traffic_light(rgb_a, 2);
        }
        
        if ((oob==1)&&(isup==1)){
            bed_ans = 2;
            //printf("Walking !!\n");
            traffic_light(rgb_a, 1);
        } 
        
        if ((oob==1)&&(isup==0)){
            bed_ans = 0;
            traffic_light(rgb_a, 4);
            //printf("* * * Warning : Out of Bed but not Standing up!  * * * \n");
        }
    #endif
    
    #ifdef ALGO_2
        if (cupmoved==1){
            cup_ans = 1;
            //printf("The cup was moved!\n");
        }
    #endif
}
