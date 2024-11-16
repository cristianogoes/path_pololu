#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <pololu/orangutan.h>
#include <pololu/3pi.h>
#include "follow-segment.h"
#include "turn.h"


// Declaring the grid and its variations ( Visited and Hueristic and dynamic path )
#if 0
int grid[7][8]= {{1,1,1,1,1,1,1,1},
                {1,0,1,0,0,0,0,1},
                {1,0,1,0,0,0,0,1},
                {1,0,1,0,0,0,0,1},
                {1,0,1,0,0,1,0,1},
                {1,0,0,0,0,1,0,1},
                {1,1,1,1,1,1,1,1},};
#endif
int grid[7][8]= {{1,1,1,1,1,1,1,1},
                {1,0,0,1,0,0,0,1},
                {1,0,0,0,0,0,0,1},
                {1,0,0,0,0,1,1,1},
                {1,0,0,1,0,0,0,1},
                {1,0,0,1,0,0,0,1},
                {1,1,1,1,1,1,1,1},};

int visited[7][8]={{1,1,1,1,1,1,1,1},
                   {1,0,0,0,0,0,0,1},
                   {1,0,0,0,0,0,0,1},
                   {1,0,0,0,0,0,0,1},
                   {1,0,0,0,0,0,0,1},
                   {1,0,0,0,0,0,0,1},
                   {1,1,1,1,1,1,1,1},};
                  
int hueristic[7][8]={{0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},};


// Declaring start goal and various other variables

//int startx=1;
//int starty=1;
//int goalx=5;
//int goaly=6;

int ROW = 7;
int COL = 8;
int WALL = 50;
int VIST = 70;

int startx=1;
int starty=1;
int goalx=1;
int goaly=5;

int locx;
int locy;

int index_2;
int costarray[4] ;

double f; // nao deve tá usando
int g = 1;
int maxval = 100;
int obst = 0;
int new_route = 0;

// Now read the sensors and check the intersection type.
unsigned int sensors[5];

char path[100] = "";
unsigned char path_length = 0; // the length of the path

char dir_prev = 'X';
char dir = 'X';
char dir_robot = 'X';

// Displays the current path on the LCD, using two rows if necessary.
void display_path()
{
	// Set the last character of the path to a 0 so that the print()
	// function can find the end of the string.  This is how strings
	// are normally terminated in C.
	path[path_length] = 0;

	clear();
	print(path);

	if(path_length > 8)
	{
		lcd_goto_xy(0,1);
		print(path+8);
	}
}

// Heuristic function (Manhattan distance)
int heuristic_man(ax, ay, bx, by)
{
    return abs(ax - bx) + abs(ay - by);
}

void path_planning(){

    init_ultra();
    lcd_init_printf();	// required if we want to use printf()

	clear();			// clear the LCD
	delay_ms(200);		// wait for 200 ms

	printf("hello");	// print "hello" on the first line of the LCD
	delay_ms(200);		// wait for 200 ms
	printf("\nworld");	// print "world" on the second line (because of '\n')
	
    delay_ms(2000);		// wait for 2 seconds


	clear();

  
    //calculating hueristic 
    if((goaly < COL && goalx < ROW  && goalx > -1 && goaly > -1 ) ){
        for(int L = 0; L < ROW; L++){
            for(int C = 0; C < COL; C++){
                hueristic[L][C] = heuristic_man(L, C, goalx, goaly);
                // Checa se é parede é aumenta distancia
                if (grid[L][C] == 1){
                    hueristic[L][C] = hueristic[L][C] + WALL;
                }
            }
        }
    }
    
    //------------------------------------------------
    // Uncomment Region to view the hueristic grid
    #if 0
    for(int L = 0; L <7; L++){
        for(int C = 0; C <8; C++){
            printf("%d ", hueristic[L][C]);
        }
        printf("\n");
    }
    #endif
    //------------------------------------------------    
 
    locx = startx;
    locy = starty;

    while(true){
        clear();
        // 0 for up.    [locx-1] [locy]
        // 1 for right. [locx]   [locy+1]
        // 2 for down.  [locx+1] [locy]
        // 3 for left.  [locx]   [locy-1]
        // Overhere we assign our costs and prioritize our robots movement.
        // Sé já visitou aumenta o valor.
        
        if (visited[locx-1][locy] == 2){
            costarray[0] = g + hueristic[locx-1][locy] + VIST;
        }else{
            costarray[0] = g + hueristic[locx-1][locy];
        }

        if (visited[locx][locy+1] == 2){
            costarray[1] = g + hueristic[locx][locy+1] + VIST;
        }else{
            costarray[1] = g + hueristic[locx][locy+1];
        }

        if (visited[locx+1][locy] == 2){
            costarray[2] = g + hueristic[locx+1][locy] + VIST;
        }else{
            costarray[2] = g + hueristic[locx+1][locy]; 
        }

        if (visited[locx][locy-1] == 2){
            costarray[3] = g + hueristic[locx][locy-1] + VIST;
        }else{
            costarray[3] = g + hueristic[locx][locy-1];
        }

        #if 0
        printf("======\n");
        printf("(%d,%d)=%d %d\n", (locx-1), locy, hueristic[locx-1][locy], costarray[0]);
        printf("(%d,%d)=%d %d\n", (locx), (locy+1), hueristic[locx][locy+1], costarray[1]);
        printf("(%d,%d)=%d %d\n", (locx+1), locy, hueristic[locx+1][locy], costarray[2]);
        printf("(%d,%d)=%d %d\n", (locx), (locy-1), hueristic[locx][locy-1], costarray[3]);
        printf("(%d,%d)=%d\n", (locx), (locy), hueristic[locx][locy]);
        #endif
        
        //----------------------------------------------------------------------------------------------------------------------
        //Finding node with least cost
        maxval = 100;
        for(int i=0;i<4;i++){
            if(costarray[i] < maxval){
                index_2 = i;
                maxval = costarray[i];
                //printf("find node: ind:%d, max:%d\n", index_2, maxval);
                //printf("cost[%d]=%d\n", i, costarray[i]);
            }
        }
        //----------------------------
        visited[locx][locy]=2;
        //move your position to the new location
        if(index_2 == 0){
            // UP
            locx = locx-1;
            printf(" U ");
            dir = 'U';
        }else if(index_2 == 1){
            // Rigth
            locy = locy+1;
            printf(" R ");
            dir = 'R';
        }else if(index_2 == 2){
            // Down
            locx = locx+1;
            printf(" D ");
            dir = 'D';
        }else if(index_2 == 3){
            // Left
            locy = locy-1;
            printf(" L ");
            dir = 'L';
        }
        
        printf("(%d,%d)", locx, locy);
        //delay_ms(2000);
        
        switch(dir_prev)
        {
        case 'L':
            if(dir == 'D'){
                printf(" r L");
                dir_robot = 'L';
            }else if(dir == 'R'){
                printf(">not L");
            }else if(dir == 'L'){
                printf(" r S");
                dir_robot = 'S';
            }else if(dir == 'U'){
                printf(" r R");
                dir_robot = 'R';
            }
            break;
        case 'R':
            if(dir == 'D'){
                printf(" r R");
                dir_robot = 'R';
            }else if(dir == 'R'){
                printf(" r S");
                dir_robot = 'S';
            }else if(dir == 'L'){
                printf(">not R");
            }else if(dir == 'U'){
                printf(" r L");
                dir_robot = 'L';
            }
            break;
        case 'U':
            if(dir == 'D'){
                printf(">not U");
            }else if(dir == 'R'){
                printf(" r R");
                dir_robot = 'R';
            }else if(dir == 'L'){
                printf(" r L");
                dir_robot = 'L';
            }else if(dir == 'U'){
                printf(" r S");
                dir_robot = 'S'; 
            }
            break;
        case 'D':
            if(dir == 'D'){
                printf(" r S ");
                dir_robot = 'S';
            }else if(dir == 'R'){
                printf(" r L");
                dir_robot = 'L';
            }else if(dir == 'L'){
                printf(" r R");
                dir_robot = 'R';
            }else if(dir == 'U'){
                printf(">not D");
            }
            break;
        case 'X':
            if(dir == 'D'){
                printf("\nVERIFICAR TURN U ");
            }else if(dir == 'R'){
                printf(" r R");
                dir_robot = 'R';
            }else if(dir == 'L'){
                printf(" r L");
                dir_robot = 'L';
            }else if(dir == 'U'){
                printf(" r S");
                dir_robot = 'S';
            }
            break;
        }
        
        turn(dir_robot);
        set_motors(0,0);
        //delay_ms(2000);

        // Store the intersection in the path variable.
		path[path_length] = dir_robot;
		path_length ++;

        while(true){
            obst = follow_segment();
            if (obst){
                set_motors(0,0);
                clear();
                printf("Obstacu");
                turn('B');
                set_motors(0,0);
                delay_ms(500);
                clear();
                new_route = 1;
            }else{
                read_line(sensors,IR_EMITTERS_ON);
                if((sensors[0] > 200) && (sensors[1] > 200) && (sensors[2] > 200) && (sensors[3] > 200) && (sensors[4] > 200))
                {
                    play(">>a32");
                    set_motors(20,20);
                    delay_ms(100);
                    //set_motors(60,60);
                    //delay_ms(300);
                    set_motors(0,0);
                    play(">>a32");
                    break;
                }
            }
        }
        
        if(new_route){
        
            clear();
            printf("OB(%d,%d)",locx, locy);

            grid[locx][locy]=1;
            visited[locx][locy]=2;
            if(index_2 == 0){
                printf("\nND(%d,%d)", locx+1, locy);
                locx = locx+1;
                dir =  'D';
            }else if(index_2 == 1){
                printf("\nNL(%d,%d)", locx, locy-1);
                locy = locy-1;
                dir = 'L';
            }else if(index_2 == 2){
                printf("\nNU(%d,%d)", locx-1, locy);
                locx = locx-1;
                dir = 'U';
            }else if(index_2 == 3){
                printf("\nNR(%d,%d)", locx, locy+1);
                locy = locy+1;
                dir = 'R';
            }
            //break;
            new_route = 0;
        }
        
        dir_prev = dir;
        //delay_ms(2000);
        
        if(locx == goalx && locy == goaly){
            visited[locx][locy] = 2;
            clear();
            printf("found\n");
            set_motors(0,0);
            // Display the path on the LCD.
		    //display_path();
            break;
        }

        if((locy > COL && locx > ROW  && locx < -1 && locy < -1 ) ){
            clear();
            printf("No target\n");
            break;
        }
    }

    //------------------------------------------------
    // Uncomment Region to view the visited grid
    // 1 for Wall
    // 2 for visited
    // 0 for no visited

    #if 0
    for(int L = 0; L <7; L++){
        for(int C = 0; C <8; C++){
            if (visited[L][C]==0){
                printf("%d ", grid[L][C]);
            }else{
                printf("%d ", visited[L][C]);
            }
        }
        printf("\n");
    }
    #endif
    //------------------------------------------------
}