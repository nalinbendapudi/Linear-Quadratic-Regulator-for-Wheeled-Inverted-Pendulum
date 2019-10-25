#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#include "../common/mb_motor.h"

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

// gains for LQR controller
float k[4] = {0.0,0.0,0.0,0.0};
float dk[4] = {0.0,0.0,0.0,0.0};
int dt = 0;


int mb_controller_init(){
    mb_controller_load_config();
    /* TODO initialize your controllers here*/


    return 0;
}


/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_load_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    /* TODO parse your config file here*/
    /* K gain from LQR*/
    fscanf(file, "%*[^\n]\n", NULL);
    fscanf(file, "%f %f %f %f %f %f %f %f %d", &k[0], &k[1], &k[2], &k[3], &dk[0], &dk[1], &dk[2], &dk[3], &dt);
    fclose(file);
    printf("Obtained gains: K1 = %f, K2 = %f, K3 = %f, K4 = %f\n", k[0],k[1],k[2],k[3]);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* 
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/
int control_time = 0;
char c = ' ';
int i = 0;
int mb_controller_update(mb_state_t* mb_state){
    /*TODO: Write your controller here*/

    // apply the controller to get duty
    // get the duty cycle for the PID controller
    // k1 = 5.0;
    // float duty = -k1 * mb_state->theta;

    // get the duty cycle for the LQR controller
    float duty = -k[0] * mb_state->theta - k[1] * mb_state->theta_d - k[2] * mb_state->phi - k[3] * mb_state->phi_d;
    if(control_time == dt*100){
        if(fabs(mb_state->theta)>M_PI/180 | fabs(mb_state->phi)>M_PI/180){
            printf("\nController stopped. Currently tuning K%d = %f.\nPress i to increase, d to decrease, r to repeat, n for next k, p for previous, s for stop\n", i+1, k[i]);
            mb_motor_disable();
            c = getchar();
            if( c == 'i') { // increase current k
                control_time = 0;
                k[i] += dk[i];
                printf("Increased to k%d = %f\n", i+1, k[i]);
            }
            else if(c == 'r') {
                control_time = 0;
                printf("Not Changed. k%d = %f\n",i+1, k[i]);
            }
            else if(c == 'd') {
                control_time = 0;
                k[i] -= dk[i];
                printf("Decreased to k%d = %f\n", i+1, k[i]);
            }
            else if(c == 'n') i++;
            else if(c == 'p') i--;
            else if(c == 's') printf("Final K values are %f %f %f %f\n", k[0], k[1], k[2], k[3]);
        }
    }
    else control_time++;
    
    mb_state->left_cmd = duty;
    mb_state->right_cmd = duty;
    
    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup(){
    return 0;
}