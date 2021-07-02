// #include "strategy/strategy_main.h"

// int main(int argc, char** argv)
// {
// 	ros::init(argc, argv, "BBthrow");
// 	ros::NodeHandle nh;
// 	KidsizeStrategy KidsizeStrategy;
    
// 	ros::Rate loop_rate(30);

//     // Load->initparameterpath();
// 	while (nh.ok()) 
// 	{
// 		ros::spinOnce();
// 		KidsizeStrategy.strategymain();
// 		loop_rate.sleep();
// 	}
// 	return 0;
// }
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h> // for printf
#include "strategy/pso.h"

#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include "strategy/obstacle.h"
#include "strategy/particle.h"
#include<iostream>
using namespace std;
bool get_obs = false;
float obs_coordinate[2] = {0};
float free_limit[2] = {0};
//==============================================================
//                  BENCHMARK FUNCTIONS
//==============================================================

double pso_sphere(double *vec, int dim, void *params) {

    double sum = 0;
    int i;
    for (i=0; i<dim; i++)
        sum += pow(vec[i]-obs_coordinate[i], 2);  // pow = vec[i]^2

    return sum;
}



double pso_rosenbrock(double *vec, int dim, void *params) {

    double sum = 0;
    int i;
    for (i=0; i<dim-1; i++)
        sum += 100 * pow((vec[i+1] - pow(vec[i], 2)), 2) +	\
            pow((1 - vec[i]), 2);

    return sum;

}


double pso_griewank(double *vec, int dim, void *params) {

    double sum = 0.;
    double prod = 1.;
    int i;
    for (i=0; i<dim;i++) {
        sum += pow(vec[i], 2);
        prod *= cos(vec[i] / sqrt(i+1));
    }

    return sum / 4000 - prod + 1;

}

void Obstaclefreearea(const strategy::obstacle &msg)
{
    get_obs = true;
    ROS_INFO("qqq");
    // sleep(1);
    // ROS_INFO("%f", msg.x);
    // ROS_INFO("%f", msg.y);
    // ROS_INFO("%f", msg.width);
    // ROS_INFO("%f", msg.height);
    // ROS_INFO("I heard: [%f],[%f],[%f],[%f]", msg->data.at(0),msg->data.at(1),msg->data.at(2),msg->data.at(3));

	float obstacle_x = msg.x;
    float obstacle_y = msg.y;
    float obstacle_width = msg.width;
    float obstacle_height = msg.height;
    obs_coordinate[0] = msg.x;
    obs_coordinate[1] = msg.y;
    free_limit[0] = msg.width;
    free_limit[1] = msg.height;
    ROS_INFO("rrr");
    sleep(1);
	ROS_INFO("%f, %f, %f, %f",obstacle_x, obstacle_y, obstacle_width, obstacle_height);

}


//==============================================================

int main(int argc, char **argv) {
    ros::init(argc, argv, "BBthrow");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/Obstaclefreearea_Topic", 100, Obstaclefreearea);
    ros::spinOnce();
    ros::Rate loop_rate(30);

    while (nh.ok())
    {
        if(get_obs)
        {
            pso_settings_t *settings = NULL;
            pso_obj_fun_t obj_fun = NULL;

            // handle the default case (no argument given)
            if (obj_fun == NULL || settings == NULL) {
                obj_fun = pso_sphere;
                settings = pso_settings_new(2, &free_limit[0], &obs_coordinate[0]);
                // settings = pso_settings_new(2, -100, 100);
                printf("Optimizing function: sphere (dim=%d, swarm size=%d)\n", settings->dim, settings->size);
            }

            // set some general PSO settings
            settings->goal = 1e-5;
            // settings->size = 30;
            settings->nhood_strategy = PSO_NHOOD_RING;
            settings->nhood_size = 10;
            settings->w_strategy = PSO_W_LIN_DEC;

            // initialize GBEST solution
            pso_result_t solution;
            // allocate memory for the best position buffer
            solution.gbest = (double *)malloc(settings->dim * sizeof(double));

            printf("dim=%d, swarm size=%d)\n", settings->dim, settings->size);
            sleep(2);

            // run optimization algorithm
            pso_solve(obj_fun, NULL, &solution, settings, nh);

            // free the gbest buffer
            free(solution.gbest);

            // free the settings
            pso_settings_free(settings);
            get_obs = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
        
    
    }
    return 0;

}