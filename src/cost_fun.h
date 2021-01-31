#ifndef __COST_FUN__
#define __COST_FUN__
#include <vector>
#include <cmath>
#include <map>
#include <iostream>

//(1/30, START HERE)

using std::vector;

// vector<bool> mode_info


void choose_mode(vector<double> sensor_fusion, const int cur_lane, vector<double> my_car, const int prev_size, std::map<int, double>& info){
    // my car vector composition
    double car_x = my_car[0];
    double car_y = my_car[1];
    double car_s = my_car[2];
    double car_d = my_car[3];
    double car_yaw = my_car[4];
    double car_speed = my_car[5];

    int cur_d = 2 + 4 * cur_lane;
    double d = sensor_fusion[6];
    double vx = sensor_fusion[3];
    double vy = sensor_fusion[4];
    double check_speed = sqrt(vx * vx + vy * vy);
    double check_car_s = sensor_fusion[5];

    check_car_s += ((double)prev_size * .02 * check_speed);
    double cost_val;

    if( (d>0) && (d < cur_d -2 ) && (cur_lane>0))// left
    {
         if ((check_car_s > car_s ) && ((check_car_s - car_s) < 30 ))  {
           cost_val = 1/(check_car_s-car_s);
           info[0] = cost_val;
           //std::cout << "left: " << info[0] << std::endl;
         }
    }

    if ( (d > cur_d +2) && ( d < (2+4*2+2)) && (cur_lane<2) ) // right, lane 3ea
    {
        if ((check_car_s > car_s ) && ((check_car_s - car_s) < 30 ))  {
            cost_val = 1/(check_car_s-car_s);
            info[1] = cost_val;
            //std::cout << "right: " << info[1] << std::endl;
         }
    }
    
    if ( ( d < ( cur_d+2)) &&  ( d> (cur_d-2)) ){ // keep_lane
        //if ((check_car_s > car_s ) && ((check_car_s - car_s) < 30 ))  {
            cost_val = 1/(check_car_s-car_s);
            info[2] = cost_val;
            //std::cout << "keep: " << info[2] << std::endl;
        // }
    }
}

void check_empty(std::map<int, double>& info, int cur_lane){

    if (info.find(0) == info.end()){ //left
        if(cur_lane> 0)
            info[0] = 0;
    }
    else if(info.find(1) == info.end()){
        if (cur_lane<2) //right
            info[1] = 0;
    }
    // (1/31) 왜 현재 lane이 제일 좌측이면 좌측으로 업데이트 하면 안되는데.. 뭔가 문제가 있네 ㅋㅋ 
}


#endif