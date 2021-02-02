#ifndef __COST_FUN__
#define __COST_FUN__
#include <vector>
#include <cmath>
#include <map>
#include <iostream>
#include <string>
#define cal_d(lane) (2+(double)(lane)*4)
//(1/30, START HERE)

using std::vector;

// vector<bool> mode_info


void choose_mode(vector<double> sensor_fusion, const int cur_lane, vector<double> my_car, const int prev_size, std::map<std::string, double>& info, double end_path_s){
    // my car vector composition
    double car_x = my_car[0];
    double car_y = my_car[1];
    double car_s = my_car[2];
    double car_d = my_car[3];
    double car_yaw = my_car[4];
    double car_speed = my_car[5];

    double cur_d = cal_d(cur_lane);
    double d = sensor_fusion[6];
    double vx = sensor_fusion[3];
    double vy = sensor_fusion[4];
    double check_speed = sqrt(vx * vx + vy * vy);
    double check_car_s = sensor_fusion[5];

    if (prev_size > 0 ){
        car_s = end_path_s;
    }
    check_car_s += ((double)prev_size * .02 * check_speed);
    double cost_val;
                    
    if (abs(check_car_s - car_s)<40 ){
        if ( ( d < ( cur_d+2)) &&  ( d> (cur_d-2)) ){ // keep_lane
            if (check_car_s > car_s)
                cost_val = 0.7/(check_car_s-car_s);
            else{
                cost_val = 0;
            }
            if (info.find("keep") == info.end())
                info["keep"] = cost_val;
            else{
                if (info["keep"] < cost_val)
                    info["keep"] = cost_val;
            }
        }
        else if( (d > ( cal_d(cur_lane-1) -2 ) ) && ( d < (cur_d -2) ) && (d>0) ){// left
            if (check_car_s > car_s)
                cost_val = 1/(check_car_s-car_s) ;
            else
                cost_val = 0.5 / (car_s - check_car_s);

            if (info.find("left") == info.end())
                info["left"] = cost_val;
            else{
                if ( info["left"] < cost_val )
                    info["left"] = cost_val;
            }
        }
        else if ( (d < (cal_d(cur_lane+1) +2) )&&(d > (cur_d +2) ) && ( d < (cal_d(2)+2) ) ){// right
            if (check_car_s > car_s)
                cost_val = 1/(check_car_s-car_s);
            else
                cost_val = 0.5 / (car_s - check_car_s) ;

            if (info.find("right") == info.end())
                info["right"] = cost_val;
            else{
                if ( info["right"] < cost_val )
                    info["right"] = cost_val;
            }
        }
        
        if (info.find("left") == info.end()){
            if ( cur_lane > 0 )
                info["left"] = 0;
        }        
        if(info.find("right") == info.end()){
            if ( cur_lane < 2 )
                info["right"] = 0;
        }
    }
}

void check_empty(std::map<std::string, double>& info, int cur_lane){

    if (info.find("left") == info.end()){ //left
        if(cur_lane> 0)
            info["left"] = 0;
    }
    else if(info.find("right") == info.end()){
        if (cur_lane<2) //right
            info["right"] = 0;
    }
}


void update_current_lane(double car_d, int &lane){
    if ((car_d > (cal_d(0) -1.8))&&(car_d < (cal_d(0) + 1.8)))
        lane = 0;
    else if ((car_d > (cal_d(1) -1.8))&&(car_d < (cal_d(1) + 1.8)))
        lane = 1;
    else if ((car_d > (cal_d(2) -1.8))&&(car_d < (cal_d(2) + 1.8)))
        lane = 2;
}




#endif