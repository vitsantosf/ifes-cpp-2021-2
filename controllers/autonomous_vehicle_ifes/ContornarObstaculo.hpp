#pragma once

class ContornarObstaculo {
    bool obstacle;
    double obstacle_dist;
    double obstacle_angle;
    int sick_width;
    float sick_fov;
public:
    ContornarObstaculo(int width, float fov) : obstacle(false), obstacle_dist(0), obstacle_angle(0), 
    sick_width(width), sick_fov(fov)
    {}

    // computes approximate angle of obstacle
    // or nothing if no obstacle was detected
    void process_sick_data(const float *sick_data) {
        const int HALF_AREA = 23; // check 20 degrees wide middle area
        int sumx = 0;
        int collision_count = 0;
        int x;
       
        obstacle_dist = 0.0;
        for (x = sick_width / 2 - HALF_AREA; x < sick_width / 2 + HALF_AREA; x++) {
            float range = sick_data[x];
            if (range < 10.0) {
            sumx += x;
            collision_count++;
            obstacle_dist += range;
            
            }
    
        }

        // if no obstacle was detected...
        if (collision_count == 0){
            obstacle = false;
            }
        else {
            obstacle = true;
            obstacle_dist = obstacle_dist / collision_count;
            obstacle_angle = ((double)sumx / collision_count / sick_width - 0.5) * sick_fov;
        }
    }

    bool obstacle_detected() {
        return obstacle;
    }

    double compute_steer_angle(double steering_angle) {
        // compute the steering angle required to avoid the obstacle
          double obstacle_steering = steering_angle;
          if (obstacle_angle > 0.0 && obstacle_angle < 0.4)
            obstacle_steering =
                steering_angle + (obstacle_angle - 0.25) / obstacle_dist;
          else if (obstacle_angle > -0.4)
            obstacle_steering =
                steering_angle + (obstacle_angle + 0.25) / obstacle_dist;
          return obstacle_steering;
    }
};