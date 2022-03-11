#pragma once

const float UNKNOWN(99999.99);
// Line following PID
const float KP(0.25); 
const float KI(0.006);
const float KD(2.0);
// Size of the yellow line angle filter
const int FILTER_SIZE(3);

class SeguirFaixa {
bool PID_need_reset;
double yellow_line_angle;
int camera_width;
int camera_height;
float camera_fov;
public:
    SeguirFaixa(int width, int height, float fov) : PID_need_reset(false), yellow_line_angle(0.0),
    camera_width(width), camera_height(height), camera_fov(fov) {}

    // compute rgb difference
    int color_diff(const unsigned char a[3], const unsigned char b[3]) {
        int i, diff = 0;
        for (i = 0; i < 3; i++) {
            int d = a[i] - b[i];
            diff += d > 0 ? d : -d;
        }
        return diff;
    }

    // returns approximate angle of yellow road line
    // or UNKNOWN if no pixel of yellow line visible
    void process_camera_image(const unsigned char *image) {
        int num_pixels =
            camera_height * camera_width;            // number of pixels in the image
        const unsigned char REF[3] = {95, 187, 203}; // road yellow (BGR format)
        int sumx = 0;                                // summed x position of pixels
        int pixel_count = 0;                         // yellow pixels count

        const unsigned char *pixel = image;
        int x;
        for (x = 0; x < num_pixels; x++, pixel += 4) {
            if (color_diff(pixel, REF) < 30) {
            sumx += x % camera_width;
            pixel_count++; // count yellow pixels
            }
        }

        double line_angle;
        // if no pixels was detected...
        if (pixel_count == 0)
            line_angle = UNKNOWN;
        else
            line_angle = ((double)sumx / pixel_count / camera_width - 0.5) * camera_fov;

        yellow_line_angle = filter_angle(line_angle);
    }

    // filter angle of the yellow line (simple average)
    double filter_angle(double new_value) {
        static bool first_call = true;
        static double old_value[FILTER_SIZE];
        int i;

        if (first_call || new_value == UNKNOWN) { // reset all the old values to 0.0
            first_call = false;
            for (i = 0; i < FILTER_SIZE; ++i)
            old_value[i] = 0.0;
        } else { // shift old values
            for (i = 0; i < FILTER_SIZE - 1; ++i)
            old_value[i] = old_value[i + 1];
        }

        if (new_value == UNKNOWN)
            return UNKNOWN;
        else {
            old_value[FILTER_SIZE - 1] = new_value;
            double sum = 0.0;
            for (i = 0; i < FILTER_SIZE; ++i)
            sum += old_value[i];
            return (double)sum / FILTER_SIZE;
        }
    }

    bool yellowLineDetected() {
        if (yellow_line_angle != UNKNOWN)
            return true;
        else {
            PID_need_reset = true;
            return false;
        }
    }

    double followYellowLine() {
        return applyPID(yellow_line_angle);
    }

    double compute_steer_angle(double obstacle_steering, double steer) {
        if (yellow_line_angle != UNKNOWN) {
            const double line_following_steering = applyPID(yellow_line_angle);
            if (obstacle_steering > 0 && line_following_steering > 0)
                steer = obstacle_steering > line_following_steering
                            ? obstacle_steering
                            : line_following_steering;
            else if (obstacle_steering < 0 && line_following_steering < 0)
                steer = obstacle_steering < line_following_steering
                            ? obstacle_steering
                            : line_following_steering;
        } 
        else
            PID_need_reset = true;
        return steer;
    }

    double applyPID(double yellow_line_angle) {
        static double oldValue(0.0);
        static double integral(0.0);

        if (PID_need_reset) {
            oldValue = yellow_line_angle;
            integral = 0.0;
            PID_need_reset = false;
        }

        // anti-windup mechanism
        if (std::signbit(yellow_line_angle) != std::signbit(oldValue))
            integral = 0.0;

        double diff = yellow_line_angle - oldValue;

        // limit integral
        if (integral < 30 && integral > -30)
            integral += yellow_line_angle;

        oldValue = yellow_line_angle;

        return KP * yellow_line_angle + KI * integral + KD * diff;
    }

};