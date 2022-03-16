#pragma once
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Device.hpp>
#include <webots/Display.hpp>
#include <webots/GPS.hpp>
#include <webots/Lidar.hpp>
#include <webots/Robot.hpp>
#include <webots/vehicle/Driver.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>

const int TIME_STEP(24);
// to be used as array indices
enum { X, Y, Z };

class Simulador {
private:
    webots::Keyboard teclado;
    webots::Driver motorista;

    // enable various 'features'
    bool enable_collision_avoidance;
    bool enable_display;
    bool has_gps;
    bool has_camera;

    // camera
    webots::Camera *camera;
    int camera_width;
    int camera_height;
    float camera_fov;

    // SICK laser
    webots::Lidar *sick_laser_sensor;
    int sick_width;
    float sick_range;
    float sick_fov;

    // display
    webots::Display *display;

    webots::ImageRef *speedometer_image;

    // GPS
    webots::GPS *gps;
    float gps_coords[3];
    float gps_speed;
    float i_coords[2];

    // misc variables
    float speed;
    float steering_angle;
    int manual_steering;
    bool autodrive;
public:
    Simulador() : enable_collision_avoidance(false), enable_display(false), has_gps(false), has_camera(false),
    camera_width(-1), camera_height(-1), camera_fov(-1.0), sick_width(-1), sick_range(-1.0), sick_fov(-1.0),
    gps_speed(0.0), speed(0.0), steering_angle(0.0), manual_steering(0), autodrive(false) 
    {
        i_coords[0] = i_coords[1] = 0.0;
        gps_coords[0] = gps_coords[1] = gps_coords[2] = 0.0;

        // check if there is a SICK and a display
        for (int j = 0; j < motorista.getNumberOfDevices(); ++j) {
            webots::Device *device = motorista.getDeviceByIndex(j);
            std::string name = device->getName();
            if (name == "Sick LMS 291")
                enable_collision_avoidance = true;
            else if (name == "display")
                enable_display = true;
            else if (name == "gps")
                has_gps = true;
            else if (name == "camera")
                has_camera = true;
        }

        // camera device
        if (has_camera) {
            camera = motorista.getCamera("camera");
            camera->enable(TIME_STEP);
            camera_width = camera->getWidth();
            camera_height = camera->getHeight();
            camera_fov = camera->getFov();
        }

        // SICK sensor
        if (enable_collision_avoidance) {
            sick_laser_sensor = motorista.getLidar("Sick LMS 291");
            sick_laser_sensor->enable(TIME_STEP);
            sick_width = sick_laser_sensor->getHorizontalResolution();
            sick_range = sick_laser_sensor->getMaxRange();
            sick_fov = sick_laser_sensor->getFov();
        }

        // initialize gps
        if (has_gps) {
            gps = motorista.getGPS("gps");
            gps->enable(TIME_STEP);
        }

        // initialize display (speedometer)
        if (enable_display) {
            display = motorista.getDisplay("display");
            speedometer_image = display->imageLoad("speedometer.png");
        }

        motorista.setHazardFlashers(true);
        motorista.setDippedBeams(true);
        motorista.setAntifogLights(true);
        motorista.setWiperMode(webots::Driver::WiperMode::SLOW);

        // allow to switch to manual control
        teclado.enable(TIME_STEP);
    }

    bool cameraEnabled() {
        return has_camera;
    }
    
    bool autodriveEnabled() {
        return autodrive;
    }

    bool collisionAvoidanceEnabled() {
        return enable_collision_avoidance;
    }

    auto getCameraImage() {
        const unsigned char *camera_image = NULL;
        if (has_camera)
            camera_image = camera->getImage();
        return camera_image;
    }

    auto getSickData() {
        const float *sick_data = NULL;
        if (enable_collision_avoidance)
            sick_data = sick_laser_sensor->getRangeImage();
        return sick_data;
    }

    float getSickWidth(){
        return sick_width;
    }

    float getSickRange(){
        return sick_range;
    }

    float getSickFOV(){
        return sick_fov;
    }

    int getCameraWidth(){
        return camera_width;
    }

    int getCameraHeight(){
        return camera_height;
    }

    float getCameraFOV(){
        return camera_fov;
    }

    float getSteeringAngle() {
        return steering_angle;
    }

    float getX() {
        return gps_coords[X];
    }

    float getY() {
        return gps_coords[Y];
    }

    float getZ() {
        return gps_coords[Z];
    }

    bool run() {
        return motorista.step() != -1;
    }

    bool refresh(int i) {
        // updates sensors only every TIME_STEP milliseconds
        return i % static_cast<int>(TIME_STEP / motorista.getBasicTimeStep()) == 0;
    }

    void update() {
      if (has_gps) 
        compute_gps_speed();
      
      if (enable_display)
        update_display();      
    }

    // set target speed
    void set_speed(double kmh) {
        // max speed
        if (kmh > 250.0)
            kmh = 250.0;

        speed = kmh;

        std::cout << "setting speed to " << kmh << " km/h\n";

        motorista.setCruisingSpeed(kmh);
    }

    void set_autodrive(bool onoff) {
        if (autodrive == onoff)
            return;
        autodrive = onoff;
        if (autodrive) {
            if (has_camera) {
                std::cout << "switching to auto-drive...\n";
                set_speed(100.0); // km/h
            } else
            std::cout << "impossible to switch auto-drive on without camera...\n";
        } else {
            std::cout << "switching to manual drive...\n";
            std::cout << "hit [A] to return to auto-drive.\n";
        }
    }

    // positive: turn right, negative: turn left
    void set_steering_angle(double wheel_angle) {
        // limit the difference with previous steering_angle
        if (wheel_angle - steering_angle > 0.05)
            wheel_angle = steering_angle + 0.05;
        if (wheel_angle - steering_angle < -0.01)
            wheel_angle = steering_angle - 0.01;
        steering_angle = wheel_angle;
        // limit range of the steering angle
        if (wheel_angle > 0.5)
            wheel_angle = 0.5;
        else if (wheel_angle < -0.1)
            wheel_angle = -0.1;
        motorista.setSteeringAngle(wheel_angle);
    }

    void change_manual_steer_angle(int inc) {
        set_autodrive(false);

        double new_manual_steering = manual_steering + inc;
        if (new_manual_steering <= 25.0 && new_manual_steering >= -25.0) {
            manual_steering = new_manual_steering;
            set_steering_angle(manual_steering * 0.02);
        }

        if (manual_steering == 0)
            std::cout << "going straight\n";
        else
            std::cout << "turning " << steering_angle << " rad " << (steering_angle < 0 ? "left" : "right") << "\n";
    }

    int check_keyboard() {
        int key = teclado.getKey();
        switch (key) {
        case webots::Keyboard::UP:
            set_speed(speed + 5.0);
            break;
        case webots::Keyboard::DOWN:
            set_speed(speed - 5.0);
            break;
        case webots::Keyboard::RIGHT:
            change_manual_steer_angle(+1);
            break;
        case webots::Keyboard::LEFT:
            change_manual_steer_angle(-1);
            break;
        case 'A':
            set_autodrive(true);
            break;
        }
        return key;
    }

    void update_display() {
        const double NEEDLE_LENGTH = 50.0;

        // display background

        display->imagePaste(speedometer_image, 0, 0, false);

        // draw speedometer needle
        double current_speed = motorista.getCurrentSpeed();
        if (std::isnan(current_speed))
            current_speed = 0.0;
        double alpha = current_speed / 260.0 * 3.72 - 0.27;
        int x = -NEEDLE_LENGTH * cos(alpha);
        int y = -NEEDLE_LENGTH * sin(alpha);

        display->drawLine(100, 95, 100 + x, 95 + y);

        // draw text
        std::stringstream txt_coords;
        txt_coords << std::fixed << std::setprecision(1)
                    << "GPS coords: " << gps_coords[X] << " " << gps_coords[Z];

        display->drawText(txt_coords.str(), 10, 130);

        std::stringstream txt_speed;
        txt_speed << std::fixed << std::setprecision(1) << "GPS speed: " << gps_speed;

        display->drawText(txt_speed.str(), 10, 140);
    }

    void compute_gps_speed() {
        const double *coords = gps->getValues();

        const double speed_ms = gps->getSpeed();

        // store into global variables
        gps_speed = speed_ms * 3.6; // convert from m/s to km/h

        // memcpy(gps_coords, coords, sizeof(gps_coords)); ----->
        for (int i = 0; i < 3; i++)
            gps_coords[i] = coords[i];
    }

    void setBrakeIntensity(double value) {
        motorista.setBrakeIntensity(value);
    }
};