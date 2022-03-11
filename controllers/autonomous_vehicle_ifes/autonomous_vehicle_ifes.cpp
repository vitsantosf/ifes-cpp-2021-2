/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Autonoumous vehicle controller example
 */


#include <cmath>
#include <string>

#include "Trajetoria.hpp"
#include "Simulador.hpp"
#include "ContornarObstaculo.hpp"
#include "SeguirFaixa.hpp"

const std::string nome_do_arquivo_de_pontos ("caminho.csv");
  
Trajetoria trajetoria;

void print_help() {
  std::cout << "You can drive this car!\n";
  std::cout << "Select the 3D window and then use the cursor keys to:\n";
  std::cout << "[LEFT]/[RIGHT] - steer\n";
  std::cout << "[UP]/[DOWN] - accelerate/slow down\n";
  std::cout << "[L]/[S] - Load/Save trajectory\n";
}

void check_keyboard(int key){
  switch (key) {
    case 'L':
        trajetoria.load(nome_do_arquivo_de_pontos);
        break;
    case 'S':
        trajetoria.save(nome_do_arquivo_de_pontos);
        break;
  }
}

int main(int argc, char **argv) {
  int i (0);
  Simulador simulador;
  ContornarObstaculo contornar_obstaculo(simulador.getSickWidth(), simulador.getSickFOV());
  SeguirFaixa seguir_faixa_amarela(simulador.getCameraWidth(), simulador.getCameraHeight(), simulador.getCameraFOV());

  print_help();

  // main loop
  while (simulador.run()) {
    // get user inputs
    check_keyboard(simulador.check_keyboard());
    
    if (simulador.refresh(i)) {
      // read sensors
      const unsigned char *camera_image = simulador.getCameraImage();
      const float *sick_data = simulador.getSickData();

      if (simulador.autodriveEnabled()) {

        seguir_faixa_amarela.process_camera_image(camera_image);

        contornar_obstaculo.process_sick_data(sick_data);

        // avoid obstacles and follow yellow line
        if (contornar_obstaculo.obstacle_detected()) {

          simulador.setBrakeIntensity(0.0);

          double current_steering = simulador.getSteeringAngle();

          // compute the steering angle required to avoid the obstacle
          double obstacle_steering = contornar_obstaculo.compute_steer_angle(current_steering);
 
          // if we see the line we determine the best steering angle to both
          // avoid obstacle and follow the line
          double target_steering = seguir_faixa_amarela.compute_steer_angle(obstacle_steering, current_steering);

          // apply the computed required angle
          simulador.set_steering_angle(target_steering);
        } 
        else if (seguir_faixa_amarela.yellowLineDetected()) {
          // no obstacle has been detected, simply follow the line
          simulador.setBrakeIntensity(0.0);
          simulador.set_steering_angle(seguir_faixa_amarela.followYellowLine());
        } 
        else {
          // no obstacle has been detected but we lost the line => we brake and
          // hope to find the line again
          simulador.setBrakeIntensity(0.4);
        }
      }

      simulador.update();

      if ( trajetoria.distanceGreaterThan1m( simulador.getX(), simulador.getZ() ) )
        trajetoria.addPoint( simulador.getX(), simulador.getZ() );
    }

    i++;
  }
  return 0;
}