#pragma once

#include <cyclone/core.h>

namespace cyclone {
    /**
     * @brief Representa a unidade básica de simulação física no motor Cyclone.
     * @details Uma partícula possui posição, velocidade e aceleração em 3D,
     * mas não possui orientação ou propriedades de rotação.
     */
     class Particle
     {
     public:
         /** Armazena a posição linear da partícula no espaço do mundo. */
         Vector3 position;

         /** Armazena a velocidade linear da partícula no espaço do mundo. */
         Vector3 velocity;

         /** 
          * Armazena a aceleração da partícula. 
          * Usada principalmente para simular a gravidade ou outras forças constantes. 
          */
         Vector3 acceleration;
     };
}
