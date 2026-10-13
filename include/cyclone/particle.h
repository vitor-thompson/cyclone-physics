#pragma once

#include <cyclone/core.h>

namespace cyclone {
    /**
     * @brief Representa a unidade básica de simulação física no motor Cyclone.
     * @details Uma partícula possui posição, velocidade, aceleração, amortecimento
     * e massa, regida pelas leis de Newton.
     */
    class Particle
    {
    public:
        /** Armazena a posição linear da partícula no espaço do mundo. */
        Vector3 position;

        /** Armazena a velocidade linear da partícula no espaço do mundo. */
        Vector3 velocity;

        /** Armazena a aceleração da partícula (geralmente usada para gravidade). */
        Vector3 acceleration;

        /**
         * @brief Coeficiente de amortecimento (damping) para a movimento linear.
         * @details Usado para remover a energia gerada por instabilidades numéricas
         * do integrador e simular uma leve resistência do ar/fricção.
         * Valores típicos ficam entre 0.990 e 0.999.
         */
        real damping;
    };
}
