#pragma once

#include <cyclone/core.h>

namespace cyclone {
    /**
     * @brief Representa a unidade básica de simulação física no motor Cyclone.
     * @details Uma partícula possui posição, velocidade, aceleração, amortecimento
     * e massa inversa, regida pelas leis de Newton.
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
         * @brief Coeficiente de amortecimento (damping) para o movimento linear.
         * @details Usado para remover a energia gerada por instabilidades numéricas.
         */
        real damping;

    protected:
        /**
         * @brief Armazena o inverso da massa da partícula (1 / massa).
         * @details Armazenar a massa inversa simplifica a integração e permite
         * representar objetos com massa infinita (imutáveis) definindo o valor como 0.
         */
        real inverseMass;

    public:
        /**
         * @brief Define a massa da partícula diretamente.
         * @param mass A massa desejada (deve ser maior que zero).
         */
        void setMass(const real mass)
        {
            if (mass <= 0.0f) {
                // Evita divisão por zero ou massas inválidas
                inverseMass = 0.0f;
                return;
            }
            inverseMass = ((real)1.0) / mass;
        }

        /**
         * @brief Retorna a massa real da partícula.
         */
        real getMass() const
        {
            if (inverseMass == 0.0f) {
                // Retorna um valor representativo para massa infinita, se necessário
                return (real)-1.0f; // Ou tratado como infinito
            }
            return ((real)1.0) / inverseMass;
        }

        /**
         * @brief Define a massa inversa da partícula diretamente.
         * @param inverseMass A massa inversa (use 0.0 para objetos com massa infinita/imutáveis).
         */
        void setInverseMass(const real inverseMass)
        {
            this->inverseMass = inverseMass;
        }

        /**
         * @brief Retorna a massa inversa da partícula.
         */
        real getInverseMass() const
        {
            return inverseMass;
        }

        /**
         * @brief Verifica se a partícula possui massa infinita (estática / imóvel).
         */
        bool hasFiniteMass() const
        {
            return inverseMass >= 0.0f; // Na prática, checa se inverseMass é maior que zero
        }
    };
}
