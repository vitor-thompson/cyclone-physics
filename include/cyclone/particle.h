#pragma once

#include <cyclone/core.h>
#include <cmath> // Necessário para powf, caso use o damping baseado no tempo

namespace cyclone {
    /**
     * @brief Representa a unidade básica de simulação física no motor Cyclone.
     */
    class Particle
    {
    public:
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        real damping;

    protected:
        real inverseMass;

    public:
        // --- CONSTRUTOR / MÉTODOS DE MASSA (já criados anteriormente) ---
        
        void setMass(const real mass)
        {
            if (mass <= 0.0f) {
                inverseMass = 0.0f;
                return;
            }
            inverseMass = ((real)1.0) / mass;
        }

        real getMass() const
        {
            if (inverseMass == 0.0f) {
                return (real)-1.0f;
            }
            return ((real)1.0) / inverseMass;
        }

        void setInverseMass(const real inverseMass)
        {
            this->inverseMass = inverseMass;
        }

        real getInverseMass() const
        {
            return inverseMass;
        }

        bool hasFiniteMass() const
        {
            return inverseMass >= 0.0f;
        }

        // --- NOVO MÉTODO: O INTEGRADOR ---

        /**
         * @brief Atualiza a posição e a velocidade da partícula com base no tempo decorrido.
         * @param duration O intervalo de tempo (dt) desde o último quadro.
         */
        void integrate(real duration)
        {
            // Se a massa inversa for 0, o objeto tem massa infinita (imutável). Não integramos.
            if (inverseMass <= 0.0f) return;

            // 1. Atualiza a posição linear: p = p + v * t
            // Usamos o nosso método otimizado addScaledVector para evitar alocações na stack.
            position.addScaledVector(velocity, duration);

            // 2. Trabalha a aceleração resultante gerada pelas forças
            // (Neste estágio inicial, a aceleração principal é a gravidade que definimos)
            Vector3 resultingAcceleration = acceleration;

            // 3. Atualiza a velocidade linear: v = v * (damping ^ t) + a * t
            // Aplicamos o amortecimento considerando o tempo decorrido (powf)
            velocity.addScaledVector(resultingAcceleration, duration);

            // Aplica o damping (amortecimento de arrasto)
            velocity *= powf(damping, duration);
        }
    };
}
