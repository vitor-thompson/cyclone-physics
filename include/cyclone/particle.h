#pragma once

#include <cyclone/core.h>

namespace cyclone {
    class Particle
    {
    public:
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        
        /** Acumulador de forças aplicadas à partícula no quadro atual. */
        Vector3 forceAccum;

        real damping;

    protected:
        real inverseMass;

    public:
        // Construtor padrão
        Particle()
            : position(0, 0, 0), velocity(0, 0, 0), acceleration(0, 0, 0),
              forceAccum(0, 0, 0), damping(0.99f), inverseMass(1.0f) {}

        // Setters básicos
        void setPosition(real x, real y, real z) { position = Vector3(x, y, z); }
        Vector3 getPosition() const { return position; }

        void setVelocity(real x, real y, real z) { velocity = Vector3(x, y, z); }
        Vector3 getVelocity() const { return velocity; }

        void setAcceleration(real x, real y, real z) { acceleration = Vector3(x, y, z); }
        void setDamping(real d) { damping = d; }

        // Métodos de Massa
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

        void setInverseMass(const real inverseMass) { this->inverseMass = inverseMass; }
        real getInverseMass() const { return inverseMass; }

        // --- NOVOS MÉTODOS DE FORÇA (Capítulo 5) ---

        /**
         * @brief Adiciona uma força à partícula para ser aplicada apenas na próxima iteração.
         * @param force A força (vetor) a ser aplicada.
         */
        void addForce(const Vector3 &force);

        /**
         * @brief Limpa o acumulador de forças. Chamado automaticamente após a integração.
         */
        void clearAccumulator();

        // Integrador Físico
        void integrate(real duration);
    };
}
