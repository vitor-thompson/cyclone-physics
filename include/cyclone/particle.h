#pragma once

#include <cyclone/core.h>

namespace cyclone {
    class Particle
    {
    public:
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        Vector3 forceAccum;
        real damping;

    protected:
        real inverseMass;

    public:
        // --- Construtor padrão ---
        Particle()
            : position(0, 0, 0), velocity(0, 0, 0), acceleration(0, 0, 0),
              forceAccum(0, 0, 0), damping(1.0f), inverseMass(0.0f) {}

        // --- Setters de Posição, Velocidade e Aceleração ---
        void setPosition(const Vector3 &position) { this->position = position; }
        void setPosition(real x, real y, real z) { position.x = x; position.y = y; position.z = z; }
        
        Vector3 getPosition() const { return position; }

        void setVelocity(const Vector3 &velocity) { this->velocity = velocity; }
        void setVelocity(real x, real y, real z) { velocity.x = x; velocity.y = y; velocity.z = z; }

        void setAcceleration(const Vector3 &acceleration) { this->acceleration = acceleration; }
        void setAcceleration(real x, real y, real z) { acceleration.x = x; acceleration.y = y; acceleration.z = z; }

        void setDamping(real damping) { this->damping = damping; }
        real getDamping() const { return damping; }

        // --- Métodos de Massa ---
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

        // --- Limpeza e Acumulador de Forças ---
        void clearAccumulator()
        {
            forceAccum = Vector3(0, 0, 0);
        }

        // --- Integrador ---
        void integrate(real duration);
    };
}
