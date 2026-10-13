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

        /**
         * @brief Integrador físico da partícula (declarado na classe).
         */
        void integrate(real duration);
    };
}
