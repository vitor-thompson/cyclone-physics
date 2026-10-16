#pragma once

#include <cyclone/particle.h>
#include <cyclone/core.h>

namespace cyclone {
    /**
     * @brief Representa um contato entre duas partículas (ou uma partícula e o cenário).
     * Resolve interpenetrações e aplica impulsos de colisão/rebotamento.
     */
    class ParticleContact
    {
    public:
        /** Partículas envolvidas no contato. particle[1] pode ser nullptr para colisões com o cenário. */
        Particle* particle[2];

        /** Coeficiente de restituição (elasticidade) do contato. */
        real restitution;

        /** Direção normal do contato em coordenadas globais (da perspectiva da partícula 0). */
        Vector3 contactNormal;

        /** Resolve o contato (velocidade e interpenetração). */
        void resolve(real duration);

        /** Calcula a velocidade de separação atual neste contato. */
        real calculateSeparatingVelocity() const;

    protected:
        /** Trata os cálculos de impulso para esta colisão. */
        void resolveVelocity(real duration);
    };
}
