#pragma once

#include <cyclone/particle.h>
#include <vector>

namespace cyclone {
    /**
     * @brief Interface base para qualquer gerador de força no motor Cyclone.
     */
    class ParticleForceGenerator
    {
    public:
        virtual ~ParticleForceGenerator() {}

        /**
         * @brief Calcula e atualiza a força aplicada à partícula fornecida.
         */
        virtual void updateForce(Particle *particle, real duration) = 0;
    };

    /**
     * @brief Registro central que gerencia quais forças se aplicam a quais partículas.
     */
    class ParticleForceRegistry
    {
    protected:
        struct ParticleForceRegistration
        {
            Particle *particle;
            ParticleForceGenerator *fg;
        };

        typedef std::vector<ParticleForceRegistration> Registry;
        Registry registrations;

    public:
        /** Registra um gerador de força para uma partícula. */
        void add(Particle* particle, ParticleForceGenerator *fg);

        /** Remove o registro do par partícula-gerador. */
        void remove(Particle* particle, ParticleForceGenerator *fg);

        /** Limpa todos os registros de conexões. */
        void clear();

        /** Executa todos os geradores de força registrados para o quadro atual. */
        void updateForces(real duration);
    };
}
