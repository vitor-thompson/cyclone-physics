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
        virtual void updateForce(Particle *particle, real duration) = 0;
    };

    /**
     * @brief Gerador de força que aplica uma força gravitacional. 
     * @details Uma única instância pode ser compartilhada por múltiplas partículas.
     */
    class ParticleGravity : public ParticleForceGenerator
    {
    protected:
        /** Armazena a aceleração devido à gravidade. */
        Vector3 gravity;

    public:
        /** Cria o gerador com a aceleração de gravidade fornecida. */
        ParticleGravity(const Vector3 &gravity);

        /** Aplica a força gravitacional à partícula dada. */
        virtual void updateForce(Particle *particle, real duration);
    };

    /**
     * @brief Gerador de força que aplica resistência do ar / arrasto (drag).
     * @details Uma única instância pode ser compartilhada por múltiplas partículas.
     */
    class ParticleDrag : public ParticleForceGenerator
    {
    protected:
        /** Coeficiente de arrasto linear (dependente da velocidade). */
        real k1;
        /** Coeficiente de arrasto quadrático (dependente da velocidade ao quadrado). */
        real k2;

    public:
        /** Cria o gerador com os coeficientes de arrasto fornecidos. */
        ParticleDrag(real k1, real k2);

        /** Aplica a força de arrasto à partícula dada. */
        virtual void updateForce(Particle *particle, real duration);
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
        void add(Particle* particle, ParticleForceGenerator *fg);
        void remove(Particle* particle, ParticleForceGenerator *fg);
        void clear();
        void updateForces(real duration);
    };
}
