#pragma once

#include <cyclone/particle.h>
#include <cyclone/pfgen.h>
#include <cyclone/pcontacts.h>
#include <vector>

namespace cyclone {

    /**
     * @brief Interface polimórfica para geradores de contatos de partículas (detectores de colisão, links, etc).
     */
    class ParticleContactGenerator
    {
    public:
        /**
         * Preenche o array de contatos fornecido com os contatos detectados.
         * Retorna o número de contatos efetivamente gerados.
         */
        virtual unsigned addContact(ParticleContact *contact, unsigned limit) const = 0;
    };

    /**
     * @brief Gerencia um conjunto de partículas, geradores de força e geradores de contato,
     * fornecendo o pipeline completo de simulação física por quadro.
     */
    class ParticleWorld
    {
    public:
        typedef std::vector<Particle*> Particles;
        typedef std::vector<ParticleContactGenerator*> ContactGenerators;

    protected:
        /** Lista de partículas registradas no mundo. */
        Particles particles;

        /** Registro de forças aplicadas às partículas. */
        ParticleForceRegistry registry;

        /** Resolvedor de contatos de partículas. */
        ParticleContactResolver resolver;

        /** Lista de geradores de contato registrados. */
        ContactGenerators contactGenerators;

        /** Array para armazenar os contatos do quadro atual. */
        ParticleContact *contacts;

        /** Número máximo de contatos permitidos simultaneamente. */
        unsigned maxContacts;

        /** Flag para recalcular iterativamente o limite de iterações do resolvedor a cada quadro. */
        bool calculateIterations;

    public:
        /**
         * Cria um novo simulador do mundo de partículas.
         */
        ParticleWorld(unsigned maxContacts, unsigned iterations = 0);

        /**
         * Destrói o mundo e libera o array interno de contatos.
         */
        ~ParticleWorld();

        /**
         * Inicializa o mundo para um novo quadro zerando o acumulador de força de todas as partículas.
         */
        void startFrame();

        /**
         * Varre todos os geradores de contato registrados e coleta os contatos para o quadro.
         */
        unsigned generateContacts();

        /**
         * Avança o estado de todas as partículas no tempo usando o integrador.
         */
        void integrate(real duration);

        /**
         * Executa o pipeline completo da física para o quadro: forças -> integração -> contatos -> resolução.
         */
        void runPhysics(real duration);

        /** Retorna a lista de partículas do mundo. */
        Particles& getParticles() { return particles; }

        /** Retorna o registro de forças do mundo. */
        ParticleForceRegistry& getForceRegistry() { return registry; }

        /** Adiciona um gerador de contatos ao mundo. */
        void addContactGenerator(ParticleContactGenerator *generator);
    };
}
