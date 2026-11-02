/*
 * pworld.h
 * 
 * Este arquivo define a classe 'ParticleWorld', que encapsula o mundo de simulação física
 * de partículas no motor Cyclone. Ele gerencia a lista de partículas ativas, o registro
 * de forças, geradores de contato e a execução do passo de integração temporal.
 * 
 * Parte da biblioteca física Cyclone.
 */

#ifndef CYCLONE_PWORLD_H
#define CYCLONE_PWORLD_H

#include "core.h"
#include "particle.h"
#include "pfgen.h"
#include "pcontacts.h"
#include <vector>

namespace cyclone {

/**
 * @brief Gerencia um conjunto de partículas, forças e contatos em uma simulação unificada.
 */
class ParticleWorld {
public:
    /**
     * @brief Interface que gera contatos entre partículas ou entre partículas e o ambiente.
     */
    class ParticleContactGenerator {
    public:
        /**
         * @brief Preenche o array de contatos fornecido com os contatos gerados.
         * 
         * @param contact Ponteiro para o início do array de contatos disponíveis.
         * @param limit Limite máximo de contatos que podem ser gerados.
         * @return O número de contatos gerados.
         */
        virtual unsigned addContact(ParticleContact *contact, unsigned limit) const = 0;
    };

protected:
    typedef std::vector<Particle*> Particles;
    Particles particles;

    // Controla se o mundo calcula a verificação de contatos automaticamente
    bool calculateIterations;

    // Registro de geradores de força globais aplicados às partículas
    ParticleForceRegistry registry;

    // Resolvedor de contatos do mundo
    ParticleContactResolver resolver;

    typedef std::vector<ParticleContactGenerator*> ContactGenerators;
    ContactGenerators contactGenerators;

    // Array de contatos disponíveis para a simulação por quadro
    ParticleContact *contacts;

    // Limite máximo de contatos permitidos por quadro
    unsigned maxContacts;

public:
    /**
     * @brief Construtor do mundo de partículas.
     * @param maxContacts Número máximo de contatos suportados por quadro.
     * @param iterations Número de iterações do resolvedor de contatos.
     */
    ParticleWorld(unsigned maxContacts, unsigned iterations = 0);

    /**
     * @brief Destrutor. Libera recursos alocados.
     */
    ~ParticleWorld();

    /**
     * @brief Executa um passo de simulação (avança no tempo).
     * @param duration Intervalo de tempo (delta time) do quadro.
     */
    void runPhysics(real duration);

    /**
     * @brief Inicializa o mundo para um novo quadro de simulação.
     */
    void startFrame();

    /**
     * @brief Retorna a lista de partículas cadastradas no mundo.
     */
    Particles& getParticles();

    /**
     * @brief Retorna o registro de forças do mundo.
     */
    ParticleForceRegistry& getForceRegistry();

    /**
     * @brief Retorna a lista de geradores de contato do mundo.
     */
    ContactGenerators& getContactGenerators();
};

} // namespace cyclone

#endif // CYCLONE_PWORLD_H
