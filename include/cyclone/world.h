/*
 * world.h
 * 
 * Este arquivo define a classe 'World' no motor físico Cyclone, responsável por 
 * gerenciar a simulação global de corpos rígidos. Ele coordena a integração de movimento,
 * aplicação de forças, detecção de colisão (fases ampla e fina) e resolução de contatos.
 * 
 * Parte da biblioteca física Cyclone.
 */

#ifndef CYCLONE_WORLD_H
#define CYCLONE_WORLD_H

#include "core.h"
#include "body.h"
#include "fgen.h"
#include "contacts.h"
#include "collide_fine.h"
#include "collide_coarse.h"
#include <vector>

namespace cyclone {

/**
 * @brief Gerencia a simulação completa de corpos rígidos, gravidade, forças e colisões.
 */
class World {
public:
    /**
     * @brief Interface para geradores de contatos personalizados no mundo (ex: colisões com o cenário).
     */
    class ContactGenerator {
    public:
        virtual unsigned addContact(RigidBodyContact *contact, unsigned limit) = 0;
    };

protected:
    typedef std::vector<RigidBody*> Bodies;
    Bodies bodies;

    // Controla se o resolvedor deve calcular o número de iterações automaticamente
    bool calculateIterations;

    // Registro global de forças aplicadas aos corpos rígidos
    ForceRegistry registry;

    // Resolvedor de contatos de corpos rígidos
    ContactResolver resolver;

    typedef std::vector<ContactGenerator*> ContactGenerators;
    ContactGenerators contactGenerators;

    // Buffer de contatos gerados durante o passo de simulação
    RigidBodyContact *contacts;

    // Limite máximo de contatos suportados por quadro
    unsigned maxContacts;

    // Indica se o gerador de contatos por colisão fina padrão está ativo
    bool calculateRigidBodyContacts;

public:
    /**
     * @brief Construtor do mundo de corpos rígidos.
     * @param maxContacts Número máximo de contatos suportados por quadro.
     * @param iterations Número de iterações do resolvedor de contatos.
     */
    World(unsigned maxContacts, unsigned iterations = 0);

    /**
     * @brief Destrutor. Libera todos os recursos alocados.
     */
    ~World();

    /**
     * @brief Avança a simulação física por um intervalo de tempo (duration).
     * @param duration Delta time (intervalo de tempo) do quadro atual.
     */
    void runPhysics(real duration);

    /**
     * @brief Inicializa o mundo para um novo quadro de simulação (limpa acumuladores).
     */
    void startFrame();

    /**
     * @brief Retorna a lista de corpos rígidos cadastrados no mundo.
     */
    Bodies& getBodies();

    /**
     * @brief Retorna o registro de forças do mundo.
     */
    ForceRegistry& getForceRegistry();

    /**
     * @brief Retorna a lista de geradores de contato do mundo.
     */
    ContactGenerators& getContactGenerators();

private:
    /**
     * @brief Inicia os processos de detecção de colisão (fase ampla e fina).
     */
    unsigned generateContacts();

    /**
     * @brief Integra o movimento de todos os corpos rígidos ativos no mundo.
     */
    void integrate(real duration);
};

} // namespace cyclone

#endif // CYCLONE_WORLD_H
