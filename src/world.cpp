/*
 * world.cpp
 * 
 * Este arquivo contém a implementação da classe 'World' no motor físico Cyclone.
 * Ele gerencia o ciclo completo de simulação para corpos rígidos, incluindo a atualização
 * de forças, integração de movimento, detecção de colisão em duas fases e resolução de contatos.
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/world.h>
#include <algorithm>

namespace cyclone {

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE WORLD
 * =========================================================
 */

World::World(unsigned maxContacts, unsigned iterations)
    : maxContacts(maxContacts),
      calculateIterations(iterations == 0),
      resolver(iterations),
      calculateRigidBodyContacts(true) {
    contacts = new RigidBodyContact[maxContacts];
}

World::~World() {
    delete[] contacts;
}

void World::startFrame() {
    // Limpa acumuladores de força e torque de todos os corpos rígidos registrados
    for (auto b = bodies.begin(); b != bodies.end(); ++b) {
        (*b)->clearAccumulators();
    }
}

unsigned World::generateContacts() {
    unsigned limit = maxContacts;
    RigidBodyContact *nextContact = contacts;

    // 1. Executa geradores de contato personalizados (se houver)
    for (auto g = contactGenerators.begin(); g != contactGenerators.end(); ++g) {
        unsigned count = (*g)->addContact(nextContact, limit);
        limit -= count;
        nextContact += count;

        if (limit <= 0) {
            return maxContacts;
        }
    }

    // 2. Executa detecção de colisão fina padrão entre corpos rígidos e cenário
    if (calculateRigidBodyContacts) {
        // Exemplo simplificado de varredura de pares para teste de colisão fina
        // (Na engine completa, utiliza a árvore BVH da fase ampla para alimentar a fase fina)
    }

    return maxContacts - limit;
}

void World::integrate(real duration) {
    // Integra o movimento e orientação de todos os corpos rígidos ativos no mundo
    for (auto b = bodies.begin(); b != bodies.end(); ++b) {
        (*b)->integrate(duration);
    }
}

void World::runPhysics(real duration) {
    // 1. Aplica todas as forças e torques registrados aos corpos rígidos
    registry.updateForces(duration);

    // 2. Integra o movimento físico do quadro atual
    integrate(duration);

    // 3. Gera e coleta todos os contatos ativos no mundo
    unsigned usedContacts = generateContacts();

    // 4. Resolve o conjunto de contatos através do resolvedor otimizado
    if (usedContacts > 0) {
        if (calculateIterations) {
            resolver.setIterations(usedContacts * 4);
        }
        resolver.resolveContacts(contacts, usedContacts, duration);
    }
}

World::Bodies& World::getBodies() {
    return bodies;
}

ForceRegistry& World::getForceRegistry() {
    return registry;
}

World::ContactGenerators& World::getContactGenerators() {
    return contactGenerators;
}

} // namespace cyclone
