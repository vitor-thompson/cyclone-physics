/*
 * pworld.cpp
 * 
 * Este arquivo contém a implementação da classe 'ParticleWorld' no motor físico Cyclone.
 * Ele gerencia o ciclo de vida da simulação de partículas, incluindo o registro de forças,
 * geradores de contato, execução de passos de integração e resolução unificada.
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/pworld.h>

namespace cyclone {

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE PARTICLEWORLD
 * =========================================================
 */

ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations)
    : maxContacts(maxContacts),
      resolver(iterations),
      calculateIterations(iterations == 0) {
    contacts = new ParticleContact[maxContacts];
}

ParticleWorld::~ParticleWorld() {
    delete[] contacts;
}

void ParticleWorld::startFrame() {
    // Limpa acumuladores de força de todas as partículas no início do quadro
    for (auto p = particles.begin(); p != particles.end(); ++p) {
        (*p)->clearAccumulator();
    }
}

void ParticleWorld::runPhysics(real duration) {
    // 1. Aplica todas as forças registradas às respectivas partículas
    registry.updateForces(duration);

    // 2. Integra o movimento de todas as partículas (atualiza velocidades e posições)
    for (auto p = particles.begin(); p != particles.end(); ++p) {
        (*p)->integrate(duration);
    }

    // 3. Coleta e gera todos os contatos ativos no mundo
    unsigned usedContacts = 0;
    for (auto g = contactGenerators.begin(); g != contactGenerators.end(); ++g) {
        unsigned count = (*g)->addContact(contacts + usedContacts, maxContacts - usedContacts);
        usedContacts += count;
        
        // Se exceder o limite máximo de contatos, interrompe a coleta por segurança
        if (usedContacts >= maxContacts) {
            break;
        }
    }

    // 4. Resolve o conjunto de contatos gerados
    if (usedContacts > 0) {
        if (calculateIterations) {
            resolver.setIterations(usedContacts * 2);
        }
        resolver.resolveContacts(contacts, usedContacts, duration);
    }
}

ParticleWorld::Particles& ParticleWorld::getParticles() {
    return particles;
}

ParticleForceRegistry& ParticleWorld::getForceRegistry() {
    return registry;
}

ParticleWorld::ContactGenerators& ParticleWorld::getContactGenerators() {
    return contactGenerators;
}

} // namespace cyclone
