#include <cyclone/pworld.h>

namespace cyclone {

    ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations)
        : resolver(iterations), maxContacts(maxContacts)
    {
        contacts = new ParticleContact[maxContacts];
        calculateIterations = (iterations == 0);
    }

    ParticleWorld::~ParticleWorld()
    {
        delete[] contacts;
    }

    void ParticleWorld::startFrame()
    {
        for (Particles::iterator p = particles.begin(); p != particles.end(); ++p)
        {
            (*p)->clearAccumulator();
        }
    }

    unsigned ParticleWorld::generateContacts()
    {
        unsigned limit = maxContacts;
        ParticleContact *nextContact = contacts;

        for (ContactGenerators::iterator g = contactGenerators.begin(); g != contactGenerators.end(); ++g)
        {
            unsigned used = (*g)->addContact(nextContact, limit);
            limit -= used;
            nextContact += used;

            // Se esgotarmos o limite do array de contatos, interrompe
            if (limit <= 0) break;
        }

        // Retorna o número total de contatos preenchidos
        return maxContacts - limit;
    }

    void ParticleWorld::integrate(real duration)
    {
        for (Particles::iterator p = particles.begin(); p != particles.end(); ++p)
        {
            (*p)->integrate(duration);
        }
    }

    void ParticleWorld::runPhysics(real duration)
    {
        // 1. Aplica todos os geradores de força
        registry.updateForces(duration);

        // 2. Integra o movimento de todas as partículas
        integrate(duration);

        // 3. Detecta e coleta todos os contatos do quadro
        unsigned usedContacts = generateContacts();

        // 4. Resolve os contatos gerados
        if (usedContacts > 0)
        {
            if (calculateIterations)
            {
                resolver.setIterations(usedContacts * 2);
            }
            resolver.resolveContacts(contacts, usedContacts, duration);
        }
    }

    void ParticleWorld::addContactGenerator(ParticleContactGenerator *generator)
    {
        contactGenerators.push_back(generator);
    }
}
