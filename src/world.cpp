#include <cyclone/world.h>

using namespace cyclone;

World::World() : firstBody(nullptr) {}

World::~World()
{
    // Limpeza da lista de corpos cadastrados
    BodyRegistration *p = firstBody;
    while (p)
    {
        BodyRegistration *next = p->next;
        delete p;
        p = next;
    }
}

void World::startFrame()
{
    BodyRegistration *reg = firstBody;
    while (reg)
    {
        // Limpa os acumuladores e recalcula dados derivados (matrizes e inércia global)
        reg->body->clearAccumulators();
        reg->body->calculateDerivedData();

        reg = reg->next;
    }
}

void World::runPhysics(real duration)
{
    // 1. Aplica os geradores de força registrados
    // registry.updateForces(duration);

    // 2. Integra o movimento de cada corpo rígido do mundo
    BodyRegistration *reg = firstBody;
    while (reg)
    {
        reg->body->integrate(duration);
        reg = reg->next;
    }
}
