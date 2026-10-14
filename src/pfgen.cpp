#include <cyclone/pfgen.h>

namespace cyclone {
    // --- Implementação do Gerador de Gravidade ---
    ParticleGravity::ParticleGravity(const Vector3 &gravity)
        : gravity(gravity) {}

    void ParticleGravity::updateForce(Particle* particle, real duration)
    {
        // Verifica se a partícula possui massa finita (objetos estáticos ignoram gravidade)
        if (!particle->hasFiniteMass()) return;

        // Aplica a força escalada pela massa: F = m * g
        particle->addForce(gravity * particle->getMass());
    }

    // --- Arrasto (Drag) ---
    ParticleDrag::ParticleDrag(real k1, real k2)
        : k1(k1), k2(k2) {}

    void ParticleDrag::updateForce(Particle* particle, real duration)
    {
        Vector3 force = particle->getVelocity();
        
        // Calcula o coeficiente de arrasto total com base na magnitude da velocidade
        real dragCoeff = force.magnitude();
        dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

        // Calcula a força final no sentido oposto à velocidade (-v)
        force.normalize();
        force *= -dragCoeff;
        
        particle->addForce(force);
    }

    // --- Implementação do Registro de Forças ---
    void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator *fg)
    {
        ParticleForceRegistration registration;
        registration.particle = particle;
        registration.fg = fg;
        registrations.push_back(registration);
    }

    void ParticleForceRegistry::remove(Particle* particle, ParticleForceGenerator *fg)
    {
        Registry::iterator i = registrations.begin();
        for (; i != registrations.end(); ++i)
        {
            if (i->particle == particle && i->fg == fg)
            {
                registrations.erase(i);
                break;
            }
        }
    }

    void ParticleForceRegistry::clear()
    {
        registrations.clear();
    }

    void ParticleForceRegistry::updateForces(real duration)
    {
        Registry::iterator i = registrations.begin();
        for (; i != registrations.end(); ++i)
        {
            i->fg->updateForce(i->particle, duration);
        }
    }
}
