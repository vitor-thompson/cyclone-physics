#include <cyclone/pfgen.h>

namespace cyclone {
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
