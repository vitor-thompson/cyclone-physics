#pragma once

#include <cyclone/particle.h>
#include <cyclone/pcontacts.h>

namespace cyclone {
    class ParticleLink
    {
    public:
        Particle* particle[2];

    protected:
        real currentLength() const;

    public:
        virtual unsigned fillContact(ParticleContact *contact, unsigned limit) const = 0;
    };

    class ParticleCable : public ParticleLink
    {
    public:
        real maxLength;
        real restitution;

    public:
        virtual unsigned fillContact(ParticleContact *contact, unsigned limit) const override;
    };

    /**
     * @brief Modela uma haste rígida que mantém duas partículas a uma distância estritamente fixa.
     */
    class ParticleRod : public ParticleLink
    {
    public:
        /** Comprimento fixo da haste. */
        real length;

    public:
        /** Preenche o contato para impedir que a haste estique ou comprima. */
        virtual unsigned fillContact(ParticleContact *contact, unsigned limit) const override;
    };
}
