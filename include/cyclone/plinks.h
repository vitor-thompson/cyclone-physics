#pragma once

#include <cyclone/particle.h>
#include <cyclone/pcontacts.h>

namespace cyclone {
    /**
     * @brief Classe base para conexões (links) entre duas partículas que geram contatos ao violar restrições.
     */
    class ParticleLink
    {
    public:
        /** O par de partículas conectadas pelo link. */
        Particle* particle[2];

    protected:
        /** Retorna o comprimento atual do link (distância entre as partículas). */
        real currentLength() const;

    public:
        /**
         * Preenche a estrutura de contato se a restrição for violada.
         * Retorna o número de contatos gerados (0 ou 1).
         */
        virtual unsigned fillContact(ParticleContact *contact, unsigned limit) const = 0;
    };

    /**
     * @brief Modela um cabo inextensível que gera um contato se as partículas se afastarem demais.
     */
    class ParticleCable : public ParticleLink
    {
    public:
        /** Comprimento máximo permitido do cabo. */
        real maxLength;

        /** Coeficiente de restituição (elasticidade do tranco) do cabo. */
        real restitution;

    public:
        /** Preenche o contato caso o cabo esteja esticado além do limite. */
        virtual unsigned fillContact(ParticleContact *contact, unsigned limit) const override;
    };
}
