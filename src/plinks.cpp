#include <cyclone/plinks.h>

namespace cyclone {

    real ParticleLink::currentLength() const
    {
        Vector3 relativePos = particle[0]->getPosition() - particle[1]->getPosition();
        return relativePos.magnitude();
    }

    // --- ParticleCable ---
    unsigned ParticleCable::fillContact(ParticleContact *contact, unsigned limit) const
    {
        real length = currentLength();
        if (length < maxLength) {
            return 0;
        }

        contact->particle[0] = particle[0];
        contact->particle[1] = particle[1];

        Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
        normal.normalize();
        contact->contactNormal = normal;
        contact->penetration = length - maxLength;
        contact->restitution = restitution;

        return 1;
    }

    // --- ParticleRod ---
    unsigned ParticleRod::fillContact(ParticleContact *contact, unsigned limit) const
    {
        real currentLen = currentLength();

        // Se o comprimento estiver exato, nenhum contato é necessário
        if (currentLen == length) {
            return 0;
        }

        contact->particle[0] = particle[0];
        contact->particle[1] = particle[1];

        Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
        normal.normalize();

        // Se a haste estiver muito longa, a normal puxa as partículas para perto.
        // Se estiver muito curta, a normal empurra as partículas para fora.
        if (currentLen > length) {
            contact->contactNormal = normal;
            contact->penetration = currentLen - length;
        } else {
            contact->contactNormal = normal * -1.0f;
            contact->penetration = length - currentLen;
        }

        // Hastes não possuem bounciness (restituição zero)
        contact->restitution = 0.0f;

        return 1;
    }
}
