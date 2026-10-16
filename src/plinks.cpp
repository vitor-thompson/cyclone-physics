#include <cyclone/plinks.h>

namespace cyclone {

    real ParticleLink::currentLength() const
    {
        Vector3 relativePos = particle[0]->getPosition() - particle[1]->getPosition();
        return relativePos.magnitude();
    }

    unsigned ParticleCable::fillContact(ParticleContact *contact, unsigned limit) const
    {
        // 1. Calcula o comprimento atual do cabo
        real length = currentLength();

        // 2. Verifica se o cabo ultrapassou o comprimento máximo
        if (length < maxLength)
        {
            return 0; // Cabo frouxo, nenhum contato gerado
        }

        // 3. Preenche o contato para o resolvedor
        contact->particle[0] = particle[0];
        contact->particle[1] = particle[1];

        // A normal aponta da partícula 1 para a partícula 0 (puxando-as de volta)
        Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
        normal.normalize();
        contact->contactNormal = normal;

        // A penetração é o quanto o cabo foi esticado além do limite
        contact->penetration = length - maxLength;
        contact->restitution = restitution;

        return 1;
    }
}
