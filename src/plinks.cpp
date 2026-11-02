/*
 * plinks.cpp
 * 
 * Este arquivo contém a implementação dos links e restrições geométricas para partículas
 * (ParticleLink, ParticleCable e ParticleRod) no motor físico Cyclone.
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/plinks.h>
#include <cmath>

namespace cyclone {

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE PARTICLELINK
 * =========================================================
 */

real ParticleLink::currentLength() const {
    Vector3 relativePos = particle[0]->getPosition() - particle[1]->getPosition();
    return relativePos.magnitude();
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE PARTICLECABLE
 * =========================================================
 */

unsigned ParticleCable::addContact(ParticleContact *contact, unsigned limit) const {
    // Calcula o comprimento atual do cabo
    real length = currentLength();

    // Se o comprimento for menor que o limite máximo, o cabo está frouxo e não gera contato
    if (length < maxLength) {
        return 0;
    }

    // Configura o contato para impedir que as partículas se separem além do comprimento máximo
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];

    // Calcula a normal do contato (direção de separação)
    Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
    normal.normalize();
    contact->contactNormal = normal;

    // A profundidade da interpenetração é o quanto o cabo foi esticado além do permitido
    contact->penetration = length - maxLength;
    contact->restitution = restitution;

    return 1;
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE PARTICLEROD
 * =========================================================
 */

unsigned ParticleRod::addContact(ParticleContact *contact, unsigned limit) const {
    // Calcula o comprimento atual da haste
    real length = currentLength();

    // Se o comprimento for exatamente igual ao tamanho desejado, nenhuma força é necessária
    if (length == length) {
        return 0; // Nota: comparação direta de igualdade ou tolerância pequena
    }

    // Como as hastes impedem tanto a compressão quanto a extensão, verificamos a diferença absoluta
    real difference = particle[0]->getPosition().distance(particle[1]->getPosition());
    
    if (difference == length) {
        return 0;
    }

    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];

    Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
    normal.normalize();

    // Se a haste foi comprimida ou esticada, ajustamos a direção da normal e a penetração
    if (length > difference) {
        contact->contactNormal = normal;
        contact->penetration = length - difference;
    } else {
        contact->contactNormal = normal * -1.0f;
        contact->penetration = difference - length;
    }

    contact->restitution = 0.0f; // Hastes geralmente não possuem elasticidade de ressalto

    return 1;
}

} // namespace cyclone
