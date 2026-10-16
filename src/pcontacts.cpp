#include <cyclone/pcontacts.h>

namespace cyclone {

    void ParticleContact::resolve(real duration)
    {
        resolveVelocity(duration);
    }

    real ParticleContact::calculateSeparatingVelocity() const
    {
        Vector3 relativeVelocity = particle[0]->getVelocity();
        if (particle[1]) {
            relativeVelocity -= particle[1]->getVelocity();
        }
        return relativeVelocity * contactNormal; // Produto escalar
    }

   void ParticleContact::resolveVelocity(real duration)
    {
        // 1. Encontra a velocidade de separação na direção da normal de contato
        real separatingVelocity = calculateSeparatingVelocity();

        // Se já está se afastando, nenhum impulso é necessário
        if (separatingVelocity > 0) {
            return;
        }

        // 2. Calcula a nova velocidade desejada com base na restituição
        real newSepVelocity = -separatingVelocity * restitution;

        // 3. Verifica o acúmulo de velocidade causado exclusivamente pela aceleração no quadro atual
        Vector3 accCausedVelocity = particle[0]->getAcceleration();
        if (particle[1]) {
            accCausedVelocity -= particle[1]->getAcceleration();
        }
        real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

        // Se houver velocidade de aproximação gerada pela aceleração, removemos do cálculo de ricochete
        if (accCausedSepVelocity < 0)
        {
            newSepVelocity += restitution * accCausedSepVelocity;
            if (newSepVelocity < 0) {
                newSepVelocity = 0; // Garante que não removemos mais do que o necessário
            }
        }

        real deltaVelocity = newSepVelocity - separatingVelocity;

        // 4. Soma a massa inversa total
        real totalInverseMass = particle[0]->getInverseMass();
        if (particle[1]) {
            totalInverseMass += particle[1]->getInverseMass();
        }

        if (totalInverseMass <= 0) return;

        // 5. Calcula e aplica o impulso proporcional à massa inversa
        real impulse = deltaVelocity / totalInverseMass;
        Vector3 impulsePerIMass = contactNormal * impulse;

        particle[0]->setVelocity(
            particle[0]->getPosition() == particle[0]->getPosition() ? // Checagem de segurança
            particle[0]->getVelocity() + impulsePerIMass * particle[0]->getInverseMass() : 
            particle[0]->getVelocity()
        );

        if (particle[1]) {
            particle[1]->setVelocity(
                particle[1]->getVelocity() + impulsePerIMass * -particle[1]->getInverseMass()
            );
        }
    }
void ParticleContact::resolveInterpenetration(real duration)
    {
        // Se não houver penetração, nada a fazer
        if (penetration <= 0) return;

        real totalInverseMass = particle[0]->getInverseMass();
        if (particle[1]) {
            totalInverseMass += particle[1]->getInverseMass();
        }

        if (totalInverseMass <= 0) return;

        // Calcula a quantidade de movimento corretiva por unidade de massa inversa
        Vector3 movePerIMass = contactNormal * (-penetration / totalInverseMass);

        // Aplica a correção de posição proporcional à massa inversa
        particle[0]->setPosition(
            particle[0]->getPosition() + movePerIMass * particle[0]->getInverseMass()
        );

        if (particle[1]) {
            particle[1]->setPosition(
                particle[1]->getPosition() + movePerIMass * -particle[1]->getInverseMass()
            );
        }
    }
}
