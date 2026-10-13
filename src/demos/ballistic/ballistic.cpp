#include <cyclone/particle.h>
#include <iostream>

using namespace cyclone;

// Tipos de armas disponíveis na demonstração
enum ShotType {
    UNUSED = 0,
    PISTOL,
    ARTILLERY,
    FIREBALL,
    LASER
};

// Estrutura que representa um projétil disparado
struct AmmoRound {
    Particle particle;
    ShotType type;
    float startTime;
};

const unsigned int MAX_AMMO = 100;
AmmoRound ammo[MAX_AMMO];
ShotType currentShotType = PISTOL;

// Simula o disparo de um novo tiro
void fire(ShotType type)
{
    // Procura um slot livre na lista de munição
    AmmoRound *shot = nullptr;
    for (unsigned int i = 0; i < MAX_AMMO; i++) {
        if (ammo[i].type == UNUSED) {
            shot = &ammo[i];
            break;
        }
    }

    if (!shot) return; // Se não houver slots livres, retorna

    shot->type = type;
    shot->startTime = 0.0f; // Exemplo simplificado de tempo inicial
    shot->particle.setPosition(0.0f, 1.5f, 0.0f);
    shot->particle.clearAccumulator();

    // Configura os parâmetros físicos de acordo com o tipo de arma
    switch (type)
    {
    case PISTOL:
        shot->particle.setMass(2.0f);                      // 2.0 kg
        shot->particle.setVelocity(0.0f, 0.0f, 35.0f);     // 35 m/s
        shot->particle.setAcceleration(0.0f, -1.0f, 0.0f); // Gravidade leve
        shot->particle.setDamping(0.99f);
        break;

    case ARTILLERY:
        shot->particle.setMass(200.0f);                    // 200.0 kg
        shot->particle.setVelocity(0.0f, 30.0f, 40.0f);    // 50 m/s
        shot->particle.setAcceleration(0.0f, -20.0f, 0.0f); // Gravidade pesada/arco
        shot->particle.setDamping(0.99f);
        break;

    case FIREBALL:
        shot->particle.setMass(1.0f);                      // 1.0 kg
        shot->particle.setVelocity(0.0f, 0.0f, 10.0f);     // 5 m/s
        shot->particle.setAcceleration(0.0f, 0.6f, 0.0f);  // Flutua para cima
        shot->particle.setDamping(0.9f);
        break;

    case LASER:
        shot->particle.setMass(0.1f);                      // 0.1 kg
        shot->particle.setVelocity(0.0f, 0.0f, 100.0f);    // 100 m/s
        shot->particle.setAcceleration(0.0f, 0.0f, 0.0f);  // Sem gravidade
        shot->particle.setDamping(0.99f);
        break;

    case UNUSED:
        break;
    }
}

// Atualiza a física de todos os tiros ativos a cada quadro
void updatePhysics(real duration)
{
    for (int i = 0; i < MAX_AMMO; i++) {
        AmmoRound *shot = &ammo[i];
        if (shot->type != UNUSED) {
            // Executa o integrador físico que criamos no Capítulo 3
            shot->particle.integrate(duration);

            // Condições de remoção do projétil (atingiu o chão ou foi longe demais)
            Vector3 pos = shot->particle.getPosition();
            if (pos.y < 0.0f || pos.z > 200.0f) {
                shot->type = UNUSED;
            }
        }
    }
}

int main() {
    std::cout << "Motor Cyclone - Demo de Balística Inicializada." << std::endl;
    return 0;
}
