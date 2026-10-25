#pragma once

#include <cyclone/body.h>
#include <cyclone/fgen.h>

namespace cyclone {

    /**
     * @brief O mundo representa uma simulação independente de física para corpos rígidos.
     */
    class World
    {
    public:
        /**
         * @brief Mantém um corpo rígido em uma lista encadeada de corpos no mundo.
         */
        struct BodyRegistration
        {
            RigidBody *body;
            BodyRegistration *next;
        };

    protected:
        /**
         * @brief Ponteiro para o primeiro corpo registrado na lista.
         */
        BodyRegistration *firstBody;

        /**
         * @brief Registro de geradores de força do mundo.
         */
        // ForceRegistry registry; // Opcional / dependendo da sua implementação de fgen

    public:
        World();
        ~World();

        /**
         * @brief Inicializa o quadro de simulação (limpa acumuladores e calcula dados derivados).
         */
        void startFrame();

        /**
         * @brief Processa todas as forças e executa a integração física dos corpos.
         */
        void runPhysics(real duration);
    };

}
