/*
 * Representação de Corpos Rígidos do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 * Atualizado com a correção de estabilidade de normalização de quatérnios (Capítulo 16).
 */

#ifndef CYCLONE_BODY_H
#define CYCLONE_BODY_H

#include "precision.h"
#include "core.h"

namespace cyclone {

    class RigidBody
    {
    protected:
        // ==========================================
        // 1. ESTADO BÁSICO DO CORPO RÍGIDO
        // ==========================================

        /// @brief Inverso da massa (armazenado como inverso para facilitar cálculos e simular objetos infinitamente pesados com 0).
        real inverseMass;

        /// @brief Posição do corpo rígido no espaço do mundo.
        Vector3 position;

        /// @brief Orientação do corpo rígido representada por um quatérnio unitário.
        Quaternion orientation;

        /// @brief Velocidade linear do corpo rígido no espaço do mundo.
        Vector3 velocity;

        /// @brief Velocidade angular (rotação) do corpo rígido no espaço do mundo.
        Vector3 rotation;

        // ==========================================
        // 2. MATRIZES E ACELERAÇÕES
        // ==========================================

        /// @brief Matriz de transformação que mapeia do espaço local para o espaço do mundo.
        Matrix4 transformMatrix;

        /// @brief Inverso do tensor de inércia no espaço local do corpo.
        Matrix3 inverseInertiaTensor;

        /// @brief Inverso do tensor de inércia transformado para o espaço do mundo (atualizado a cada quadro).
        Matrix3 inverseInertiaTensorWorld;

        /// @brief Aceleração linear acumulada (ex: gravidade).
        Vector3 acceleration;

        /// @brief Aceleração do último quadro, útil para compensações em contatos de repouso.
        Vector3 lastFrameAcceleration;

        // ==========================================
        // 3. FORÇAS E TORQUES ACUMULADOS
        // ==========================================

        Vector3 forceAccumulator;
        Vector3 torqueAccumulator;

        // ==========================================
        // 4. ESTADO DE MOVIMENTO (DORMIR / ATIVIDADE)
        // ==========================================

        bool isAwake;
        bool canSleep;

        /// @brief Amortecimento linear para dissipar energia gradativamente.
        real linearDamping;

        /// @brief Amortecimento angular para dissipar rotação gradativamente.
        real angularDamping;

    public:
        // ==========================================
        // MÉTODOS DE ATUALIZAÇÃO E INTEGRAÇÃO
        // ==========================================

        /**
         * Integra o corpo rígido avançando no tempo por um período 'duration'.
         * Aplica forças, velocidades, torques, rotações e executa a normalização 
         * de segurança do quatérnio (Seção 16.1.1).
         */
        void integrate(real duration)
        {
            if (!isAwake) return;

            // 1. Calcula a aceleração linear a partir das forças acumuladas
            lastFrameAcceleration = acceleration;
            lastFrameAcceleration += forceAccumulator * inverseMass;

            // 2. Calcula a aceleração angular a partir dos torques acumulados
            Vector3 angularAcceleration = inverseInertiaTensorWorld.transform(torqueAccumulator);

            // 3. Atualiza as velocidades linear e angular
            velocity += lastFrameAcceleration * duration;
            rotation += angularAcceleration * duration;

            // 4. Aplica o amortecimento (drag)
            velocity *= real_pow(linearDamping, duration);
            rotation *= real_pow(angularDamping, duration);

            // 5. Atualiza a posição e a orientação
            position += velocity * duration;
            orientation.addScaledVector(rotation, duration);

            // =====================================================================
            // CORREÇÃO DE ESTABILIDADE (Capítulo 16.1.1 - Quaternion Drift)
            // Normaliza o quatérnio para evitar deformações visuais e esmagamentos
            // causados pelo acúmulo de erros de ponto flutuante.
            // =====================================================================
            orientation.normalize();

            // 6. Reconstrói a matriz de transformação do mundo e o tensor de inércia
            calculateDerivedData();

            // 7. Limpa os acumuladores para o próximo quadro
            clearAccumulators();
        }

        /**
         * Recalcula a matriz de transformação e o tensor de inércia no mundo
         * com base na nova posição e orientação atualizadas.
         */
        void calculateDerivedData()
        {
            orientation.normalize();
            transformMatrix.setOrientationAndPos(orientation, position);

            // Transforma o tensor de inércia para o espaço do mundo: R * I_local * R^T
            // (implementação padrão do motor Cyclone)
            calculateInverseInertiaTensorWorld(
                &inverseInertiaTensorWorld, 
                inverseInertiaTensor, 
                transformMatrix
            );
        }

        void clearAccumulators()
        {
            forceAccumulator = Vector3::ZERO;
            torqueAccumulator = Vector3::ZERO;
            lastFrameAcceleration = acceleration;
        }

        // ==========================================
        // MÉTODOS AUXILIARES E GETTERS/SETTERS
        // ==========================================

        void setAcceleration(const Vector3 &acc) { acceleration = acc; }
        Vector3 getAcceleration() const { return acceleration; }
        Vector3 getLastFrameAcceleration() const { return lastFrameAcceleration; }

        void setPosition(const Vector3 &pos) { position = pos; }
        Vector3 getPosition() const { return position; }

        void setOrientation(const Quaternion &ori) { orientation = ori; orientation.normalize(); }
        Quaternion getOrientation() const { return orientation; }

        void setVelocity(const Vector3 &vel) { velocity = vel; }
        Vector3 getVelocity() const { return velocity; }

        void setRotation(const Vector3 &rot) { rotation = rot; }
        Vector3 getRotation() const { return rotation; }

        real getInverseMass() const { return inverseMass; }
        bool hasFiniteMass() const { return inverseMass >= 0.0f; }

        void getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const
        {
            *inverseInertiaTensor = inverseInertiaTensorWorld;
        }

        void setAsleep(bool asleep = true)
        {
            isAwake = !asleep;
            if (asleep) {
                velocity = Vector3::ZERO;
                rotation = Vector3::ZERO;
            }
        }

        bool getIsAwake() const { return isAwake; }

    private:
        // Função auxiliar interna para transformação do tensor de inércia
        static void calculateInverseInertiaTensorWorld(
            Matrix3 *iitWorld, 
            const Matrix3 &iitBody, 
            const Matrix4 &rotmat
        )
        {
            // Implementação padrão de rotação de tensor de inércia R * I * R^T
            real t4 = rotmat.data[0]*iitBody.data[0]+rotmat.data[1]*iitBody.data[3]+rotmat.data[2]*iitBody.data[6];
            real t9 = rotmat.data[0]*iitBody.data[1]+rotmat.data[1]*iitBody.data[4]+rotmat.data[2]*iitBody.data[7];
            real t14 = rotmat.data[0]*iitBody.data[2]+rotmat.data[1]*iitBody.data[5]+rotmat.data[2]*iitBody.data[8];
            real t28 = rotmat.data[4]*iitBody.data[0]+rotmat.data[5]*iitBody.data[3]+rotmat.data[6]*iitBody.data[6];
            real t33 = rotmat.data[4]*iitBody.data[1]+rotmat.data[5]*iitBody.data[4]+rotmat.data[6]*iitBody.data[7];
            real t38 = rotmat.data[4]*iitBody.data[2]+rotmat.data[5]*iitBody.data[5]+rotmat.data[6]*iitBody.data[8];
            real t52 = rotmat.data[8]*iitBody.data[0]+rotmat.data[9]*iitBody.data[3]+rotmat.data[10]*iitBody.data[6];
            real t57 = rotmat.data[8]*iitBody.data[1]+rotmat.data[9]*iitBody.data[4]+rotmat.data[10]*iitBody.data[7];
            real t62 = rotmat.data[8]*iitBody.data[2]+rotmat.data[9]*iitBody.data[5]+rotmat.data[10]*iitBody.data[8];

            iitWorld->data[0] = t4*rotmat.data[0]+t9*rotmat.data[1]+t14*rotmat.data[2];
            iitWorld->data[1] = t4*rotmat.data[4]+t9*rotmat.data[5]+t14*rotmat.data[6];
            iitWorld->data[2] = t4*rotmat.data[8]+t9*rotmat.data[9]+t14*rotmat.data[10];
            iitWorld->data[3] = t28*rotmat.data[0]+t33*rotmat.data[1]+t38*rotmat.data[2];
            iitWorld->data[4] = t28*rotmat.data[4]+t33*rotmat.data[5]+t38*rotmat.data[6];
            iitWorld->data[5] = t28*rotmat.data[8]+t33*rotmat.data[9]+t38*rotmat.data[10];
            iitWorld->data[6] = t52*rotmat.data[0]+t57*rotmat.data[1]+t62*rotmat.data[2];
            iitWorld->data[7] = t52*rotmat.data[4]+t57*rotmat.data[5]+t62*rotmat.data[6];
            iitWorld->data[8] = t52*rotmat.data[8]+t57*rotmat.data[9]+t62*rotmat.data[10];
        }
    };

} // namespace cyclone

#endif // CYCLONE_BODY_H
