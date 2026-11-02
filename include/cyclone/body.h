/*
 * body.h
 * 
 * Este arquivo define a classe 'RigidBody' no motor físico Cyclone.
 * Um corpo rígido possui massa, posição, orientação no espaço 3D (usando quatérnions),
 * velocidade linear, velocidade angular e tensores de inércia para simulação física realista.
 * 
 * Parte da biblioteca física Cyclone.
 */

#ifndef CYCLONE_BODY_H
#define CYCLONE_BODY_H

#include "core.h"

namespace cyclone {

/**
 * @brief Representa um corpo rígido 3D na simulação física.
 */
class RigidBody {
protected:
    /**
     * @brief Inverso da massa do corpo (1/massa).
     * Um valor de 0 indica massa infinita (corpo inamovível/estático).
     */
    real inverseMass;

    /**
     * @brief Coeficiente de amortecimento linear para simular atrito com o ar.
     */
    real damping;

    /**
     * @brief Coeficiente de amortecimento angular para simular perda de rotação.
     */
    real angularDamping;

    /**
     * @brief Posição do centro de massa no espaço do mundo (world space).
     */
    Vector3 position;

    /**
     * @brief Orientação do corpo no espaço do mundo expressa por um Quatérnion.
     */
    Quaternion orientation;

    /**
     * @brief Velocidade linear do centro de massa.
     */
    Vector3 velocity;

    /**
     * @brief Velocidade angular (eixo e taxa de rotação em rad/s).
     */
    Vector3 rotation;

    /**
     * @brief Inverso do tensor de inércia no espaço local do corpo.
     */
    Matrix3 inverseInertiaTensor;

    /**
     * @brief Inverso do tensor de inércia transformado para o espaço do mundo.
     */
    Matrix3 inverseInertiaTensorWorld;

    /**
     * @brief Quantidade de força acumulada a ser aplicada no próximo passo de integração.
     */
    Vector3 forceAccum;

    /**
     * @brief Quantidade de torque acumulado a ser aplicado no próximo passo de integração.
     */
    Vector3 torqueAccum;

    /**
     * @brief Aceleração constante (ex: gravidade) aplicada ao corpo.
     */
    Vector3 acceleration;

    /**
     * @brief Aceleração linear do corpo acumulada do quadro anterior.
     */
    Vector3 lastFrameAcceleration;

    /**
     * @brief Indica se o corpo está acordado (ativo) ou em repouso (sleeping).
     */
    bool isAwake;

    /**
     * @brief Indica se o corpo pode entrar em modo de repouso para economizar processamento.
     */
    bool canSleep;

    /**
     * @brief Matriz de transformação afim 3x4 derivada da posição e orientação.
     */
    Matrix4 transformMatrix;

public:
    /**
     * @brief Calcula e atualiza os dados derivados do corpo (como a matriz de transformação e o tensor no mundo).
     */
    void calculateDerivedData();

    /**
     * @brief Integra o corpo rígido no tempo pelo intervalo dt.
     * @param duration Delta time do quadro.
     */
    void integrate(real duration);

    /**
     * @brief Define a massa do corpo rígido.
     */
    void setMass(const real mass);

    /**
     * @brief Retorna a massa do corpo.
     */
    real getMass() const;

    /**
     * @brief Define a massa inversa diretamente.
     */
    void setInverseMass(const real inverseMass);

    /**
     * @brief Retorna a massa inversa.
     */
    real getInverseMass() const;

    /**
     * @brief Define o tensor de inércia local.
     */
    void setInertiaTensor(const Matrix3 &inertiaTensor);

    /**
     * @brief Define a posição do corpo no espaço do mundo.
     */
    void setPosition(const Vector3 &position);

    /**
     * @brief Preenche a referência informada com a posição atual.
     */
    void getPosition(Vector3 *position) const;

    /**
     * @brief Retorna a posição do corpo.
     */
    Vector3 getPosition() const;

    /**
     * @brief Define a orientação do corpo.
     */
    void setOrientation(const Quaternion &orientation);

    /**
     * @brief Retorna a orientação do corpo.
     */
    Quaternion getOrientation() const;

    /**
     * @brief Define a velocidade linear.
     */
    void setVelocity(const Vector3 &velocity);

    /**
     * @brief Retorna a velocidade linear.
     */
    Vector3 getVelocity() const;

    /**
     * @brief Define a velocidade angular.
     */
    void setRotation(const Vector3 &rotation);

    /**
     * @brief Retorna a velocidade angular.
     */
    Vector3 getRotation() const;

    /**
     * @brief Limpa o acumulador de forças e torques.
     */
    void clearAccumulators();

    /**
     * @brief Aplica uma força no centro de massa do corpo rígido.
     */
    void addForce(const Vector3 &force);

    /**
     * @brief Aplica uma força em um ponto específico no espaço do mundo.
     * @param force Vetor da força.
     * @param point Ponto de aplicação no espaço do mundo.
     */
    void addForceAtPoint(const Vector3 &force, const Vector3 &point);

    /**
     * @brief Aplica uma força em um ponto relativo às coordenadas locais do corpo.
     */
    void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);

    /**
     * @brief Aplica um torque diretamente ao corpo.
     */
    void addTorque
