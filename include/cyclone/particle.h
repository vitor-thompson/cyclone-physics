#ifndef CYCLONE_PARTICLE_H
#define CYCLONE_PARTICLE_H

#include "core.h"

namespace cyclone {

/**
 * @brief Uma particula e o objeto mais simples que pode ser simulado no motor de fisica.
 * 
 * Ela possui posicao, velocidade e aceleracao, mas nao tem extensao espacial nem orientacao.
 */
class Particle {
protected:
    /**
     * @brief Massa inversa da particula.
     * 
     * Armazenar a massa inversa (1/m) e computacionalmente util pois objetos com
     * massa infinita (estaticos) sao facilmente representados com massa inversa igual a zero.
     */
    real inverseMass;

    /**
     * @brief Coeficiente de amortecimento linear.
     * 
     * Simula a resistencia do ar ou atrito simples para evitar acúmulo numerico de energia.
     */
    real damping;

    /**
     * @brief Posicao da particula no espaco tridimensional (mundo).
     */
    Vector3 position;

    /**
     * @brief Velocidade linear atual da particula.
     */
    Vector3 velocity;

    /**
     * @brief Aceleracao acumulada da particula (ex: gravidade constante).
     */
    Vector3 acceleration;

    /**
     * @brief Acumulador de forcas a serem aplicadas no proximo passo de integracao.
     */
    Vector3 forceAccum;

public:
    /**
     * @brief Realiza a integracao numerica da particula no tempo (Integracao de Euler Modificada).
     * 
     * Atualiza a posicao e velocidade com base nas forcas acumuladas e no intervalo de tempo.
     * 
     * @param duration Intervalo de tempo do quadro (delta time em segundos).
     */
    void integrate(real duration) {
        // Nao integra particulas de massa infinita/nula
        if (inverseMass <= 0.0f) return;

        // Garante que a duracao seja estritamente positiva
        if (duration <= 0.0f) return;

        // Atualiza a posicao linear
        position.addScaledVector(velocity, duration);

        // Calcula a aceleracao resultante das forcas acumuladas
        Vector3 resultingAcc = acceleration;
        resultingAcc.addScaledVector(forceAccum, inverseMass);

        // Atualiza a velocidade linear a partir da aceleracao
        velocity.addScaledVector(resultingAcc, duration);

        // Aplica o amortecimento linear
        velocity *= real_pow(damping, duration);

        // Limpa o acumulador de forcas para o proximo quadro
        clearAccumulator();
    }

    /**
     * @brief Define a massa da particula.
     * @param mass Valor da massa em kg (deve ser > 0).
     */
    void setMass(const real mass) {
        if (mass != 0) {
            inverseMass = ((real)1.0) / mass;
        }
    }

    /**
     * @brief Retorna a massa da particula.
     */
    real getMass() const {
        if (inverseMass == 0) {
            return REAL_MAX;
        } else {
            return ((real)1.0) / inverseMass;
        }
    }

    /**
     * @brief Define a massa inversa diretamente.
     */
    void setInverseMass(const real inverseMass) {
        this->inverseMass = inverseMass;
    }

    /**
     * @brief Retorna a massa inversa da particula.
     */
    real getInverseMass() const {
        return inverseMass;
    }

    /**
     * @brief Verifica se a particula tem massa infinita (objeto estatico).
     */
    bool hasFiniteMass() const {
        return inverseMass > 0.0f;
    }

    /**
     * @brief Define o fator de amortecimento linear.
     */
    void setDamping(const real damping) {
        this->damping = damping;
    }

    /**
     * @brief Retorna o fator de amortecimento linear.
     */
    real getDamping() const {
        return damping;
    }

    /**
     * @brief Define a posicao da particula no espaco tridimensional.
     */
    void setPosition(const Vector3 &position) {
        this->position = position;
    }

    /**
     * @brief Define a posicao da particula pelas componentes X, Y, Z.
     */
    void setPosition(const real x, const real y, const real z) {
        position.x = x;
        position.y = y;
        position.z = z;
    }

    /**
     * @brief Preenche o vetor fornecido com a posicao da particula.
     */
    void getPosition(Vector3 *position) const {
        *position = Particle::position;
    }

    /**
     * @brief Retorna a posicao da particula.
     */
    Vector3 getPosition() const {
        return position;
    }

    /**
     * @brief Define a velocidade linear da particula.
     */
    void setVelocity(const Vector3 &velocity) {
        this->velocity = velocity;
    }

    /**
     * @brief Define a velocidade linear pelas componentes X, Y, Z.
     */
    void setVelocity(const real x, const real y, const real z) {
        velocity.x = x;
        velocity.y = y;
        velocity.z = z;
    }

    /**
     * @brief Preenche o vetor fornecido com a velocidade da particula.
     */
    void getVelocity(Vector3 *velocity) const {
        *velocity = Particle::velocity;
    }

    /**
     * @brief Retorna a velocidade linear da particula.
     */
    Vector3 getVelocity() const {
        return velocity;
    }

    /**
     * @brief Define a aceleracao constante da particula (ex: gravidade).
     */
    void setAcceleration(const Vector3 &acceleration) {
        this->acceleration = acceleration;
    }

    /**
     * @brief Define a aceleracao pelas componentes X, Y, Z.
     */
    void setAcceleration(const real x, const real y, const real z) {
        acceleration.x = x;
        acceleration.y = y;
        acceleration.z = z;
    }

    /**
     * @brief Preenche o vetor fornecido com a aceleracao da particula.
     */
    void getAcceleration(Vector3 *acceleration) const {
        *acceleration = Particle::acceleration;
    }

    /**
     * @brief Retorna a aceleracao da particula.
     */
    Vector3 getAcceleration() const {
        return acceleration;
    }

    /**
     * @brief Limpa todas as forcas acumuladas na particula.
     */
    void clearAccumulator() {
        forceAccum.clear();
    }

    /**
     * @brief Adiciona uma forca a ser aplicada na particula no proximo quadro.
     */
    void addForce(const Vector3 &force) {
        forceAccum += force;
    }
};

} // namespace cyclone

#endif // CYCLONE_PARTICLE_H
