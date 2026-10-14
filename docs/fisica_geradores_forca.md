# Documentação de Física: Geradores de Força e o Princípio de D'Alembert (Capítulo 5)

Esta documentação resume os conceitos arquiteturais e físicos implementados no **Capítulo 5**, cobrindo a soma de múltiplas forças simultâneas e a modularização através de interfaces e geradores de força.

---

## 1. O Princípio de D'Alembert e o Acumulador de Forças
Quando múltiplos campos de força atuam sobre um objeto ao mesmo tempo, podemos substituir todas essas forças por uma **única força resultante** obtida através da simples soma vetorial (Princípio de D'Alembert):

$$\mathbf{f} = \sum_{i} \mathbf{f}_i$$

Para gerenciar isso computacionalmente, adicionamos um vetor acumulador (`forceAccum`) à partícula. A cada quadro, as forças são acumuladas, integradas para gerar a aceleração resultante e, em seguida, o acumulador é limpo.

- **Código associado (`src/particle.cpp`):**
  ```cpp
  void Particle::integrate(real duration)
  {
      assert(duration > 0.0f);
      position.addScaledVector(velocity, duration);

      // Calcula a aceleração resultante incluindo as forças acumuladas
      Vector3 resultingAcc = acceleration;
      resultingAcc.addScaledVector(forceAccum, inverseMass);

      velocity.addScaledVector(resultingAcc, duration);
      velocity *= real_pow(damping, duration);

      clearAccumulator(); // Limpa as forças para o próximo quadro
  }

  void Particle::addForce(const Vector3 &force) {
      forceAccum.addScaledVector(force, 1.0f);
  }
  ```
## 2. Geradores de Força (Force Generators) e Polimorfismo

Para evitar que o motor de física precise conhecer os detalhes de cálculo de cada força específica, utilizamos uma interface abstrata (ParticleForceGenerator). O registro central (ParticleForceRegistry) associa partículas aos seus respectivos geradores e atualiza todas as forças a cada quadro.

-** Interface Abstrata (`include/cyclone/pfgen.h`):**
```C++
class ParticleForceGenerator
{
public:
    virtual ~ParticleForceGenerator() {}
    virtual void updateForce(Particle *particle, real duration) = 0;
};
```

## 3. Exemplos de Geradores: Gravidade Modular e Arrasto (Drag)

A. Gerador de Gravidade Modular (ParticleGravity)
Aplica uma força proporcional à massa da partícula ($\mathbf{f} = m \cdot \mathbf{g}$):

```C++
void ParticleGravity::updateForce(Particle* particle, real duration)
{
    if (!particle->hasFiniteMass()) return; // Objetos estáticos ignoram gravidade
    particle->addForce(gravity * particle->getMass());
}
```
B. Gerador de Arrasto / Resistência do Ar (ParticleDrag)

Modela forças de atrito com o ar com termos lineares ($k_1$) e quadráticos ($k_2$), opondo-se à direção da velocidade:

$$\mathbf{f}_{drag} = -\hat{\mathbf{v}} (k_1 \vert{}\mathbf{v}\vert{} + k_2 \vert{}\mathbf{v}\vert{}^2)$$

```C++
void ParticleDrag::updateForce(Particle* particle, real duration)
{
    Vector3 force = particle->getVelocity();
    real dragCoeff = force.magnitude();
    dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

    force.normalize();
    force *= -dragCoeff;
    particle->addForce(force);
}
```
