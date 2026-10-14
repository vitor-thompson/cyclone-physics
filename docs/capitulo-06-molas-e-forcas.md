# Capítulo 6: Molas, Elasticidade e Geradores de Força

Este documento sintetiza os conceitos físicos, modelos matemáticos e a implementação prática desenvolvida no Capítulo 6 do motor de física *Cyclone*.

---

## 1. Fundamentos Físicos e a Lei de Hooke

A grande maioria dos comportamentos elásticos no mundo real pode ser modelada utilizando a **Lei de Hooke**, que rege a força restauradora exercida por uma mola deformada:

$$F = -k \Delta x$$

Onde:
* $F$ é a força resultante gerada pela mola.
* $k$ é a constante elástica (*spring constant*), que determina a rigidez da mola.
* $\Delta x$ é o vetor de elongação ou compressão (distância atual menos o comprimento de repouso $l_0$).

### Aplicações Diretas no Motor:
* **Molas entre Partículas (`ParticleSpring`):** Conecta duas partículas dinâmicas no espaço, aplicando forças de tração ou compressão proporcionais à distância entre elas.
* **Molas Ancoradas (`ParticleAnchoredSpring`):** Fixa uma extremidade da mola a um ponto estático no espaço (*anchor*).
* **Cabos Elásticos / *Bungee* (`ParticleBungee`):** Uma mola unilateral que só exerce força quando esticada além de seu comprimento de repouso ($\vert{}x\vert{} > l_0$), simulando cordas elásticas flexíveis.
* **Empuxo e Flutuação (`ParticleBuoyancy`):** Simula a força de empuxo hidrostático baseada na profundidade de imersão de um objeto em um fluido, comportando-se analogamente a uma mola linear.

---

## 2. O Problema das Molas Rígidas e o Método de Penalidade

Para simular colisões perfeitas ou objetos extremamente rígidos, teoricamente poderíamos usar molas com constantes elásticas altíssimas (método de penalidade). No entanto, em integradores numéricos explícitos (como o de Euler):

* **Instabilidade Numérica:** Se a constante $k$ for muito alta comparada ao passo de tempo ($\Delta t$), a força calculada no início do quadro empurra o objeto em excesso (*overshoot*). No quadro seguinte, a força inverte e cresce exponencialmente, fazendo com que o sistema exploda em direção ao infinito.
* **Solução Parcial (Molas Rígidas Simuladas / *Fake Implicit Springs*):** Para evitar a explosão em apêndices como cabelos ou tecidos leves, implementamos o gerador analítico `ParticleFakeSpring`, que resolve a EDO do oscilador harmônico amortecido para prever o comportamento ao fim do quadro.

---

## 3. Exemplo Prático de Código (`ParticleAnchoredSpring`)

Abaixo está o trecho principal que implementa a lógica da mola ancorada no motor *Cyclone*:

```cpp
void ParticleAnchoredSpring::updateForce(Particle *particle, real duration)
{
    // 1. Calcula o vetor de posição relativo à âncora
    Vector3 force = particle->getPosition();
    force -= anchor;

    // 2. Calcula a magnitude da elongação considerando o comprimento de repouso
    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;

    // 3. Normaliza a direção e aplica a força proporcional
    force.normalize();
    force *= -magnitude;
    particle->addForce(force);
}
```

