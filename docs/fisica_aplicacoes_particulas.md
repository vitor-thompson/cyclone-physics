# Documentação de Física: Aplicações do Motor de Partículas (Capítulo 4)

Esta documentação resume os conceitos práticos de física aplicada implementados no **Capítulo 4**, cobrindo simulações de **Balística** e **Sistemas de Partículas (Fogos de Artifício)**.

---

## 1. Balística e Ajustes de Design em Jogos
No mundo real, projéteis viajam a velocidades altíssimas (de $250\text{ m/s}$ a $1.800\text{ m/s}$). Em jogos, velocidades reais tornariam os projéteis invisíveis na tela. Por isso, desaceleramos os projéteis para velocidades visíveis (entre $5$ e $25\text{ m/s}$) e aplicamos duas compensações físicas:

- **Massa Compensatória (Conservação de Energia):** Para manter o impacto realista (ex: derrubar uma caixa), ajustamos a massa com base na velocidade reduzida utilizando a relação de energia cinética ($e = ms^2$).
- **Gravidade Reduzida:** Como projéteis mais lentos ficam mais tempo no ar, ajustamos a gravidade específica do projétil para evitar que caiam bruscamente a poucos metros:
  $$g_{bullet} = \frac{1}{s} \cdot g_{normal}$$

- **Código associado (`src/demos/ballistic/ballistic.cpp` - Configuração de Armas):**
  ```cpp
  // Exemplo: Configuração de um projétil de Artilharia
  shot->particle.setMass(200.0f);                    // 200 kg
  shot->particle.setVelocity(0.0f, 30.0f, 40.0f);    // Velocidade inicial vetorial
  shot->particle.setAcceleration(0.0f, -20.0f, 0.0f); // Gravidade acentuada para o arco
  shot->particle.setDamping(0.99f);                  // Amortecimento de arrasto
  ```

2. Sistemas de Partículas e Cadeias de Eventos (Fogos de Artifício)
Sistemas de partículas permitem simular explosões, fumaça, fogo e água. No motor Cyclone, criamos uma classe derivada Firework que herda de Particle e adiciona controle de tempo de vida e regras de evolução em cadeia.

Propriedades da Partícula de Fogo de Artifício:

type: Identificador numérico da regra de comportamento.

age: O tempo restante no rastilho (fuse-left). Quando atinge zero, o projétil detona e libera sua carga útil (payload).

- **Código associado (src/demos/fireworks/fireworks.cpp):**
 ```cpp
class Firework : public Particle
{
public:
    unsigned type;
    real age; // Tempo restante do rastilho

    bool update(real duration)
    {
        integrate(duration); // Reutiliza o integrador do Capítulo 3
        age -= duration;     // Reduz o tempo de vida
        return (age < 0.0f); // Retorna true se chegou a hora de explodir
    }
};
```
Geração de Filhos (Payloads e Variação Randômica):
Quando um foguete atinge o fim de sua vida (age < 0), ele é desativado e gera partículas filhas ao redor de sua posição atual, herdando a velocidade e somando um componente vetorial aleatório para simular a explosão isotrópica:

 ```cpp
void create(Firework *firework, const Firework *parent = nullptr) const
{
    Random r;
    firework->type = type;
    firework->age = r.randomReal(minAge, maxAge);

    if (parent) {
        firework->setPosition(parent->getPosition());
        Vector3 vel = parent->getVelocity();
        vel += r.randomVector(minVelocity, maxVelocity); // Adiciona dispersão aleatória
        firework->setVelocity(vel);
    }

    firework->setMass(1.0f);
```
    firework->setDamping(damping);
    firework->setAcceleration(Vector3(0.0f, -10.0f, 0.0f));
}  
