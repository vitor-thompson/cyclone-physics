# Documentação de Física: Leis do Movimento e Integrador (Capítulo 3)

Esta documentação resume os principais conceitos físicos, as leis de Newton e a implementação do integrador de partículas no motor **Cyclone**.

---

## 1. O Conceito de Partícula
No motor Cyclone, uma **partícula** representa uma massa pontual no espaço tridimensional que possui posição, velocidade e aceleração, mas **não possui orientação** (rotação). É ideal para simular projéteis, fagulhas e detritos.

- **Código associado (`include/cyclone/particle.h`):**
  ```cpp
  class Particle
  {
  public:
      Vector3 position;     // Posição linear no espaço
      Vector3 velocity;     // Velocidade linear
      Vector3 acceleration; // Aceleração linear (ex: gravidade)
  };
  ```
## 2. A 1ª Lei de Newton e o Amortecimento (Damping)

A primeira lei estabelece que um objeto continua com velocidade constante a menos que uma força atue sobre ele. Devido a erros de arredondamento de ponto flutuante em simulações computacionais, os objetos podem acumular energia indesejada. Para corrigir isso e simular arrasto/atrito, introduzimos o amortecimento (damping), que drena uma fração da velocidade a cada quadro.

Fórmula de atualização com damping:

$$\dot{\mathbf{p}} = \dot{\mathbf{p}} \cdot d^t + \ddot{\mathbf{p}}t$$

- **Código associado:**
  ```cpp
  // Aplica o amortecimento exponencial ajustado pelo tempo decorrido
  velocity *= real_pow(damping, duration);
  ```
## 3. A 2ª Lei de Newton e a Massa Inversa (Inverse Mass)

A segunda lei relaciona força, massa e aceleração ($f = ma$). Isolando a aceleração:

$$\ddot{\mathbf{p}} = \frac{1}{m}\mathbf{f}$$

Para evitar divisões por zero (massa zero) e representar objetos com massa infinita/imutáveis (como paredes e pisos, cuja massa inversa é 0), armazenamos a massa inversa em vez da massa pura.

- **Código associado(`include/cyclone/particle.h`):**
  ```cpp
  void setMass(const real mass) {
      if (mass <= 0.0f) { inverseMass = 0.0f; return; }
      inverseMass = 1.0f / mass;
  }
  ```

## 4. A Força da Gravidade

Como a distância da superfície terrestre ao centro da Terra é gigantesca ($6.400\text{ km}$), a aceleração da gravidade pode ser considerada constante. Quando combinamos a gravidade com a 2ª Lei de Newton ($a = \frac{1}{m}mg$), a massa se cancela. Isso significa que todos os objetos caem na mesma taxa, independentemente do peso.

Fórmula do vetor gravidade (eixo Y apontando para cima):

$$\mathbf{g} = \begin{bmatrix} 0 
                \\ -g
                \\ 0 \end{bmatrix}$$ 

## 5. O Integrador Físico (Newton-Euler)

O integrador é o bloco de código executado a cada quadro (frame) para atualizar o estado cinemático da partícula com base no tempo decorrido ($\Delta t$).

Equações implementadas:

1. Atualização de Posição:$\mathbf{p} = \mathbf{p} + \dot{\mathbf{p}} \cdot \Delta t$
2. Atualização de Velocidade: $\dot{\mathbf{p}} = \dot{\mathbf{p}} + \ddot{\mathbf{p}} \cdot \Delta t$

- **Código associado(`src/particle.cpp`):**
  ```cpp
  void Particle::integrate(real duration)
  {
      assert(duration > 0.0f);

    // 1. Atualiza a posição linear
    position.addScaledVector(velocity, duration);

    // 2. Calcula a aceleração resultante (incluindo forças acumuladas e massa inversa)
    Vector3 resultingAcc = acceleration;
    resultingAcc.addScaledVector(forceAccum, inverseMass);

    // 3. Atualiza a velocidade linear
    velocity.addScaledVector(resultingAcc, duration);

    // 4. Aplica o amortecimento (drag)
    velocity *= real_pow(damping, duration);

    // 5. Limpa o acumulador de forças para o próximo quadro
    forceAccum = Vector3(0, 0, 0);
  }
  ```  
