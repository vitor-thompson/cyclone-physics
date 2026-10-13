# Documentação de Física: Matemática Vetorial no Cyclone

Esta documentação resume os principais conceitos de física e matemática vetorial implementados na classe `Vector3` do motor de física **Cyclone**.

---

## 1. Vetores como Posições e Deslocamentos

Na mecânica clássica, um vetor em 3D representa uma posição espacial ou uma mudança de posição (deslocamento) através de três coordenadas cartesianas $(x, y, z)$.

- **Código associado:**
  ```cpp
  Vector3 posicao(10.0f, 0.0f, -5.0f);
  ```
## 2. Magnitude (Módulo) e Normalização
  
A magnitude representa o comprimento em linha reta de um vetor (distância euclidiana), calculada pelo Teorema de Pitágoras em 3D:

$$\vert{}\mathbf{a}\vert{} = \sqrt{x^2 + y^2 + z^2}$$

Otimização (Quadrado da Magnitude): Para evitar o uso custoso da função sqrt() em comparações de distância, utilizamos o quadrado da magnitude:

$$\vert{}\mathbf{a}\vert{}^2 = x^2 + y^2 + z^2$$

Normalização: Converter um vetor em um vetor unitário (tamanho 1) mantendo sua direção:

$$\hat{\mathbf{n}} = \frac{\mathbf{a}}{\vert{}\mathbf{a}\vert{}}$$

- **Código associado:**
  ```cpp
  real tamanho = v.magnitude();
  real tamanhoQuadrado = v.squareMagnitude();
  v.normalize();
  ```
## 3. Multiplicação por Escalar e Inversão
  
Multiplicar um vetor por um escalar redimensiona seu comprimento. Multiplicar por $-1$ inverte o seu sentido.
  
Fórmula:

$$k\mathbf{a} = (kx, ky, kz)$$

  ```Código associado:
  Vector3 aceleracaoDuplicada = aceleracao * 2.0f;
  v.invert(); // Inverte o sentido (equivalente a v *= -1)
  ```
## 4. Adição, Subtração e o Integrador (addScaledVector)

A adição e a subtração combinam deslocamentos. O método addScaledVector combina a adição vetorial com a multiplicação por escalar, sendo a base do integrador físico para atualizar posições com base na velocidade e no tempo ($\Delta t$):

$$\mathbf{p}_{novo} = \mathbf{p} + \mathbf{v} \cdot \Delta t$$

- **Código associado:**
  ```cpp
  // Atualiza a posição somando a velocidade multiplicada pelo tempo (dt)
  posicao.addScaledVector(velocidade, dt);
  ```
## 5. Produto Escalar (Dot Product)
  
O produto escalar multiplica componentes correspondentes e soma os resultados, gerando um número escalar. Ele mede o grau de alinhamento entre dois vetores:

$$\mathbf{a} \cdot \mathbf{b} = a_x b_x + a_y b_y + a_z b_z = |\mathbf{a}||\mathbf{b}|\cos\theta$$

 - **Código associado:**
  ```cpp
  real alinhamento = vetorA * vetorB; // ou vetorA.scalarProduct(vetorB);
  ```
## 6. Produto Vetorial (Cross Product)

O produto vetorial entre dois vetores gera um novo vetor que é estritamente perpendicular (ortogonal a 90°) aos dois primeiros. É fundamental para calcular torques e eixos de rotação.

$$\mathbf{a} \times \mathbf{b} = (a_y b_z - a_z b_y, \; a_z b_x - a_x b_z, \; a_x b_y - a_y b_x)$$

- **Código associado:**
  ```cpp
  Vector3 eixoPerpendicular = vetorA % vetorB; // ou vetorA.vectorProduct(vetorB);
  ```
