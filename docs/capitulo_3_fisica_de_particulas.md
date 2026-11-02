# Capitulo 3: Fisica de Particulas (Particle Physics)

Este documento registra os conceitos e a implementacao da simulacao de partículas sem extensao espacial no motor **Cyclone**, correspondente ao **Capitulo 3** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 3 introduz a menor unidade simulavel do motor: a **Particula**. Trata-se de um objeto pontual que possui massa e posicao no espaco 3D, mas nao possui volume, forma geométrica ou orientacao rotacional.

## ⚛️ 1. O Modelo de Particula

Uma particula e ideal para simular objetos onde a rotacao pode ser ignorada, como projeteis, gotas de chuva, fumaça ou pequenas esferas de poeira.

### Propriedades Fundamentais:

* **Posicao ($\mathbf{p}$):** Vetor 3D indicando a localizacao no espaco mundial.
* **Velocidade ($\mathbf{v}$):** Vetor 3D representando a taxa de variacao da posicao no tempo.
* **Aceleracao ($\mathbf{a}$):** Vetor 3D representando a taxa de variacao da velocidade. Pode ser constante (como a gravidade) ou acumulada a partir de forcas externas.
* **Amortecimento (*Damping* - $\gamma$):** Um fator de atrito numerico escalar (entre $0$ e $1$) aplicado a velocidade a cada quadro para simular o arrasto do ar e garantir estabilidade numerica.
* **Massa Inversa ($w = 1/m$):** Em vez de armazenar a massa $m$ diretamente, armazena-se a massa inversa $w$. Isso traz duas vantagens fundamentais:
  1. Permite representar **objetos inamoviveis (massa infinita)** definindo $w = 0$.
  2. Evita divisoes por zero nos calculos da 2ª Lei de Newton ($\mathbf{a} = \mathbf{F} / m = \mathbf{F} \cdot w$).

## ⚙️ 2. O Integrador de Newton-Euler

O motor utiliza o integrador **Newton-Euler de primeira ordem** (Euler Explicito / Semi-implicito) para atualizar o estado da particula a cada passo de tempo ($\Delta t$).

### Equacoes de Integracao:

1. **Atualizacao de Posicao:**
   

   $$
   \mathbf{p}' = \mathbf{p} + \mathbf{v} \cdot \Delta t
   $$

2. **Calculo da Aceleracao Resultante:**
   

   $$
   \mathbf{a}_{\text{total}} = \mathbf{a}_{\text{constante}} + \mathbf{F}_{\text{acumulada}} \cdot w
   $$

3. **Atualizacao de Velocidade:**
   

   $$
   \mathbf{v}' = \mathbf{v} \cdot \gamma^{\Delta t} + \mathbf{a}_{\text{total}} \cdot \Delta t
   $$

4. **Limpeza de Forcas:**
   Apos a integracao, as forcas acumuladas sao zeradas para preparar a particula para o proximo quadro.

## 💻 3. Estrutura da Classe `Particle`

A classe `Particle` e definida em `include/cyclone/particle.h` e implementada em `src/particle.cpp`.

### Principais Metodos:

* `integrate(real duration)`: Executa a integracao matematica de posicao e velocidade ao longo do intervalo `duration`.
* `setMass(real mass)` / `getMass()`: Utilitarios para manipular a massa ajustando internamente o valor de `inverseMass`.
* `addForce(const Vector3 &force)`: Acumula uma forca linear a ser processada no proximo passo de integracao.