# Capitulo 4: Geradores de Forca (Force Generators)

Este documento registra a analise teorica e a arquitetura de geradores de forca para particulas no motor **Cyclone**, correspondente ao **Capitulo 4** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 4 introduz uma abstracao fundamental para desvincular a simulacao do movimento das fontes que geram dinamica: o padrao de projeto **Force Generator** e a estrutura do **Force Registry**.

## 🎯 1. O Conceito de Gerador de Forca

Em vez de codificar diretamente na classe da particula os efeitos de gravidade, vento, molas ou arrasto, o motor utiliza uma arquitetura baseada em polimorfismo.

Cada tipo de interacao ou efeito fisico e encapsulado em uma classe derivada da interface abstrata `ParticleForceGenerator`.

### A Interface Base:

```cpp
class ParticleForceGenerator
{
public:
    virtual void updateForce(Particle *particle, real duration) = 0;
};
```

A cada quadro, o motor chama o metodo `updateForce` de cada gerador ativo, permitindo que este calcule e aplique as forcas necessarias a particula receptora.

## 🗂️ 2. O Registro de Forcas (*ParticleForceRegistry*)

O `ParticleForceRegistry` e o componente responsavel por manter o mapeamento entre quais geradores de forca devem afetar quais particulas.

* Mantem uma lista de registros contendo pares `(Particle*, ParticleForceGenerator*)`.
* A cada quadro, percorre a lista e executa `updateForce` para todas as ligacoes registradas.
* Permite adicionar ou remover dinamica em tempo de execucao (ex.: conectar ou desconectar uma mola).

## 🧮 3. Tipos de Geradores de Forca Implementados

### 3.1 Gravidade (`ParticleGravity`)

Aplica uma aceleracao gravitacional constante $\mathbf{g}$ (geralmente $-9.81 \mathbf{\hat{y}}$) independente da massa da particula.

$$
\mathbf{F}_{\text{grav}} = m \mathbf{g}
$$

### 3.2 Arrasto Aerodinamico (`ParticleDrag`)

Simula a resistencia do ar com base na velocidade da particula, utilizando dois coeficientes de atrito ($k_1$ e $k_2$):

$$
\mathbf{F}_{\text{drag}} = - \hat{\mathbf{v}} \left( k_1 \vert{}\mathbf{v}\vert{} + k_2 \vert{}\mathbf{v}\vert{}^2 \right)
$$

### 3.3 Mola Elastica (`ParticleSpring`)

Conecta uma particula a um ponto fixo de ancoragem no espaco ou a outra particula, seguindo a **Lei de Hooke**:

$$
\mathbf{F}_{\text{mola}} = -k \left( d - d_0 \right) \hat{\mathbf{d}}
$$

Onde $k$ e a constante de rigidez da mola, $d$ e a distancia atual, $d_0$ e o comprimento de repouso e $\hat{\mathbf{d}}$ e a direcao da mola.

### 3.4 Mola Ancorada (`ParticleAnchoredSpring`)

Variacao da mola elastica onde uma das pontas e fixa em uma coordenada constante do mundo.

### 3.5 Mola de Bungee (`ParticleBungee`)

Simula um cabo elastico flexivel que so gera forca de tracao quando esticado alem de seu comprimento de repouso ($d > d_0$). Se o cabo estiver frouxo ($d \le d_0$), nenhuma forca e aplicada.

### 3.6 Flutuabilidade (`ParticleBuoyancy`)

Simula a forca de empuxo exercida por um fluido em uma particula parcialmente ou totalmente submersa (Principio de Arquimedes).

$$
\mathbf{F}_{\text{empuxo}} = \begin{cases} 
0 & \text{se } d > d_{\text{max}} \\
\rho V \mathbf{\hat{y}} & \text{se } d < d_{\text{min}} \\
\rho V \left( \frac{d_{\text{max}} - d}{d_{\text{max}} - d_{\text{min}}} \right) \mathbf{\hat{y}} & \text{caso contrario}
\end{cases}
$$

Onde $\rho$ e a densidade do liquido, $V$ e o volume da particula e $d$ e a profundidade em relacao a superficie do liquido.