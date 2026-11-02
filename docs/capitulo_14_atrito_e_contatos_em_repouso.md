# Capitulo 14: Atrito e Contatos em Repouso (Friction and Resting Contact)

Este documento registra os avanços no modelo de resolucao de contatos com a inclusao de atrito e suporte a corpos em repouso no motor **Cyclone**, correspondente ao **Capitulo 14** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 14 aprimora o sistema de impulsos desenvolvidos no Capitulo 13, estendendo a resolucao escalar de velocidade para um **sistema matricial tridimensional de base de contato** que lida simultaneamente com restituicao, atrito dinamico/estatico e micro-quiques (*resting contacts*).

## 🧭 1. A Base do Espaco de Contato (*Contact Basis*)

Para tratar o atrito de forma analitica, o motor projeta os vetores de velocidade e impulso do espaco do mundo para um sistema de coordenadas local alinhado ao ponto de impacto, denominado **Base de Contato** ($\mathbf{M}_{\text{contact}}$):

$$
\mathbf{M}_{\text{contact}} = \begin{bmatrix} \mathbf{\hat{n}} & \mathbf{\hat{x}} & \mathbf{\hat{y}} \end{bmatrix}^T
$$

* $\mathbf{\hat{n}}$: Normal de contato (eixo $Z$ local ou eixo $X$ primario no motor).
* $\mathbf{\hat{x}}, \mathbf{\hat{y}}$: Dois vetores tangentes ortogonais entre si e perpendiculares a normal $\mathbf{\hat{n}}$, representando o plano de atrito de contato.

## 🧱 2. Modelo de Atrito de Coulomb (Impulso 3D)

O atrito e simulado aplicando impulsos nas direcoes tangenciais ($\mathbf{\hat{x}}$ e $\mathbf{\hat{y}}$) para zerar a velocidade de deslizamento relativo no ponto de contato.

### O Cone de Atrito (*Coulomb Friction Cone*):

A magnitude total do impulso de atrito tangencial ($\mathbf{g}_{\text{fric}}$) e limitada pelo coeficiente de atrito ($\mu$) multiplicado pelo impulso normal aplicado ($g_n$):

$$
\vert{} \mathbf{g}_{\text{fric}} \vert{} \le \mu \cdot g_n
$$

1. **Atrito Estatico:** Se o impulso necessario para zerar o deslizamento estiver dentro do cone ($\vert{}\mathbf{g}_{\text{fric}}\vert{} \le \mu g_n$), o impulso tangencial e aplicado integralmente e o objeto para de deslizar.
2. **Atrito Dinamico:** Se o impulso tangencial exceder a borda do cone ($\vert{}\mathbf{g}_{\text{fric}}\vert{} > \mu g_n$), o impulso e escalonado para ficar exatamente no limite da borda do cone de atrito.

## 💤 3. Contatos em Repouso (*Resting Contact*)

Objetos parados sobre superfícies (como uma caixa descansando sobre uma mesa) tendem a acumular pequenas velocidades devido a aceleração gravitacional contínua a cada quadro, gerando quiques indesejados (*jitter*).

### Solucao para Micro-Quiques:

* **Remocao da Restituicao por Aceleracao:** Quando a velocidade de aproximação em um contato for causada puramente pela aceleracao do quadro ($\mathbf{a} \cdot \Delta t$), o coeficiente de restituicao $e$ e temporariamente forçado a zero ($e = 0$).

* **Ajuste de Velocidade Alvo:** Subtrai-se da velocidade de separacao desejada a componente gerada pelas forcas de corpo durante o quadro atual.

## 📐 4. Resolucao da Penetracao Posicional

Além de ajustar as velocidades por meio de impulsos, o motor resolve a sobreposição física entre os corpos (*penetration*) aplicando **deslocamentos posicionais e rotacionais diretos**:

1. A penetracao $g$ e dividida proporcionalmente a massa inversa e a inercia angular inversa dos dois corpos.
2. Aplica-se uma translacao linear e uma rotacao angular instantanea em ambos os corpos para descolar suas geometrias.
3. Para evitar vibrações numéricas contínuas, aplica-se um limiar de tolerancia de penetracao (*penetration epsilon*).