# Capitulo 16: Juntas e Restricoes (Joints and Constraints)

Este documento registra a teoria, arquitetura e implementacao de juntas e restricoes articuladas entre corpos rigidos no motor **Cyclone**, correspondente ao **Capitulo 16** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 16 expande o sistema de contatos para permitir a construcao de estruturas articuladas complexas — como portas com dobradicas, correntes, ragdolls e suspensoes de veiculos — conectando dois corpos rigidos ou um corpo rigido a um ponto fixo do mundo.

## 🔗 1. O Conceito de Junta como Restricao

Uma **junta** (*joint*) e uma restricao geometrica que reduz os graus de liberdade de movimento entre dois corpos rigidos. Em vez de resolver restricoes via matrizes globais complexas (metodo de coordenadas reduzidas ou multiplicadores de Lagrange), o motor **Cyclone** modela juntas gerando **contatos sinteticos** (*synthetic contacts*) a cada quadro.

### Tipos de Restricoes Suportadas:

1. **Junta Ponto-a-Ponto (*Ball and Socket Joint*):** Mantem um ponto de ancoragem no corpo A colado a um ponto de ancoragem no corpo B, permitindo rotacao livre em todas as direcoes 3D (3 graus de liberdade de rotacao, 0 de translacao).

2. **Haste Rigida entre Corpos (*Rigid Rod Constraint*):** Mantem a distancia entre dois pontos especificos em dois corpos rigidos estritamente constante.

3. **Cabo de Tracao (*Cable Constraint*):** Impede que dois pontos em corpos rigidos se afastem alem de um limite maximo, mas permite aproximacao livre.

## 🧮 2. Geracao de Contatos Sinteticos

A cada quadro de simulacao, a classe `Joint` avalia o desvio em relacao a posicao ideal da articulacao:

### 2.1 Junta Ponto-a-Ponto (`Joint`)

Dadas as posicoes de ancoragem nos espacos locais dos corpos ($\mathbf{p}_{lA}$ e $\mathbf{p}_{lB}$), calcula-se suas posicoes globais atuais:

$$
\mathbf{p}_{gA} = \mathbf{M}_A \cdot \mathbf{p}_{lA}, \quad \mathbf{p}_{gB} = \mathbf{M}_B \cdot \mathbf{p}_{lB}
$$

O erro de alinhamento e o vetor separacao:

$$
\mathbf{d} = \mathbf{p}_{gA} - \mathbf{p}_{gB}
$$

Se $\vert{}\mathbf{d}\vert{} > 0$, um contato sintetico e preenchido:

* **Ponto de Contato:** $\frac{1}{2}(\mathbf{p}_{gA} + \mathbf{p}_{gB})$

* **Normal de Contato:** $\mathbf{\hat{n}} = \frac{\mathbf{d}}{\vert{}\mathbf{d}\vert{}}$

* **Penetracao / Desvio:** $g = \vert{}\mathbf{d}\vert{}$

* **Restituicao:** $e = 0$ (para evitar oscilacoes elaticas na articulacao).

O resolvedor de contatos padrao (`ContactResolver`) processa esse contato sintetico aplicando impulsos lineares e angulares para puxar os dois pontos de volta a sobreposicao perfeita.

## 🧱 3. Ragdolls e Montagem de Cadeias

Combinando varias juntas ponto-a-ponto e restricoes de limites angulares, e possivel montar esqueletos articulados (*Ragdolls*).

### Desafios de Cadeias de Juntas:

* **Instabilidade em Cadeias Longas:** Resolver juntas sequencialmente via impulsos locais pode causar elasticidade visivel ("efeito borracha") em correntes longas ou ragdolls pesados.

* **Ajuste de Iteracoes:** Para que redes complexas de juntas permanecam estaveis, o numero de iteracoes do `ContactResolver` deve ser aumentado proporcionalmente ao numero total de juntas na estrutura.