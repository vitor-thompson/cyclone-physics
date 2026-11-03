# Capítulo 18: Outros Tipos de Física (Other Types of Physics)

## 1. Visão Geral

O Capítulo 18 conclui a jornada teórica e prática do desenvolvimento do motor físico **Cyclone**. Diferente dos capítulos anteriores — focados no desenvolvimento incremental de código —, este capítulo apresenta um panorama comparativo sobre decisões de arquitetura e abordagens alternativas de física utilizadas na indústria de jogos e simulações avançadas.

O motor **Cyclone** adota uma resolução de contatos baseada em **impulsos sequenciais**, o que proporciona alta performance, clareza didática e bom comportamento para ampla variedade de aplicações. Contudo, este capítulo explora outras arquiteturas de solução (como resolvedores simultâneos baseados em LCP/Jacobiano e abordagens de coordenadas reduzidas), mapeando suas vantagens, desvantagens e limites operacionais.

---

## 2. Principais Abordagens Analisadas

### 2.1. Resolução Simultânea de Contatos (Simultaneous Contact Resolution)

Enquanto o Cyclone trata colisões e restrições uma a uma iterativamente (o que pode gerar pequenos desvios ou tremores em empilhamentos complexos), motores de nível de produção frequentemente resolvem o conjunto completo de contatos ao mesmo tempo.

* **Física Baseada em Forças (*Force-Based Physics*):** Em vez de usar impulsos isolados por quadro para