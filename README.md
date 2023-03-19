# Projeto Embarcados

Desenvolvendo um controle para o Spotify.

## Entrega 1

### Integrantes

- João Gabriel Valentim
- Hudson Monteiro Araújo

### Ideia

A ideia central do projeto é controlar (softwares/aplicações) por meio de processamento de imagem, áudio e membrana de botões.
Então por meio de movimentos será possível identificar com gestos ou por meio de comando de voz a realização de uma tarefa.

### Nome

SmartControl 

### Usuários 

É especialmente útil para pessoas com mobilidade reduzida ou que precisam de uma forma mais intuitiva e precisa de controle de dispositivos eletrônicos.


### Software/Jogo 


O software de controle é o spotify.


### Jornada do usuários (3 pts)


jornada 1:
O usuário liga o sistema de som e o controle, e a matriz de botões acende indicando que está pronto para uso.

O usuário decide utilizar o comando de voz para começar a tocar música. Ele segura o botão do comando de voz e diz "tocar música". O sensor de áudio detecta a voz do usuário e envia um sinal para a caixa de som, que começa a tocar a música.

O usuário quer aumentar o volume. Ele usa a matriz de botões para selecionar o controle de volume. A caixa de som recebe o sinal da matriz e aumenta o volume de acordo.


O usuário decide mudar a música por comando de voz. Ele segura o botão do comando de voz novamente e diz "tocar próxima música". O sensor de áudio detecta a voz do usuário e envia um sinal para a caixa de som, que passa para a próxima música.

Finalmente, o usuário decide parar de usar o controle e desliga a caixa de som e o controle. Ele pressiona o botão de desligar na matriz de botões e o controle se desliga.

jornada 2:

O usuário liga o sistema de som e o controle, e a matriz de botões acende indicando que está pronto para uso.

A música está tocando, mas o usuário quer mudar de música. Ele faz um gesto com a mão para a direita, indicando que quer avançar para a próxima música. O sensor de movimento detecta o gesto e envia um sinal para a caixa de som, que passa para a próxima música.

O usário gostou da música, com um gesto ele curte a música que está escutando.

Finalmente, o usuário decide parar de usar o controle e desliga a caixa de som e o controle. Ele pressiona o botão de desligar na matriz de botões e o controle se desliga.


### Comandos/ Feedbacks (2 pts)

Comandos de gestos:
Mostrar o dedo para a esquerda ou direita para mudar de música.
Fazer um movimento para pausar ou retomar a reprodução de música.
Comando de voz para mudar a música
Comando de voz para pausar.
Feedbacks visuais na própria plataforma indicando que houve pausa ou mudou de música.
Led mostra que a musica passou ou voltou, ou pause/play na música. 


## In/OUT (3 pts)

Reconhecer gestos, comandos de avançar ou voltar, pause/ play : Camêra OpenMV M7
Comando de voz : Sensor de áudio LMV324
Volume da música: Matriz de botões
Trocou música : LED1
Pause / Play : LED2

### Design (2 pts)

![arkad] arkad.png



