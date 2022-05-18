## Frequenze eccitazione
- Portante >= 1 rad/s
- Chirp a partire da 10 rad/s (primo polo che si incontra)
- Provare chirp al contrario
- tenere un giunto fermo mentre si muove l'altro (serve un controllore)
    - valutare possibile approccio closed-loop
    - in realtà i risultati sono già buoni usando l'open-loop
- **Le f.d.t. sono tra coppie motrici e velocità DEL MOTORE**
    - perchè nella pratica solitamente SP e PV sono riferiti alle coordinate ai giunti
    - PV: misurazione da encoder sull'albero motore
    - SP: traiettoria nello spazio dei giunti che deriva dalla cinematica\
inversa applicata alla traiettoria nello spazio cartesiano

## Risultati attesi
- Secondo motore: doppia antirisonanza
- Primo motore: risonanza singola, ma il picco non sarà enorme, dati i valori dei parametri

Le risonanze giacciono tra 80-200 rad/s

## Feedforward
- Implementare feedforward di coppia solo dopo aver tarato bene il doppio loop in cascata
- Opzionalmente implementare feedforward di velocità (potrebbe generare sovraelongazioni)
- Attenzione a come si implementa l'antiwindup!