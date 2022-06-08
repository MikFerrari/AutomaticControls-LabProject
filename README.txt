# Progetto Laboratorio di Automatica

## Per eseguire la valutazione del controllore, eseguire lo script src >> cascade_PID_optimal >> ValutazioneScore.m


Il repository è strutturato nel modo seguente:

ProjectEcho
	|
	|--- estimated_models
	|		|--- data -> strutture dati parametri modelli identificati 
	|		|--- figures -> figure (.fig) modelli identificati 
	|
	|--- model_scripts -> classi e funzioni fornite inizialmente
	|
	|--- presentation
	|		|--- cartelle contenenti schemi e plot esportati
	|		|--- Presentazione_GruppoEcho.pptx -> Presentazione Powerpoint progetto
	|
	|--- simulations
	|		|--- identification
	|		|		|--- data -> strutture dati risultati simulazioni di identificazione
	|		|		|--- figures -> figure (.fig) risultati simulazioni di identificazione
	|		|
	|		|--- validation
	|				|--- data -> strutture dati risultati simulazioni di validazione
	|				|--- figures -> figure (.fig) risultati simulazioni di validazione
	|
	|--- src
	|		|--- cascade_PID_optimal -> tuning, discretizzazione e valutazione score
	|		|--- controllers
	|		|		|--- classes -> classi che implementano i controllori (PID, Cascata, SCARA) 
	|		|		|--- params -> parametri tarati e salvati
	|		|
	|		|--- identification -> identificazione e validazione
	|		|--- utils -> inizializzazione, studio ordine dei modelli, funzionalità aggiuntive
	|
	|--- validated_models
	|		|--- data -> strutture dati parametri modelli validati 
	|		|--- figures -> figure (.fig) modelli validati 
	|
	|--- Note taratura PID.txt -> Log fine-tuning manuale
	|
	|--- Modello di SPONG SCARA Robot.pdf -> Modellizzazione matematica spazio degli stati