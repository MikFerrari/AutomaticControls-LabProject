# Progetto Laboratorio di Automatica

## Per eseguire la valutazione del controllore, eseguire lo script src >> cascade_PID_optimal >> ValutazioneScore.m


Il repository è strutturato nel modo seguente:

ProjectEcho
	|
	|--- estimated_models
	|		|--- data -> strutture dati contenenti i parametri dei modelli identificati 
	|		|--- figures -> figure (.fig) dei modelli identificati 
	|
	|--- model_scripts -> classi e funzioni fornite inizialmente
	|
	|--- presentation
	|		|--- cartelle contenenti schemi e plot esportati
	|		|--- Presentazione_GruppoEcho.pptx -> Presentazione Powerpoint progetto
	|
	|--- simulations
	|		|--- identification
	|		|		|--- data -> strutture dati contenenti i risultati delle simulazioni di identificazione
	|		|		|--- figures -> figure (.fig) dei risultati delle simulazioni di identificazione
	|		|
	|		|--- validation
	|				|--- data -> strutture dati contenenti i risultati delle simulazioni di validazione
	|				|--- figures -> figure (.fig) dei risultati delle simulazioni di validazione
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
	|		|--- data -> strutture dati contenenti i parametri dei modelli validati 
	|		|--- figures -> figure (.fig) dei modelli validati 
	|
	|--- Note taratura PID.txt -> Log fine-tuning manuale
	|
	|--- Modello di SPONG SCARA Robot.pdf -> Modellizzazione matematica spazio degli stati