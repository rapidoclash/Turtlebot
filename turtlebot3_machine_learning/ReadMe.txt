                              Dans les fichiers environment_stage_0 et 1

La fonction def setReward est la fonction qui permet de donner les récompenses au robot :
	- current_distance : distance à l'objectif
        - heading : orientation par rapport à l'objectif (en radian)
        
        prends en param :
        	- state : tableau retourné par la fonction getState contenant : 
        		heading : l'angle par rapport à la cible
        		current_distance : la distance à la cible
        	- done : boolean indiquant si l'on a touché un mur
        	- action : retour de la fonction getAction dans le fichier turtlebot3_dqn_stage_0 et 1
        
        retourne :
        	- reward : récompense qui sera donnée au robot (positive ou négative)
        	
        	
        	
        	
        	               Dans les fichiers turtlebot3_dqn_stage_0 et 1

La fonction def __init__ est la fonction qui permet de définir les paramètres de bases :
	- self.load_model : permet de load un model deja entrainer => de base a False
        - self.load_episode : permet de dire quel model on veut load => de base a 0
        
        prends en param :
        	- state_size : nombre de laser du lidar
        	- action_size : nombre de possibilité de mobilité du robot (gauche, un peu gauche, tout droit, un peu droite, droite)
        retourne :
        	rien
        	

La fonction def buildModel est la fonction qui permet de construire le réseaux de neuronnes :
	prends en param : 
		rien
	
	retourne : 
		model : le model de réseaux de neuronnes



La fonction trainModel est la fonction qui permet d'entrainer le réseaux de neuronnes :
	prends en param : 
		target: boolean par default à False => je sais pas à quoi ça sert
	
	retourne : 
		rien
