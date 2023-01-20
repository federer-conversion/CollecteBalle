# Debrief du 20/01/2023 (Après-midi)

PO: Damien ESNAULT


## Bilan

Pourcentage de tâches réalisées: 

### Ce qui a fonctionné

- Guidage complet
- Fonction de coût à peaufiner (ajuster les coefs + guidage en absence de balle)
- Commande de la pince ok mais à améliorer (mettre en place un service + pb avec les bibliotheques)
- Detection/Localisation de la pince et du robot
- Publication de la position et de l'orientation

### Ce qui n'a pas fonctionné



### Retour d'expérience du PO

- Le client conseille de faire un système qui stocke les balles dans une zone fermée (non visible de puis la caméra)

### Conseils pour le prochain PO

- Modifier la pince car perte de balles si coincé contre le mur (retour client)
- Gros problème d'inertie du robot pour le controle avec manette et quand la pince bouge => voir avec Mirado

## Mesures à garder

- Prévoir les absences des membres de l'équipe

### Sprint suivant : Besoin client
- Par ordre de priorité :
	* Chosir un style de code (voir norme google, python, etc) => Je pense PEP8 ok!
	* Mettre en place des outils d'intégration continue sur Github
	* Mettre en place des tests unitaires pour chaque programme