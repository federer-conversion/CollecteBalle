# Debrief du 20/01/2023 (Après-midi)

PO: Damien ESNAULT


## Bilan

Pourcentage de tâches réalisées: 100%

### Ce qui a fonctionné

- Guidage complet
- Fonction de coût à peaufiner (ajuster les coefs + guidage en absence de balle)
- Detection/Localisation de la pince et du robot
- Publication de la position et de l'orientation (peut etre ajouter un filtrage pour la pince, type ouverture)

### Ce qui n'a pas fonctionné

- Le code de commande de mirado
- Au final hugo n'a pas touché à mon code pour la manette

### Retour d'expérience du PO

- Le client conseille de faire un système qui stocke les balles dans une zone fermée (non visible de puis la caméra)

### Conseils pour le prochain PO

- Modifier la pince car perte de balles si coincé contre le mur (retour client)
- Gros problème d'inertie du robot pour le controle avec manette et quand la pince bouge => voir avec Mirado
- MIRADO remets la pince en position initiale car cela bloque tout !!!!

## Mesures à garder

- Prévoir les absences des membres de l'équipe (en théorie hugo ne revient pas)

### Sprint suivant : Besoin client
- Par ordre de priorité :
	* Chosir un style de code (voir norme google, python, etc) => Je pense PEP8 (norme python) ok!
	* Mettre en place des outils d'intégration continue sur Github (voir tutoriel sur le git de nathan)
	* Mettre en place des tests unitaires pour chaque programme (le client a dis que c'est le moins urgent)