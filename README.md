# ROBOTIQUE
## Robot autonome suiveur de ligne
## DESCRIPTION
Ce projet consiste en la programmation d’un robot autonome basé sur la carte MeAuriga, capable de naviguer dans un environnement complexe à l’aide d’une machine à états finis .

### Le robot combine :
- le suivi de ligne par infrarouge
- la détection d’obstacles par capteur ultrasonique
- la stabilisation et le contrôle angulaire par gyroscope

### Le parcour inclus  :
- Des virages précis
- Des manœuvres de recul
- La gestion de croisements
- L' arrêt final contrôlé

 L’ensemble du comportement est géré de façon entièrement autonome, sans intervention humaine. 

## ARCHITECTURE

- Carte MeAuriga (Makeblock)

- Programmation Arduino / C++

- Machine à états finis  pour la navigation

- Capteurs multiples synchronisés

- Boucle principale non bloquante

- Utilisation d 'un code  en Python fourni par l 'enseignant pour démarrer le robot à travers l' ordinateur

## MACHINE À ÉTATS FINI (principales)

| État          | Rôle                                    |
| ------------- | --------------------------------------- |
| **CALIBRER**  | Calibration des capteurs IR (min / max) |
| **SUIVRE**    | Suivi de ligne en temps réel            |
| **TOURNER**   | Rotation contrôlée (gyroscope)          |
| **RECULER**   | Manœuvre de recul sur distance précise  |
| **STOPFINAL** | Arrêt complet du robot                  |

## FONCTIONNALITÉS

| Module                   | Comportement                                 |
| ------------------------ | -------------------------------------------- |
| **Suivi de ligne**       | 5 capteurs IR avec normalisation des valeurs |
| **Asservissement / PID** | Encodeurs moteurs pour vitesse constante     |
| **Distances contrôlées** | Calcul via encodeurs                         |
| **Virages précis**       | Rotation stabilisée par gyroscope            |
| **Croisements**          | Détection multi-capteurs                     |

## OBJECTIFS

- **Autonomie complète** : Réaliser un parcours complet sans aucune intervention humaine.

- **Précision sensorielle** : Calibration des capteurs IR (min/max) pour s’adapter aux surfaces et à la luminosité.

- **Modularité du code** : Structuration via  enum facilitant l’ajout de nouveaux états.

- **Optimisation du mouvement** : Synchronisation des moteurs gauche et droit pour des trajectoires fluides et des virages précis.

## PROBLEMES TECHNIQUES

| Problemes                | Descriptions                                                          |
| ------------------------ | --------------------------------------------------------------------- |
| Sensibilité à la lumière | Les capteurs IR peuvent être perturbés par une forte lumière ambiante et de la couleur du sol ce qui est compliqué de travailler dans un autre environnement  
| Dérive du gyroscope      | Accumulation d’erreurs angulaires sur de longs parcours  à travers la structure du sol et son taux de glisse               |
| Obstacles et virages     | Régler enfin d obtenir un virage plus précis                       |
| Batterie                 | Variation légère de précision selon la tension                        |
| Calcul du PID            | Calcul de l'encodeur qui était tres complexes                          | 

--------------------------------------------------------------------------------------------
CONCEPTION 22 NOVEMBRE - 06 DECEMBRE 2024 
CREATION DU REPOSITORIE 02 FEVRIER 2026

