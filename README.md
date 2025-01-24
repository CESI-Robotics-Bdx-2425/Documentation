# TIAGO JPO - Documentation

Ce projet a pour but de réaliser la distribution de flyer promotionnels en interaction avec le robot Tiago++ de PAL Robotics.

## Sommaire

0. [Prérequis](#0-prerequis)
1. [Points de vigilance](#1-points-de-vigilance)
2. [Installation de l'environnement](#2-installation-de-lenvironnement-machine)
3. [Préparation de l'environnement physique](#3-preparation-de-lenvironnement-physique)
4. [Réglage du robot / Calibration](#4-réglage-du-robot--calibration)
5. [Démarrage du programme](#5-démarrage-du-programme)
6. [Services](#6-services)
7. [Erreurs connues](#7-erreurs-connues)
8. [Ressources](#8-ressources)

## 0. Prerequis

Pour utiliser ce projet, il est nécessaire d'avoir en sa possession :
- Un robot Tiago++
- Un ordinateur avec Ubuntu 20.04 installé
- ROS Noetic installé sur l'ordinateur

Voir aussi [Prérequis](PREREQUISITES.md)

## 1. Points de vigilance

> [!CAUTION]  
> **SÉCURITÉ CRITIQUE : Toutes les actions réalisées avec le robot doivent être supervisées et contrôlées par un humain. Le robot ne peut en aucun cas rester seul sans surveillance.**

> [!IMPORTANT]  
> **NE JAMAIS utiliser la fonction 'HOME' de Tiago.** Pour remettre le robot dans sa position initiale, se référer au service `go_home`.

> [!IMPORTANT]  
> **ATTENTION : L'utilisation de la manette de Tiago n'arrête en aucun cas les mouvements programmés.** Il est nécessaire de recourir à un arrêt d'urgence pour interrompre les mouvements de Tiago.


## 2. Installation de l'environnement machine

### 1. Installation de l'environnement ROS

Voir [INSTALL.md](INSTALL.md)
> [!IMPORTANT]  
> Veillez à bien suivre les instructions de la partie : "3. Connexion au Robot TIAGO Physique"

### 2. Installation du workspace

**🔧 Procédure d'installation :**

1. Cloner le repository :
```bash
git clone https://github.com/CESI-Robotics-Bdx-2425/project-ws.git
cd project-ws
```

2. Construction du workspace avec catkin :
```bash
conda activate cesi-python
catkin_make
source devel/setup.bash
```

## 3. Preparation de l'environnement physique

Le robot Tiago utilise des marqueurs Aruco pour la calibration de son environnement. Une installation précise de ces éléments est fondamentale pour garantir une calibration optimale et un fonctionnement fiable du système.

### Configuration des tables

L'environnement de travail du robot Tiago nécessite deux tables d'écolier standard, dont les caractéristiques sont strictement définies :

**🔍 Spécifications essentielles des tables :**
- Hauteur précise de 76 centimètres (±5 millimètres)
- Longueur maximale (tables rassemblées) de 1.40 mètre
- Le robot doit être positionné parallèlement aux tables dans le sens de la longueur
- Distance critique de 50 centimètres (±1 centimètre) entre la pige et le bord de la table
> [!IMPORTANT]  
> **Cette mesure doit être prise par rapport au bord de la table et non par rapport aux pieds**

Pour optimiser l'interaction avec le robot, la disposition des supports doit suivre une configuration spécifique :
- Deux supports positionnés sur la gauche du robot
- Un support placé sur sa droite
> [!WARNING]  
> **Les supports ne doivent en aucun cas masquer les marqueurs Aruco présents sur la table**

### Installation des Arucos

Le système de marqueurs Aruco requiert une installation méticuleuse pour garantir son efficacité :

**🎯 Points clés pour l'installation des Arucos :**
- Chaque marqueur est spécifiquement assigné à un support particulier
- Utilisation d'une nomenclature descriptive (ex: "flyer", "table")
- Positionnement précis :
  - Alignement au centre du support
  - Alignement avec le dessous de la table
> [!IMPORTANT]  
> **Vérifier la mesure de la taille des Arucos avant le lancement du programme**

### Configuration matérielle

**🔧 Configuration requise :**
- Utilisation d'un switch réseau pour la connexion de plusieurs ordinateurs
- Configuration des pinces :
  - Une pince d'origine sur une main
  - Une pince imprimée en 3D sur l'autre main (Voir [Ressources](#ressources))

### Assemblage du support de flyer

Pour l'assemblage du support de flyer, il est nécessaire de préparer préalablement les pièces suivantes avec la découpeuse laser :
- Support_Cote.svg (×2)
- Support_Cale.svg (×1)
- Support_Travers.svg (×2)

### Vidéo récapitulative :
[![IMAGE ALT TEXT](http://img.youtube.com/vi/MmhG2tn8pBc/0.jpg)](http://www.youtube.com/watch?v=MmhG2tn8pBc "Video Title")

### Remplissage support de flyer

Lors du démarrage du robot, assurez-vous que l'entièreté des supports sont remplis.
Si le robot dit qu'il faut remplir l'un supports, il faudra remplir tous les supports (et pas uniquement celui qui est vide.)

**Pour cela, il faut :**
- cliquer sur le bouton de refill dans les paramètres de l'interface graphique.
- remplir TOUS les supports (pas uniquement celui qui est vide).
- cliquer sur le bouton de fin de refill dans les paramètres de l'interface graphique.

## 4. Réglage du robot / Calibration

**👥 Note importante : La calibration nécessite la présence de deux personnes pour une exécution optimale.**

### Préparation initiale

#### Configuration du WebCommander

1. **Accès au WebCommander :**
   - Ouvrir un navigateur et accéder à l'adresse : `10.68.0.1:8080`

2. **Configuration requise :**
   - Dans "Startup" : Désactiver "head_manager"
   - Dans "Robot Demos" : Activer "Gravity Compensation"

#### Positionnement du robot

**⚠️ Points critiques pour le positionnement :**
- La pince doit être en position de saisie d'un flyer
- L'articulation doit être alignée avec le marqueur Aruco
- Le bras doit maintenir une position stable
> [!IMPORTANT]  
> **Une fois la position stable atteinte, désactiver la compensation de gravité dans WebCommander pour garantir une mesure plus précise.**

### Processus de calibration

1. **Lancement du programme :**
```bash
roslaunch camera_arm_calibration camera_arm_calibration.launch
```

2. **Vérification des éléments essentiels :**
   - **🔍 Contrôler :**
     - La détection du marqueur Aruco par le robot
     - La génération correcte de la matrice de transformation

3. **Enregistrement de la calibration :**
   - Dans un nouveau terminal, exécuter :
```bash
rosservice call /camera_arm_calibration/save "{}"
```
   - **⚠️ Note :** Une erreur s'affichera et le premier terminal s'arrêtera. C'est un comportement normal.

4. **Validation de l'enregistrement :**
   - Vérifier la présence et le contenu du fichier :
```bash
cat project_ws/src/camera_arm_calibration/config/aruco.npy
```

> [!IMPORTANT]  
> - S'assurer que le robot ne bouge pas pendant la phase de calibration
> - En cas d'échec, recommencer la procédure depuis le début

## 5. Démarrage du programme

### Prérequis au démarrage

> [!IMPORTANT]  
> Avant de lancer le programme, s'assurer que :
> - L'environnement physique est correctement installé
> - Les supports de flyers sont en place
> - Les marqueurs Aruco sont correctement positionnés
> - La calibration du robot a été effectuée

### Lancement du programme

1. **Démarrage du système :**
```bash
roslaunch state_machine state_machine.launch
```

**🔄 Phase initiale :** Au démarrage, le robot effectue automatiquement une phase de calibration. Il est donc crucial que l'environnement physique soit parfaitement préparé avant le lancement.

### Accès à l'interface web

**🌐 Configuration de l'interface :**
- Trouver l'adresse IP du PC :
```bash
# Dans le terminal Docker
ifconfig
```
- L'adresse sera de la forme : `10.68.0.XX`
- Port à utiliser : `8000`

**💻 Accès à l'interface :**
- Option 1 : Sur la tablette du robot (si fonctionnelle)
- Option 2 : Sur un écran déporté
- URL d'accès : `http://10.68.0.XX:8000`

**⚡ Note :** En cas de non-fonctionnement de la tablette, privilégier l'utilisation d'un écran déporté pour une meilleure stabilité.

## 6. Services

### Homing

Ce service permet de mettre le robot dans sa position initiale sans utiliser la fonction 'HOME' qui peut causer des erreurs.
Voir [Homing.md](services/Homing.md)

### Pick_And_Give

Ce service permet de déclencher la récupération d'un flyer.
Voir [Pick_And_Give.md](services/Pick_And_Give.md)

## 7. Erreurs connues

Lors de notre projet, nous avons identifié plusieurs bugs et erreurs significatifs.

### Erreur Python : `move_group unreachable`

**🔍 Symptôme :** Le message d'erreur indique que le `move_group` est injoignable lors de l'utilisation de MoveIt.

**❌ Solutions non fonctionnelles :**
- Redémarrage du robot
- Redémarrage du conteneur Docker
- Déconnexion/reconnexion Ethernet

**⚡ Solution temporaire :** Attendre ou changer d'ordinateur.

### Erreur "Home"

**⚠️ AVERTISSEMENT CRITIQUE :**
Ne jamais utiliser la fonction "Home" native de Tiago. Cette action peut compromettre le fonctionnement de MoveIt.

**✅ Alternative sûre :** Utiliser notre fonction "Home" personnalisée, mais avec précaution :
- Vérifier l'espace libre autour du robot
- Être attentif à la vitesse d'exécution rapide

### Problèmes de manette

#### ⚠️ Reprise de contrôle

> [!IMPORTANT]  
> **Correction d'une information erronée :** Contrairement à ce qui a été indiqué en formation, appuyer sur "start" de la manette ne permet PAS d'interrompre une action en cours.

**🚨 Procédure d'urgence :**
- Solution temporaire : Fermer le terminal exécutant l'action
- **En cas de danger immédiat : Utiliser IMPÉRATIVEMENT l'arrêt d'urgence**

#### Manipulation de la manette

**⚠️ Recommandations importantes :**
- Éviter de déplacer le robot avec la manette pendant le développement
- Ne pas manipuler la tête avec la manette (risque de perte de contrôle)

### Problèmes de tablette

**🔧 Problème identifié :** Crashs fréquents lors de l'utilisation du 'CESI Publish Sound'

**✅ Solution recommandée :**
- Démarrer l'utilitaire depuis un ordinateur connecté
- En cas de crash : Redémarrer le robot et réitérer la procédure

### Précision des mouvements

**🎯 Problème :** Imprécision occasionnelle dans la saisie des flyers

**✅ Solution :** Recalibrer les bras du robot (voir section [Calibration](#4-réglage-du-robot--calibration))

## 8. Ressources

- [Fichier ARUCOS.pdf](files/Arucos.pdf)
- [Fichier Pince.stl](files/Pince.stl)
- [Fichier Support_Cote.svg](files/support_cote.svg)
- [Fichier Support_Cale.svg](files/support_cale.svg)
- [Fichier Support_Travers.svg](files/support_travers.svg)
