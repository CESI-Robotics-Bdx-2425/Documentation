# TIAGO JPO - Documentation

Ce projet à pour but de réaliser la distribution de flyer promotionnels en interaction avec le robot Tiago++ de PAL Robotics.

## Sommaire

0. [Prérequis](#prerequis)
1. [Points de vigilance](#points-de-vigilance)
2. [Installation de l'environnement](#installation-de-lenvironnement-machine)
3. [Préparation de l'environnement physique](#preparation-de-lenvironnement-physique)
4. [Réglage du robot / Calibration](#réglage-du-robot--calibration)
5. [Démarrage du programme](#démarrage-du-programme)
6. [Services](#services)
8. [Erreurs connues](#erreurs-connues)
9. [Ressources](#ressources)

## Prerequis

Pour utiliser ce projet, il est nécessaire d'avoir en sa possession :
- Un robot Tiago++
- Un ordinateur avec Ubuntu 20.04 installé
- ROS Noetic installé sur l'ordinateur

Voir aussi [Prérequis](PREREQUISITES.md)

## Points de vigilance

> [!CAUTION]  
> Toutes les actions réalisées avec le robot doivent être supervisées et controllées par un humain. Le robot ne peut rester seul sans surveillance.

> [!IMPORTANT]  
> Ne pas utiliser la fonction 'HOME' de Tiago. Pour remettre le robot dans sa position initiale, se réferer au service `go_home`.

> [!IMPORTANT]  
> L'utilisation de la manette de Tiago, n'arrete en aucun cas les mouvements programmés. Il est nécessaire de recourir à un arrêt d'urgence pour interrompre les mouvements de Tiago.

## Installation de l'environnement machine

Voir [INSTALL.md](INSTALL.md)
> [!IMPORTANT]  
> Veillez à bien suivre les instructions de la partie : "3. Connexion au Robot TIAGO Physique"

## Preparation de l'environnement physique

Le robot Tiago utilise des Arucos pour la calibration de son environnement, il est donc nécessaire de bien positionner ces derniers pour assurer une calibration optimale. Le processus pour la préparation de l'environnemen de Tiago est décrit ci-dessous.

### Assemblage du support de flyer

#### Vidéo

https://youtu.be/MmhG2tn8pBc 

Pour l'assemblage du support de flyer, il est auparavant nécessaire d'avoir préparer avec la découpeuse laser les pièces suivantes :
- Support_Cote.svg x2
- Support_Cale.svg x1
- Support_Travers.svg x2

<iframe width="560" height="315" src="https://www.youtube.com/embed/MmhG2tn8pBc?si=NR1_361GET5mNWlv" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Disposition des tables

## Réglage du robot / Calibration

## Démarrage du programme

## Services

### Homing

Ce service permet de mettre le robot dans sa position initiale sans utiliser la fonctione 'HOME' qui est susceptible d'occasionner des erreurs.
Voir [Homing.md](services/Homing.md)

### Pick_And_Give

Ce service permet de déclencher la récupération d'un flyer.
Voir [Pick_And_Give.md](services/Pick_And_Give.md)

## Erreurs connues

Lors de notre projet, nous avons identifié certains bugs et erreurs.  
Dans cette partie de la documentation, nous allons évoquer les erreurs et bugs trouvés et fournir une solution pour celles-ci (si possible).

### Erreur Python : `move_group unreachable`

Parfois, lorsque vous tentez de faire bouger Tiago via MoveIt, sans raison apparente, vous pouvez obtenir des erreurs disant que le `move_group` est injoignable.

La cause de cette erreur n'a pas encore été identifiée.

La solution pour cette erreur n'a toujours pas été trouvée à ce jour.  
- Redémarrer le robot ne change rien.  
- Redémarrer le conteneur Docker ne change rien non plus.  
- Se brancher et se débrancher (Ethernet) ne change rien non plus.

La seule solution trouvée à ce jour : attendre ou changer d'ordinateur.

### "Home"

Lors de notre formation, nous avons appris que faire effectuer le mouvement "Home" à Tiago permet de le remettre dans son état initial.

Cette action ne doit pas être effectuée.

Elle a pour effet de faire bugger l'utilisation de MoveIt (surtout en utilisant la fonction 'go' d'un `move_group`).

Nous avons recodé une fonction "Home" qui ne pose pas de problème.

Il faut cependant savoir que cette fonction "Home" est plutôt rapide.  
Il faut donc uniquement l'appeler en faisant attention à l'environnement autour de Tiago : il faut qu'il ait de la place autour de lui. (Nous avons cassé certaines pinces imprimées en 3D avec cette action).

### Utilisation de la manette

#### Reprendre le contrôle

Lors de notre formation, nous avons également appris qu'utiliser la manette (en appuyant sur start) permet de reprendre le contrôle pendant que le robot effectue une action (afin d'éviter un accident).

Il s'avère que cela ne fonctionne pas ! La seule manière de faire en sorte que Tiago arrête son mouvement est de tuer le terminal qui exécute l'action.

> [!CAUTION]
> En cas de réel danger, il est nécessaire de recourir à l'utilisation de l'arrêt d'ugence.

#### Bouger le robot avec la manette

Lors du développement de fonctions pour les mouvements du robot, nous pensons qu'il faut éviter de bouger le robot avec la manette. Ce n'est pas certain, mais il semble que cela pose des problèmes avec le code.

Bouger la tête avec la manette a également tendance à faire perdre le contrôle de la tête.

### Tablette qui crash

La tablette a tendance à planter avec l'utilisation du 'CESI Publish Sound'.
L'apparition de ce problème est plutôt aléatoire, et nous ne savons pas vraiment comment l'éviter.

Nous préconisons de démarrer cet utilitaire via un ordinateur connecté avec Tiago et non depuis la tablette du robot. Dans le cas ou la tablette viendrait à s'éteindre avec cette manipulation, répeter celle-ci après avoir redémarrer le robot.

### Main qui va trop loin par rapport aux flyers

Lorsque Tiago va chercher les flyers, parfois il va trop loin et abîme les flyers.  
Si cela arrive, il faut refaire une calibration de ses bras (Voir la documentation pour la [calibration](#réglage-du-robot--calibration)).

## Ressources

- [Fichier ARUCOS.pdf](files/Arucos.pdf)
- [Fichier Pince.stl](files/Pince.stl)
- [Fichier Support_Cote.svg](files/support_cote.svg)
- [Fichier Support_Cale.svg](files/support_cale.svg)
- [Fichier Support_Travers.svg](files/support_travers.svg)