# TIAGO JPO - Documentation

Ce projet à pour but de réaliser la distribution de flyer promotionnels en interaction avec le robot Tiago++ de PAL Robotics.

## Sommaire

0. [Prérequis](#prerequis)
1. [Points de vigilance](#points-de-vigilance)
2. [Installation de l'environnement](#installation-de-lenvironnement-machine)
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

Pour l'assemblage du support de flyer, il est auparavant nécessaire d'avoir préparer avec la découpeuse laser les pièces suivantes :
- Support_Cote.svg x2
- Support_Cale.svg x1
- Support_Travers.svg x2

> Vidéo à intégrer

### Disposition des tables

## Ressources

- [Fichier ARUCOS.pdf](files/Arucos.pdf)
- [Fichier Pince.stl](files/Pince.stl)
- [Fichier Support_Cote.svg](files/support_cote.svg)
- [Fichier Support_Cale.svg](files/support_cale.svg)
- [Fichier Support_Travers.svg](files/support_travers.svg)