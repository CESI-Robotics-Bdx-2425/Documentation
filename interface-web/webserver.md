# Documentation du Serveur Web ROS


## Table des Matières


* [Introduction](#introduction)
* [Installation](#installation)
* [Fonctionnalités](#fonctionnalités)
* [Structure du Code](#structure-du-code)
* [Fonctionnement](#fonctionnement)
* [Gestion des Erreurs](#gestion-des-erreurs)
* [Démarrage du Serveur](#démarrage-du-serveur)
* [Conclusion](#conclusion)

## Introduction


Le serveur web ROS est conçu pour servir des fichiers statiques (HTML, CSS, JavaScript) à un client web. Il est initialisé en tant que nœud ROS et utilise asyncio pour gérer les requêtes de manière asynchrone.

### Principe de Fonctionnement

Le serveur web ROS utilise les technologies suivantes :

* ROS (Robot Operating System) pour l'intégration avec les nœuds ROS
* aiohttp pour gérer les requêtes HTTP de manière asynchrone
* asyncio pour gérer les requêtes de manière asynchrone

## Installation


Pour exécuter ce serveur, vous devez avoir ROS et les bibliothèques Python suivantes installées :

* rospy
* rospkg
* aiohttp

Vous devez installer ces bibliothèques en plus pour le bon fonctionnement :

```bash
pip install pyopenssl autobahn tornado bson pymongo service_identity psutil empy==3.3.4
```

### Configuration

Avant de démarrer le serveur, vous devez configurer les paramètres suivants :

* Le port HTTP
* Le chemin des fichiers statiques

## Fonctionnalités
--

* Servir des fichiers HTML, CSS et JavaScript.
* Gérer les requêtes HTTP de manière asynchrone.
* Redémarrer automatiquement en cas d'erreur.

### Fonctionnalités Avancées

* Gestion des erreurs de manière appropriée
* Enregistrement des messages d'erreur dans le journal ROS

## Structure du Code
-----

Le code est structuré autour de la classe WebServer, qui contient les méthodes suivantes :

* `__init__`: Initialise le nœud ROS et configure les paramètres.
* `handle_index`: Gère les requêtes pour le fichier index.html.
* `handle_roslib`: Gère les requêtes pour le fichier roslib.min.js.
* `handle_styles`: Gère les requêtes pour le fichier style.css.
* `handle_js`: Gère les requêtes pour le fichier leJcleS.js.
* `run_server`: Démarre le serveur HTTP.
* `run`: Boucle principale pour gérer le serveur.

## Fonctionnement
-

* Initialisation: Le serveur est initialisé en tant que nœud ROS. Les paramètres tels que le port HTTP sont récupérés.
* Gestion des Requêtes: Les méthodes de gestion des requêtes sont définies pour servir les fichiers statiques.
* Démarrage du Serveur: Le serveur HTTP est démarré et écoute les requêtes entrantes.
* Boucle de Fonctionnement: Le serveur reste en fonctionnement tant que la variable `self.running` est vraie.

## Gestion des Erreurs
-----

Le serveur gère les erreurs de manière appropriée en enregistrant les messages d'erreur dans le journal ROS. En cas d'erreur dans le serveur, il redémarre après un délai de 5 secondes.

### Gestion des Erreurs Avancées

* Enregistrement des messages d'erreur dans le journal ROS
* Redémarrage automatique en cas d'erreur

## Démarrage du Serveur
-------

Pour démarrer le serveur, vous devez exécuter le script suivant :

```bash
python web_server.py
```

## Conclusion
----------

Le serveur web ROS est un outil puissant pour servir des fichiers statiques à un client web. Il est conçu pour être facile à utiliser et à configurer, et il offre des fonctionnalités avancées pour la gestion des erreurs et la redémarrage automatique.