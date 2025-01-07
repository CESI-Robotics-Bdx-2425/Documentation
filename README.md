# Documentation Tiago CESI

Cette documentation considère que vous êtes familié avec les environnements Linux. Toutes les commandes décrites ci-après, ont été testées dans un environnement Ubuntu 20.04.

## Prérequis

Disposer d'une machine Ubuntu 20.04 avec Docker installé. Se référer à [PRERIQUISITES.md](PREREQUISITES.md)

## Installation des outils

Pour pouvoir utiliser l'environnement de développement mis à disposition par CESI pour utiliser le robot TIAGO suivez les étapes qui seront décrites.
Dans un premier temps, il convient d'avoir télécharger l'image Docker fournie par le CESI. Dans cette documentation, l'image se nomme `docker_tiagocesi_container_full.tar.gz`

### Charger l'image de Tiago

Nous allons importer l'image fournie par le CESI dans docker et la renomée `docker_tiagocesi_image_full`.

```bash
docker import docker_tiagocesi_container_full.tar.gz docker_tiagocesi_image_full
```

Vérifiez que cette dernière a bien été importée.
```bash
docker image list
```

### Démarrer un conteneur

Pour démarrer le conteneur contenant les paquets ROS de Tiago utilisez la commande suivante:
```bash
docker run \
  -it \
  -d \
  -u user \
  -e DISPLAY=$DISPLAY \
  -p 80:80 \
  -p 8000:8000 \
  -p 443:443 \
  -p 9090:9090 \
  --privileged \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  --name="tiago" \
  docker_tiagocesi_image_full \
  bash
```

> NB: Sur MAC qui est un environnement UNIX il est nécéssaire d'installer XQuartz pour pouvoir utiliser les GUI des outils au sein du Docker. Il faudra alors remplacer `-e DISPLAY=$DISPLAY` par `-e DISPLAY=host.docker.internal:0`.

Vous pouvez ensuite accéder au docker en utilisant :

```bash
docker exec -it tiago bash
```

Pour plus d'aisance il est recommandé de passer par un environnement VSCode pouvant accéder au conteneur Docker.
Dans un terminal saisir:  
![Start VSCode](images/code.png "Start VSCode")

Cela lance une fenetre VSCode. Connectez vous au conteneur Docker de tiago.
![Start Container](images/container.png "Start Container")
![Tiago Container](images/tiago_docker.png "Tiago Container")

Par ailleurs, il est recommandé d'ajouter au fichier `.bashrc` les deux lignes suivantes
```bash
nano ~/.bashrc
```
```bash
source /opt/pal/gallium/setup.bash
source /usr/share/cesi-tiago-package/behaviour_tree/ws_behaviotree/devel/setup.bash
```

## Lancer une simulation de base

> Les commandes qui suivent sont citées dans la documentation fournie par le CESI. Si l'image à été mise à jour, il se peut que ces commandes ne soient plus fonctionnelles ou que les chemins des fichiers soient erronés.

```bash
sudo service apache2 start
sudo cesi_gazebo_very_high_speed
source /opt/pal/gallium/setup.bash
source /usr/share/cesi-tiago-package/behaviour_tree/ws_behaviotree/devel/setup.bash
roslaunch tiago_dual_187_gazebo tiago_dual_navigation.launch webgui:=true
```

Gazebo et RViz devrait se lancer. Vous pouvez accéder à l'interface de controle de Tiago en utilisant: [localhost](http://localhost/). Les identifiants sont `pal` et `pal`.

## Se connecter avec le vrai robot TIAGO

Pour accéder à l'environnement de TIAGO depuis Docker, nous recommandons de créer un second container dont les paramètres seront légèrements différents. Dans la suite de ces explications, nous considererons le second conteneur comme `tiago-real`.

Dans un premier temps, il est important sur la **machine Linux**, de saisir la commande:
```bash
xhost +local:
```
Cela permettra au nouveau conteneur de pouvoir continuer à executer des interfaces GUI.
> Si votre Linux est installé avec un Machine Virtuelle, nous recommandons d'installer deux cartes réseaux à celle-ci. La première en mode connexion "NAT" sur votre carte Wifi. Elle permettra l'accès à internet au sein de la VM. La seconde en "Accès par pont" sur votre carte réseau Ethernet. Elle permettra une connexion avec le robot.

Ensuite lancez un second conteneur qui sera dédié au connexion avec le robot TIAGO.
```bash
docker run \
  -it \
  -d \
  -u user \
  -e DISPLAY=$DISPLAY \
  --network host \
  --privileged \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  --name="tiago-real" \
  docker_tiagocesi_image_full \
  bash
```

Au sein de ce docker il faut répéter les opérations précédentes : 
```bash
docker exec -it tiago-real bash
```
```bash
nano ~/.bashrc
```
```bash
source /opt/pal/gallium/setup.bash
source /usr/share/cesi-tiago-package/behaviour_tree/ws_behaviotree/devel/setup.bash
```

Une fois les opérations précédentes effectuée, nous allons pouvoir préparer la machine virtuelle pour se connecter avec le robot. Pour cela il va falloir accéder temporairement au conteneur avec les droits root.
```bash
docker exec -it -u root tiago-real bash
```
```bash
nano /etc/hosts
```
Rajoutez à la fin du fichier:
> Le nom du robot est de la forme `tiago-<Numéro de série>c`. Ce numéro de série se trouve à l'arrière du robot sur une étiquette.
```bash
10.68.0.1   tiago-202c # Robot Tiago de Bordeaux
```

Reconnectez vous sur le docker en mode utilisateur. Nous allons modifier le fichier `.bashrc` pour que celui-ci conserve la connexion avec Tiago.
```bash
nano ~/.bashrc
```
Ajoutez
```bash
export ROS_MASTER_URI=http://tiago-202c:11311 # Le nom du robot est à ajuster selon le votre
export ROS_IP=10.68.0.XX # A ajuster selon votre adresse IP dans le conteneur.
```
> Il est important que votre conteneur est une adresse IP dans la plage de celle de TIAGO. Ceci est normalement du au paramètre `--network` de la commande pour la création du conteneur.

Pour vérifier le bon fonctionnement, executez la commande `rostopic list`. Si vous obtenez la liste des TOPICS, vous êtes correctement connectés à TIAGO.
