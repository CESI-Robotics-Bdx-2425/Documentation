# Guide d'Installation de Docker sur Ubuntu

Ce guide détaille l'installation complète de Docker Engine sur Ubuntu, en suivant les bonnes pratiques recommandées.

## Prérequis

- Un système Ubuntu à jour
- Un utilisateur avec les droits sudo
- Une connexion internet stable

## 1. Préparation du système

### 1.1 Mise à jour du système

Avant de commencer l'installation, assurez-vous que votre système est à jour :

```bash
sudo apt-get update
sudo apt-get upgrade -y
```

### 1.2 Suppression des versions antérieures

Si vous aviez déjà Docker installé, supprimez tous les packages pouvant créer des conflits :

```bash
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
```

## 2. Installation de Docker

### 2.1 Configuration du référentiel

1. Installez les packages nécessaires pour utiliser le référentiel HTTPS :

```bash
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release
```

2. Ajoutez la clé GPG officielle de Docker :

```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

3. Configurez le référentiel :

```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
```

### 2.2 Installation des packages Docker

Installez la dernière version de Docker Engine et ses dépendances :

```bash
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

## 3. Configuration post-installation

### 3.1 Vérification de l'installation

Vérifiez que Docker fonctionne correctement en exécutant l'image hello-world :

```bash
sudo docker run hello-world
```

Si l'installation est réussie, vous verrez un message de confirmation indiquant que Docker fonctionne correctement.

### 3.2 Configuration des permissions

Pour éviter d'avoir à utiliser sudo à chaque commande Docker :

1. Créez le groupe docker s'il n'existe pas déjà :
```bash
sudo groupadd docker
```

2. Ajoutez votre utilisateur au groupe docker :
```bash
sudo usermod -aG docker $USER
```

3. Activez les modifications :
```bash
newgrp docker
```

4. Vérifiez que vous pouvez exécuter Docker sans sudo :
```bash
docker run hello-world
```

### 3.3 Configuration du démarrage automatique

Configurez Docker pour qu'il démarre automatiquement au démarrage du système :

```bash
sudo systemctl enable docker.service
sudo systemctl enable containerd.service
```

## 4. Dépannage courant

### 4.1 Erreur de permission

Si vous rencontrez une erreur de permission du type :
```
permission denied while trying to connect to the Docker daemon socket
```

Solutions possibles :
1. Vérifiez que vous avez redémarré votre session après avoir ajouté votre utilisateur au groupe docker
2. Exécutez : `sudo chmod 666 /var/run/docker.sock`

### 4.2 Problèmes de réseau Docker

Si vous rencontrez des problèmes de réseau :
```bash
# Redémarrez le service Docker
sudo systemctl restart docker

# Si le problème persiste, réinitialisez le réseau Docker
docker network prune
```

## 5. Commandes utiles

Voici quelques commandes Docker essentielles pour débuter :

```bash
# Vérifier la version de Docker
docker --version

# Afficher les informations système Docker
docker info

# Lister les conteneurs en cours d'exécution
docker ps

# Lister tous les conteneurs (y compris ceux arrêtés)
docker ps -a

# Lister les images Docker
docker images
```