### Installation de Docker

La première étape consiste à installer Docker sur votre machine. Vous pouvez vous référer à la [documentation officielle](https://docs.docker.com/engine/install/ubuntu/) de Docker.

#### Supprimer les paquets conflictuels
La première étape consiste à supprimer touts les packages pouvant entrer en conflit avec la nouvelle installation :
```bash
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
```

#### Installer docker avec `apt`

1. Mettre en place le repository Docker
```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

2. Installer les paquets Docker
```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

3. Vérifier la bonne installation
```bash
sudo docker run hello-world
```

4. Créer le groupe `docker`
```bash
sudo groupadd docker
```

5. Ajouter votre utilisateur au groupe `docker`
```bash
sudo usermod -aG docker $USER
```

Redémarrer ensuite votre machine.