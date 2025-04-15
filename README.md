# Installation du WorkSpace ROS2

Ce WorkSpace se base sur **ROS2 Humble** sur l'OS **Ubuntu 22.04**.

## 1 - Git Clone

Il faurt d'abord git clone le repo ROS2 via cette commande :
```
git clone https://github.com/jikan-xyz/go2-ros2
```

## 2 - Installer les Dépendances CycloneDDS
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
```
sudo apt install ros-humble-rosidl-generator-dds-idl
```

## 3 - Compiler CycloneDDS

Pour compiler CycloneDDS il **ne faut pas** que ROS2 soit source dans le terminal.
S'il se source par défaut, il faut éditer .bashrc via cette commande : 
```
sudo nano ~/.bashrc
```
Et commenter la ligne :
```
source /opt/ros/humble/setup.bash 
```
Résultat final :
```
# source /opt/ros/humble/setup.bash 
```


D'abord aller dans le src de CycloneDDS : 
```
cd ~/go2-ros2/src/cyclonedds_ws/src
```

Cloner ces deux repo :
```
git clone https://github.com/ros2/rmw_cyclonedds -b humble
```
```
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
```

Une fois cloné, retourner un dossier en arrière (~/go2-ros2/src/cyclonedds_ws) :
```
cd ..
```

Et compiler CycloneDDS :
```
colcon build --packages-select cyclonedds
```

## 4 - Compiler le WorkSpace

Nous avons maintenant besoin de source ROS2.
```
source /opt/ros/humble/setup.bash 
```
Ne pas oublier de le remettre dans le fichier bashrc si il l'était avant.


Dans le dossier root du WorkSpace (~/go2-ros2), installer toutes les dépendances nécessaires via ces 3 commandes : 
```
sudo rosdep init
```
```
rosdep update
```
```
rosdep install --from-paths src --ignore-src -r -y
```

Puis lancer la compilation : 
```
colcon build
```

## 5 - Configurer l'interface réseau

Il faut d'abord configurer les paramètres IPV4 d'Ubuntu sur le port ethernet de la manière suivante : 

**IPV4 Methods** : __Manual__

|Intitulé| Valeur |
|--|--|
| Adress | 192.168.123.18 |
| Netmask | 255.255.255.0 |
| Gateway | 192.168.123.1 |


Ces valeurs sont les valeurs utilisées par la Jetson d'origine du robot.
Cela permet de recevoir toutes les informations qu'envoie le robot.


En suite il va falloir connaîte le nom de l'interface réseau, pour cela il existe cette commande : 
```
ifconfig
```

Chercher le nom de votre interface réseau (dans mon cas **enp0s31f6**).
Puis modifier le fichier **setup.sh** avec la bonne valeur de votre interface.
```
nano setup.sh
```
```
#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/humble/setup.bash
source $HOME/go2-ros2/src/cyclonedds_ws/install/setup.bash
echo "Setup custom go2 ros2 environment"
source $HOME/go2-ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="NOM_INTERFACE" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

Ne pas oublier de modifier **NOM_INTERFACE** avec votre nom.

La valeur par défaut de **NOM_INTERFACE** est **enp0s31f6** conformément au laptop utilisé pendant le développement de ce WorkSpace.


## 6 - Connexion et Test

Maintenant il faut source le fichier **setup.sh** du WorkSpace :
```
source ~/go2-ros2/setup.sh
```


On peut ajouter cette commande avec le chemin complet du fichier dans **.bashrc** pour ne plus devoir cette commande à l'avenir.


Et maintenant nous pouvons exécuter une commande basique du robot telle qu'**Hello** pour qu'il fasse un bonjour de la patte :
```
ros2 run go2_basics hello
```