Repo contenant mon apprentissage de ROS2 et Gazebo, ainsi que du premier projet pratique.

Gazebo fortress a été utilisé, étant la version recommandée pour ROS2 Jazzy.

Gazebo utilise le format `SDF` pour décrire un robot, format qui lui est spécifique. Il existe également le format `URDF` pour Universal Robot Description Format, qui lui est plus général à ROS. Il est possible d'automatiquement générer un SDF pour Gazebo à partir d'un URDF. Je vais donc apprendre à faire une description en URDF.

Tutoriels suivies :
- [Beginner: CLI tools - ROS2 Docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)
- [Beginner: Client libraries - ROS2 Docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html)
- [URDF - ROS2 Docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Gazebo Tutorials - Gazebo Docs](https://gazebosim.org/docs/latest/tutorials/)

Connaissances apprises :
- Nodes, topics, services, à la fois théorique et mise en pratique.
- Action (théorique, non mis en application)
- Usage de l'API ROS2 en C++
- Le CLI de ROS2 (echo, bag, rosdoctor, etc...)
- L'outil `rqt`
- L'usage d'un `.devcontainer` pour avoir le même environnement ROS2 de dev depuis n'importe quelle machine pouvant faire tourner Docker, y compris avec le fonctionnement du GUI. (dev fait sur Fedora)
- Première découverte des plugins (à approfondir)
- Rosdep pour la gestion des dépendances
- `URDF` ainsi que `Xacro` pour la description physique de robots
- Rviz pour la visualisation de simulations

Choses à approfondir :
- [ ] Apprendre CMake, afin de ne pas simplement deviner ce que font les fichiers `CMakeLists.txt`.
- [ ] Qu'est ce que QoS ? (Quality of Service ?)
- [ ] Le lintage propre, et les testes (les actions github du template)
- [ ] Apprendre `tf2`
- [ ] Export de OnShape à URDF ([outil](https://onshape-to-robot.readthedocs.io/en/latest/))
- [ ] La commande `source`