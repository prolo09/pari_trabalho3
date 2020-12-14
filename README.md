# Turtlebot3 first steps
###### Trabalho 3 da unidade curricular de projecto de automação e robótica do curso de engenharía mecânica da universidade de aveiro

#Indice
 - [Instalação](#instalao)
 - [Gazebo](#criao-do-ambiente-simulado)
 - [Rviz](#visualizao-rviz)
 - [Spawn](#spawn-do-robot)
 - [Controlo](#comando-do-robot)
 
# Instalação
Certifique-se que ja tem o ROS instalado, pode consultar informação para o fazer [aqui](http://wiki.ros.org/ROS/Installation).

Depois de ter o ROS instalado pode fazer o download do repositório. Lá dentro encontrará dois packages Ros.
O Primeiro Package chama-se **p_g5_description** e o segundo **p_g5_bringup**, a ordem pela qual aparecem não é relevante.

# Criação do ambiente simulado
Apos obter este dois packages pode começar a criar o cenário para o seu robot. Abre um terminal e introduza:
        
        roslaunch p_g5_bringup bringup_gazebo.launch

Este comando fará despoletar o simulador gazebo, já carregado com a casa do turtle bot.
       
# Visualização Rviz
Apos se ter usado o simulador gazebo é possivel tambem usar o Rviz, software de vizualização 3d, que permitirá ver as
imagens captadas pelo robo, scans captados pelo robot, tranformações captadas pelo robo, etc... basicamente
poder fazer a amostragem dos dados. Para tal abrir o terminal e digitar:

    roslaunch p_g5_bringup visualize.launch
    
# Spawn do robot
Tendo agora todos os ambientes gráficos criados podem finalmente criar o nosso robot. Para isso no terminal introduzir:
    
    roslaunch p_g5_bringup spawn.launch player_name:=[nome] player_color:=[cor_robot] base_color:=[cor_gazebo] scan_color:=[cor_gazebo]

**cor_gazebo:** deve-se usar a convenção de cores do gazebo, [exemplos](http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials)

**cor**
 - black
 - DarkGrey
 - FlatBlack
 - light_black
 - Blue
 - Green
 - Grey
 - Orange
 - brown
 - red
 - Red
 - white
 - White
 
# Comando do robot
Agora para comando o robot podemos usar o controlador do ROS. Para tal podemos usar este comando no terminal:
    
        roslaunch p_g5_bringup teleop.launch player_name:=[nome]
      
    