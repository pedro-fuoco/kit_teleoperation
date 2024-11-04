# kit_teleoperation

Esse é o pacote ROS do Kit de Robótica dedicado à teleoperação. Diferentemente dos demais pacotes desenvolvidos para o Kit de Robótica, esse pacote deve ser instalado fora da raspberry pi do Kit, servindo para receber e enviar dados para o Kit diretamente de um computador conectado na mesma rede que ela.

## Dependencias

Clone esse pacote dentro do seu ROS workspace, dentro da pasta src. Para isso, substitua o endereço e nome do seu ROS workspace e copie os comandos abaixo:
```bash
cd ~/ros2_ws/src/
git clone git@github.com:pedro-fuoco/kit_teleoperation.git
```

### Rosdep
Esse pacote foi desenvolvido para a versão ROS 2 Jazzy. Uma vez que o ROS estiver devidamente instalado e inicializado, as dependencias especificas desse pacote podem ser instaladas através do rosdep.

Rosdep é um meta-package manager, utilizado para facilitar a instalação de dependencias. Apesar do nome com base histórica, essa ferramenta se tornou independente do ROS e deve ser instalada separadamente:

```bash
sudo apt install python3-rosdep
```

Uma vez instalado, ele deve ser inicializado e atualizado:

```bash
sudo rosdep init
rosdep update
```

Por fim, para esse repositorio, vamos utilizar o rosdep para instalar as dependencias listadas no arquivo `package.xml`. Para isso, substitua o endereço e nome do seu ROS workspace e copie os comandos abaixo:
```bash
cd ~/ros2_ws/src/kit_teleoperation
rosdep install --from-paths . -y --ignore-src
```

### ROS DOMAIN ID
Para que um computador consiga se comunicar com outro computador com ROS 2 na mesma rede, ambos precisam selecionar o mesmo ROS DOMAIN ID. Para isso, basta que esse comando seja feito em AMBOS os terminais, com {selecione seu identificador} sendo um número de 0 até 255.
```bash
export ROS_DOMAIN_ID={selecione seu identificador}
# Por exemplo, o seguinte comando deve ser rodado no terminal do seu computador E da raspberry pi do seu kit:
# export ROS_DOMAIN_ID=1
```

Para testar a conexão, basta inicializar qualquer node ROS na raspberry Pi e rodar esse comando no terminal do seu computador:
```bash
ros2 topic list
```
Se a configuração foi um sucesso, você deve enchergar os tópicos publicados pela raspberry pi no seu computador.


### Colcon
Inicialize o seu ROS workspace e compile ele utilizando a ferramenta `colcon`. Mais uma vez, é necessario substituir o endereço e nome do seu próprio ROS workspace abaixo:
```bash
cd ~/ros2_ws/
source install/setup.sh
colcon build --packages-select kit_teleoperation
```
Fique de olho em possiveis erros nesse processo e realize debug se necessario.

## Launch
TODO