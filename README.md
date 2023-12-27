# Alexa Conversation

Package that allows communication between ROS2 and Alexa, implementing two different communication channels:

- User &rarr; ROS2: through a custom Alexa Conversation Skill that communicates with a local server in azure functions.
- ROS2 &rarr; User: using unsolicited Text-To-Speech of Node-RED block that use Alexa-API

## Requirements

- Ubuntu 20.04+
- Python 3.8.10
- ROS2 Foxy
- Anaconda / Miniconda

## Installation

### Prerequisites

- Install ROS2 Foxy: [Ubuntu Guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

        sudo apt install ros-foxy-desktop python3-argcomplete

- Install `miniconda`: [Official Guide](https://docs.conda.io/en/main/miniconda.html)

- Create a `conda` environment with `python=3.8.10` and `openssl=3.0.9`:

        conda create -n alexa_conversation_env python=3.8.10 openssl=3.0.9
        conda activate alexa_conversation_env

- Install Python Requirements:

        pip install -r ../path/to/this/repo/requirements.txt

### Microsoft Azure

- Install `curl` if not already installed:

        sudo apt install curl

- Install the Microsoft signing key:

        curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
        sudo mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg

        sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/microsoft-ubuntu-$(lsb_release -cs)-prod $(lsb_release -cs) main" > /etc/apt/sources.list.d/dotnetdev.list'

- Check `/etc/apt/sources.list.d/dotnetdev.list` for the correct distribution name. For example, if you are using Ubuntu 20.04, the distribution name is `focal`.

            Linux Distribution      Version
            Debian 11               bullseye
            Debian 10               buster
            Debian 9                stretch
            Ubuntu 20.04            focal
            Ubuntu 19.04            disco
            Ubuntu 18.10            cosmic
            Ubuntu 18.04            bionic
            Ubuntu 17.04            zesty
            Ubuntu 16.04            xenial

- Install the `Azure SDK`:

        sudo apt-get update
        sudo apt-get install azure-functions-core-tools-4

### Node RED

- Install `Node.js`, follow the [Official Guide](https://github.com/nodesource/distributions):

        sudo apt-get update
        sudo apt-get install -y ca-certificates curl gnupg
        sudo mkdir -p /etc/apt/keyrings
        curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg

        NODE_MAJOR=20
        echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_$NODE_MAJOR.x nodistro main" | sudo tee /etc/apt/sources.list.d/nodesource.list

        sudo apt-get update
        sudo apt-get install nodejs

- To install `Node-RED` you can use the `npm` command that comes with `node.js`:

        sudo apt install npm
        sudo npm install -g --unsafe-perm node-red

- Install `node-red-contrib-alexa-remote2-applestrudel` and `node-red-ros2-plugin`:

        node-red -u ~/.node-red-2
        cd ~/.node-red-2
        npm install node-red-contrib-alexa-remote2-applestrudel 
        npm install node-red-ros2-plugin

- Install `ask-sdk-core` e `webservice`:

        cd ~/.node-red-2
        npm install --save ask-sdk
        pip install ask-sdk-webservice-support
        pip install ask-sdk-core

#### Configuration of `node-red-contrib-alexa-remote2-applestrudel`

- Start `Node-RED`:

        node-red -u ~/.node-red-2

- Add the `Alexa Init` Node to the Node-RED flow

- Edit the `Alexa Init` Node properties to add your Alexa Account:

  - Account &rarr; Add new Alexa Account &rarr; Edit (Pencil Icon):

  - Global Config:

        Name: <Custom_Name>
        Auth Method: Proxy
        This IP: localhost
        Port : 3456
        File Path : /home/<user>/AuthFile

  - For Italian language:

        Service Host: alexa.amazon.it
        Page:amazon.it
        Language: it-IT

  - For English language:

        Service Host: alexa.amazon.com
        Page:amazon.com
        Language: en-US

- Click on the `Add` button and follow the instructions to add your account (you will be redirected to the Amazon Alexa login page)

- Click on the `Deploy` button to save the configuration

#### Configuration of `node-red-ros2-plugin`

- Install dependencies:

        sudo apt-get install libyaml-cpp-dev libboost-program-options-dev \
         libwebsocketpp-dev libboost-system-dev libboost-dev libssl-dev \
         libcurlpp-dev libasio-dev libcurl4-openssl-dev git

- Clone `Integration-Service` Packages: [Official Guide](https://github.com/eProsima/node-red-ros2-plugin)

        cd ~/colcon_ws/src
        mkdir Node-RED && cd "$_"
        git clone https://github.com/eProsima/Integration-Service.git
        git clone https://github.com/eProsima/WebSocket-SH.git
        git clone https://github.com/eProsima/ROS2-SH.git
        git clone https://github.com/eProsima/FIWARE-SH.git

- Build `Integration-Service` Packages:

        cd ~/colcon_ws
        source /opt/ros/foxy/setup.bash
        colcon build --cmake-args -DIS_ROS2_SH_MODE=DYNAMIC

- Source `Integration-Service` Packages:

        . ~/colcon_ws/install/setup.bash

- Add `export ROS_DOMAIN_ID=10` to `~/.bashrc` to set the Domain ID to `10`:

        echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc

### Ngrok

- Install Ngrok

    sudo apt update
    sudo apt install snapd
    sudo snap install ngrok

- Add your Token in the `ngrok.yml` file in the `ngrok` folder

## Azure Configuration

- To create a New Project and a Skill Template run the following commands:

        func init MyFunctionProj
        cd MyFunctionProj
        func new --template "Http Trigger" --name MySkill

- Edit `local.settings.json` by adding your `AlexaSkillID`:

        {
            "IsEncrypted": false,
            "Values": {
            "FUNCTIONS_WORKER_RUNTIME": "python",
            "AzureWebJobsStorage": "",
            "AlexaSkillID": "amzn1.ask.skill.96421c1e-fb2b-44eb-80a3-f9097c146ffd",
            "UpdatesURL": "https://azure.microsoft.com/updates/feed/"
            }
        }

- Azure extension for VSCode:

  Launch VS Code Quick Open (Ctrl+P), paste the following command, and press enter:

      ext install ms-azuretools.vscode-azurefunctions

## Node-RED Flows Configuration

Example: Import `example_flows.json` files in the `scripts` folder to Node-RED to have a working example of the flows.

- Use the `Alexa Init` Node to add your Alexa Account
- Use the `Alexa Routine` Node to play voice messages

- Use the `ROS2 Type` Node linked to the `ROS2 Subscriber` Node to subscribe to ROS2 topics.
- Use the `ROS2 Inject` Node linked to the `ROS2 Type` and `ROS2 Publisher` Nodes to publish to ROS2 topics.
- Configure the `ROS2 Type` Node to match the message type.
- Configure the `ROS2 Subscriber` or `ROS2 Publisher` Node to match the topic name.
- Set the Domain ID of the `ROS2 Subscriber` or `ROS2 Publisher` Node to the same value of the ROS2 Bridge (Equal to `export ROS_DOMAIN_ID`).

## Launch Instructions

- Activate the `conda` environment:

        conda activate alexa_conversation_env

- Remember to source ROS2 and export the Domain ID (if not in `~/.bashrc`):

        source /opt/ros/foxy/setup.bash
        . ~/colcon_ws/install/setup.bash
        export ROS_DOMAIN_ID=10

### Launch Skill Back-End

- Launch `skill_backend_launcher`:

        ros2 launch alexa_voice_control skill_backend.launch.py

- Launch only Node-RED:

        ros2 launch alexa_voice_control node_red.launch.py

- Run Azure Functions locally:

        cd ../AzureFunction
        func start

### Configure Alexa Skill End-Point

- Change Skill Endpoint in Developer Console with the one given by `ngrok` adding `/api/"SkillName"` at the end:

        https://2c13-151-38-117-194.eu.ngrok.io/api/SkillBackend

- Change the Endpoint Sub-domain:

        My development endpoint is a sub-domain of a domain that has a wildcard certificate from a certificate authority
