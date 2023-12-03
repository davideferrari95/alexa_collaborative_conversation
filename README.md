# Alexa Conversation

Package that allows communication between ROS2 and Alexa, implementing two different communication channels:

- User → ROS2: through a custom Alexa Conversation Skill that communicates with a local server in azure functions.
- ROS2 → User: using unsolicited Text-To-Speech of Node-RED block that use Alexa-API

## Requirements

- Ubuntu 20.04+
- Python 3.8.10
- ROS2 Foxy
- ROS2-Bridge
- Azure 4.0+
- Node-RED
- ngrok
- ask-sdk

## Installation

### Prerequisites

- Install ROS2 Foxy: [Ubuntu Guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

        sudo apt install ros-foxy-desktop python3-argcomplete

- Install `rosbridge` suite:

        sudo apt-get install ros-foxy-rosbridge-server

- Install `miniconda`: [Official Guide](https://docs.conda.io/en/main/miniconda.html)

- Create a `conda` environment with `openssl=3.0.9`:

        conda create --name alexa_conversation_env python=3.8.10 openssl=3.0.9

- Install dependencies in `requirements.txt`:

        pip install -r requirements.txt

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

- Install the Azure SDK:

        sudo apt-get update
        sudo apt-get install azure-functions-core-tools-4

### ask-sdk-core e webservice

- install ask-sdk-core e webservice:

        npm install --save ask-sdk
        pip install ask-sdk-webservice-support
        pip install ask-sdk-core

### Node RED

- To install Node-RED you can use the `npm` command that comes with node.js:

        sudo apt-get install npm
        sudo npm install -g --unsafe-perm node-red

- in Node RED → manage palet → install:

  - ROS → Node RED:

        node-red-contrib-ros2

  - Node RED → Alexa:

        node-red-contrib-alexa-remote2-applestrudel

- You can install the Node-RED packages also using `npm`:

        npm install node-red-contrib-ros2
        npm install node-red-contrib-alexa-remote2-applestrudel

### Ngrok

    sudo apt update
    sudo apt install snapd
    sudo snap install ngrok

## Configuration

### node-red-contrib-ros

- in sub/pub node → ROS2 SERVER → add new ROS2 server:

        url: ws://localhost:9091/
        Topic: name_topic

### node-red-contrib-alexa-remote2-applestrudel

- Alexa initialize node → Account → Add new Alexa Account:

  - Global Config:

        Name: Name
        Auth Method: Proxy
        This IP: IP_ADDRESS
        Port : 3456
        File Path : AuthFile

  - For Italian language:

        Service Host: alexa.amazon.it
        Page:amazon.it
        Language: it-IT

  - For English language:

        Service Host: alexa.amazon.com
        Page:amazon.com
        Language: en-US

- Option → initialize

### Azure

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

## Running

- Launch `skill_backend_launcher`:

        ros2 launch alexa_conversation skill_backend_launch.py

- Run Azure Functions locally:

        cd ../AzureFunction
        func start

- Change Skill Endpoint in Developer Console with the one given by `ngrok` adding `/api/"SkillName"` at the end:

        https://2c13-151-38-117-194.eu.ngrok.io/api/SkillBackend
