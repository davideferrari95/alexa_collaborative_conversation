# Alexa_conversation_voice_control

Package that allows communication between ros and alexa, implementing two different communication channels:

- User  →  ROS:  through a custom Alexa Conversation Skill that communicates with a local server in azure functions.
- ROS   →  User: using unsolicited Text-To-Speech  of NodeRED block that use AlexaAPI

## Requirements

* Ubuntu 20.04+
* Python 3.8+
* ROS Noetic+
* ROS bridge
* azure 4.0+
* NodeRED
* ngrok
* ask-sdk

## Installation

### Azure:
*
```
sudo apt install curl
curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
```

```    
sudo mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg
```

```
sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/microsoft-ubuntu-$(lsb_release -cs)-prod $(lsb_release -cs) main" > /etc/apt/sources.list.d/dotnetdev.list'
```

```
Controllare il /etc/apt/sources.list.d/dotnetdev.list file per una delle stringhe di versione Linux appropriate elencate di seguito:

            Distribuzione di Linux 	Versione
            Debian 11 	bullseye
            Debian 10 	buster
            Debian 9 	stretch
            Ubuntu 20.04 	focal
            Ubuntu 19.04 	disco
            Ubuntu 18.10 	cosmic
            Ubuntu 18.04 	bionic
            Ubuntu 17.04 	zesty
            Ubuntu 16.04/Linux Mint 18 	xenial

sudo apt-get update
sudo apt-get install azure-functions-core-tools-4
```



### NodeRED
* To install Node-RED you can use the   npm        command that comes with node.js:
    ```
    sudo apt-get install npm
    sudo npm install -g --unsafe-perm node-red
    ```
    in node red  → manage palet → install:
    
		1- node-red-contrib-alexa-remote2-applestrudel #comunicazione ros to nodeRED

		
        2- node-red-contrib-ros #comunicazione nodeRED to alexa
        or in bash:
    ```
    npm install node-red-contrib-ros
    npm install node-red-contrib-alexa-remote2-applestrudel 
    ```

### Ros bridge
* install rosbridge suite:
    ```
    sudo apt-get install ros-noetic-rosbridge-server
    ```

###   ask-sdk-core e webservice
*  install ask-sdk-core e webservice:
    ```
    npm install --save ask-sdk
    pip install ask-sdk-webservice-support
    pip install ask-sdk-core
    ``` 


### Ngrok
```
sudo apt update
sudo apt install snapd
sudo snap install ngrok
```

### Configuration NodeRED contrib ROS
  * in sub/pub node:
        ROS SERVER:

    add new ros server :
    ```
    url: ws://localhost:9091/
	Topic: name_topic
    ```

### Configuration NodeRED contrib Alexa Remote 2:
 * in Alexa inizialize node:

    1-Account → Add new Alexa Account:
    ```
                Name: Nome
			    Auth Method: Proxy
			    This IP: IP_ADDRESS
			    Port : 3456
			    File Path : AuthFile
                
			    Service Host: alexa.amazon.it
			    Page:amazon.it
			    Language: it-IT
    ```
    2-Option: initialize

 ### Configuration Azure:
  

 * 1- Move to project dir,In the terminal window or from a command prompt, run the following command to create the project and local Git repository:
    ```
    func init MyFunctionProj
	func new --template "Http Trigger" --name MyHttpTrigger
    ```
 * 2-in local.settings.json:
    ```
    {
	    "IsEncrypted": false,
		"Values": {
		"FUNCTIONS_WORKER_RUNTIME": "python",
		"AzureWebJobsStorage": "",
		"AlexaSkillID": "amzn1.ask.skill.96421c1e-fb2b-44eb-80a3-f9097c146ffd", #ID skill alexa
		"UpdatesURL": "https://azure.microsoft.com/updates/feed/"
		}
	}
    ```
  * 3- In requirmente.txt:
    ```
    azure-functions
    beautifulsoup4
    requests
    service_identity
    ask-sdk-core
    ask-sdk-webservice-support
    rospkg
    rospy
    ```        
 * 4- Run functions locally:
    ```
	func start    
    ```
  * 5-azure extension for vscode:
	Launch VS Code Quick Open (Ctrl+P), paste the following command, and press enter:
    ```
	ext install ms-azuretools.vscode-azurefunctions
    ```

 ## Configure Alexa skill Endopoint
 * change skill endpoint: 
        
        https://2c13-151-38-117-194.eu.ngrok.io/api/HttpTriggerAlexaROS