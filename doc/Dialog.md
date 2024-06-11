# Conversation Dialog for Chimera Robot

## Movement Dialogue - Direction and Distance

- **User**: "Move {forward, backward, left, right} {distance} {meters, centimeters, millimeters}"

**Accept Movement Command:**

- **Alexa**: "Okay, I will move {forward, backward, left, right} {distance} {meters, centimeters, millimeters}"
- **API**: `move {forward, backward, left, right} {distance} {meters, centimeters, millimeters}`

**Reject Movement Command:**

- **Alexa**: "I'm sorry, I can't move {forward, backward, left, right} {distance} {meters, centimeters, millimeters} right now because {provide reason}."

## Movement Dialogue - Go To

- **User**: "Go to {location}" / "Navigate to {location}" / "Move there + {point-at-location}"

**Accept Movement Command:**

- **Alexa**: "Okay, I will navigate to {location}" / "I will move to {location}" / "I will go there"

**Reject Movement Command:**

- **Alexa**: "I'm sorry, I can't go to {location} right now because {provide reason}."

## Movement Dialogue - Stop

- **User**: "Stop" / "Halt" / "Cease"

**Accept Movement Command:**

- **Alexa**: "Okay, I will stop moving"

**Reject Movement Command:**

- **Alexa**: "I'm sorry, I can't stop moving right now because {provide reason}."

## Pickup Dialogue - Bring Object to User

- **User**: "Pick me up the {object}" / "Grab me the {object}" / "Take me the {object}"
- **User**: "Bring {object} to me" / "Take {object} to me"

**Accept Pickup Command:**

- **Alexa**: "Okay, I will bring you the {object}" / "I will take you the {object}"

**Extra Info Pickup Command:**

- **Alexa**: "Where is the {object} located?"
- **User**: "The {object} is located at {location}" / "There + {point-at-location}" / "It is at {location}"
- **Alexa**: "Okay, I will bring you the {object} from {location}" / "I will take you the {object} from {location}"

**Reject Pickup Command:**

- **Alexa**: "I'm sorry, I can't bring you the {object} right now because {provide reason}."

## Pickup Dialogue - Bring Object to Location

- **User**: "Move {object} to {location}" / "Take {object} to {location}" / "Bring {object} to {location}"
- **User**: "Move {object} from {location} to {location}" / "Take {object} from {location} to {location}"

**Accept Pickup Command:**

- **Alexa**: "Okay, I will bring {object} to {location}" / "I will take {object} to {location}"
- **Alexa**: "Okay, I will move {object} from {location} to {location}"

**Extra Info Pickup Command:**

- **Alexa**: "Where is the {object} located?"
- **User**: "The {object} is located at {location}" / "There + {point-at-location}" / "It is at {location}"
- **Alexa**: "Okay, I will bring {object} to {location} from {location}" / "I will take {object} to {location} from {location}"

**Reject Pickup Command:**

- **Alexa**: "I'm sorry, I can't bring {object} to {location} right now because {provide reason}."
- **Alexa**: "I'm sorry, I can't move {object} from {location} to {location} right now because {provide reason}."

## Execute Task Dialogue

- **User**: "Execute task {task-name}" / "Perform task {task-name}" / "Run task {task-name}"
- **User**: "Execute {task-name}" / "Perform {task-name}" / "Run {task-name}"

**Accept Task Command:**

- **Alexa**: "Okay, I will execute task {task-name}" / "I will perform task {task-name}" / "I will run task {task-name}"

**Reject Task Command:**

- **Alexa**: "I'm sorry, I can't execute task {task-name} right now because {provide reason}."
