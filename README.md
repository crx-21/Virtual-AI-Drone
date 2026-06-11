# Virtual-AI-Drone

 Roadmap:

- [ ] Create a mod in order to fetch more information out of the game (Speed, X, Y, Z, Pitch, Yaw, Roll, Crash Status)
- [ ] Decoder for the mod info
- [ ] Setting up environment for training to begin. (Implementing virtual controllers for ai to use, Core Functions etc..)
- [ ] Designing the reward system and implementing it.
- [ ] Set Milestones.md so that I can keep track AI progress.
- [ ] Release final trained model to github.
- [ ] Setup an application so that other people can run this AI in their game and watch it fly.

---
# System Design

### Requirements and Constraints:

  Constraints:
  - Scaling to real life might be troublesome as the data in the game might not be accuarate to real life.
  - Training is limited to the Hardware i currently have.
  - Latency
  - Extracting Game Data accurately.
  - AI might get used only to the game's images so running it in other games/real life might be difficult.
  - Running the AI with telemetry only will restrain the drone and make it only hover mostly. *(Possible solution: Computer Vision on Virtual LIDAR)*
    
    *(Must find solutions to these problems)*
  
  Functional Requirements: 
  - Drone must be fully controlled by AI while AI is running.
  - AI must act really fast and press buttons in ms. (low latency)
  - Mod should provide details in console about flight data, drone status, etc...
  - The mod must provide accurate data to the AI.
   
 > Future Scope (If this repo gains traction): Create an application that can be downloaded by anyone. User presses a start button and the game starts, picks a random map and the AI starts flying. Real time game  telemetrics are simplified and shown by the application.

  Non-Functional Requirements:
  - Scale from the game world to real life.

### High-Level Arhitecture:
 
![Diagram](https://github.com/crx-21/Virtual-AI-Drone/blob/0abbf03eaf6b0347b870f15d6764bd5445f334e2/images/Diagram.png)

### Core Architectural Layers:

1. Telemetry Capture Mod: Captures the data from the game and lends it to the python listener.
2. Python Listener: Interprets the data and works hand-in-hand with the AI so it can make decisions.
3. AI Brain: Controls the drone based on the received data from the python listener and takes actions.




    
