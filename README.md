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
  
  Functional Requirements: 
  
  - Users can open and run the application to test the AI.
  - Users can stop the AI at any point in time.
  - Drone must be fully controlled by AI while AI is running.
  - AI must act really fast and press buttons in ms. (low latency)
  - Mod should provide details in console about flight data, drone status, etc...
  - The mod must provide accuarate data to the AI.

   *(Further down the line I should scale this to be seen by the user in the application)*

  Non-Functional Requirements:
  - Scale from the game world to real life.

 High-Level Arhitecture:



    
