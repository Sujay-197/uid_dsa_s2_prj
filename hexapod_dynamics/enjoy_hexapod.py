from stable_baselines3 import PPO
from hexapod_env import HexapodEnv

def view():
    env = HexapodEnv(render_mode='human')
    
    
    # model_path = "models/10000" 
    
    
    print("This script is for evaluating the trained model.")
    print("Please provide the path to the model zip file.")
    model_path = input("Model path (e.g., models/10000.zip): ")
    
    if not model_path:
        print("No model path provided. Exiting.")
        return

    model = PPO.load(model_path)

    obs, info = env.reset()
    while True:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        
        if terminated or truncated:
            obs, info = env.reset()

if __name__ == "__main__":
    view()
