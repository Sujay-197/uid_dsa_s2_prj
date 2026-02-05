import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from hexapod_env import HexapodEnv
import os
import time

def train():
    log_dir = "logs"
    models_dir = "models"
    
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    if not os.path.exists(models_dir):
        os.makedirs(models_dir)

    # create environment
    env = HexapodEnv(render_mode=None) # Train without GUI for speed
    
    # Vectorize environment (optional for single env but good practice)
    env = DummyVecEnv([lambda: env])
    
    model = PPO(
        "MlpPolicy", 
        env, 
        verbose=1, 
        tensorboard_log=log_dir,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
    )
    
    print("Starting training...")
    TIMESTEPS = 10000 
    iters = 0
    
    # Check for existing models to resume
    existing_models = []
    if os.path.exists(models_dir):
        files = os.listdir(models_dir)
        for f in files:
            if f.endswith(".zip"):
                try:
                    step_count = int(f.replace(".zip", ""))
                    existing_models.append(step_count)
                except ValueError:
                    pass
    
    if existing_models:
        latest_step = max(existing_models)
        model_path = f"{models_dir}/{latest_step}.zip"
        print(f"Loading existing model to resume: {model_path}")
        model = PPO.load(model_path, env=env, tensorboard_log=log_dir)
        iters = latest_step // TIMESTEPS
    
    while True:
        iters += 1
        model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False)
        current_steps = TIMESTEPS * iters
        model.save(f"{models_dir}/{current_steps}")
        print(f"Saved model at {current_steps} steps")
        
        # Optional: Evaluate occasionally (commented out for pure training loop)
        # break # For testing script, just run once

if __name__ == "__main__":
    train()
