import rclpy
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from rl_navigation_pkg.drone_env import DroneEnv
import os

def main(args=None):
    rclpy.init(args=args)
    
    env = DroneEnv()
    
    # Optional: Check if environment follows gym API
    # check_env(env)
    
    model_path = os.path.join(os.getcwd(), "ppo_drone_nav")
    
    print("Starting PPO Training for Drone Navigation...")
    # Initialize PPO agent
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        tensorboard_log="./ppo_drone_tensorboard/"
    )
    
    try:
        # Train the model
        model.learn(total_timesteps=100000, progress_bar=True)
        # Save the model
        model.save(model_path)
        print(f"Model saved to {model_path}.zip")
    except KeyboardInterrupt:
        print("Training interrupted! Saving current progress...")
        model.save(model_path)
    finally:
        env.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
