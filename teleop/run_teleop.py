import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run XR teleoperation placeholder.")
    parser.add_argument("--mode", default="demo", help="Teleop mode")
    args = parser.parse_args()
    print(f"Run teleop placeholder: mode={args.mode}")
