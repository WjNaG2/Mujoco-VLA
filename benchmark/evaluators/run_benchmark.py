import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run benchmark placeholder.")
    parser.add_argument("--split", default="test", help="Dataset split")
    args = parser.parse_args()
    print(f"Run benchmark placeholder: split={args.split}")
