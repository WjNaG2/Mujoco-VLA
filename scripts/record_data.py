import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record data placeholder.")
    parser.add_argument("--output", default="data/raw", help="Output data directory")
    args = parser.parse_args()
    print(f"Record data placeholder: output={args.output}")
