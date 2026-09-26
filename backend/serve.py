import sys
import os

# Ensure backend directory is in path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import uvicorn

if __name__ == "__main__":
    try:
        uvicorn.run("app.main:app", host="127.0.0.1", port=8000, log_level="info")
    except Exception as e:
        print(f"Uvicorn stopped with exception: {e}", file=sys.stderr)
