import sys
import os

root_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.join(root_dir, "backend")
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

import uvicorn

if __name__ == "__main__":
    print("=" * 65)
    print("  [GCS] Geoscan Fleet Mission Planner | Tactical GCS Platform")
    print("  [WEB] Web Interface: http://127.0.0.1:8000")
    print("  [DOC] API Docs:      http://127.0.0.1:8000/docs")
    print("=" * 65)
    try:
        uvicorn.run("app.main:app", host="127.0.0.1", port=8000, log_level="info", reload=False)
    except KeyboardInterrupt:
        print("\nServer stopped by user.")
    except Exception as e:
        print(f"\nUvicorn stopped with error: {e}", file=sys.stderr)
