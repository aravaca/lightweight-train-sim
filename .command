git add .
git commit -m "fix tasc bug of not accelerating"
git push origin main

pip install fastapi uvicorn[standard]
uvicorn server:app --reload --host 127.0.0.1 --port 8000

+++
add curve track
play around with brake/accel val to make em smoother