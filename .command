git add .
git commit -m "add acceleration smoothing for power-on sequence"
git push origin main

pip install fastapi uvicorn[standard]
cd tasc
uvicorn server:app --reload --host 127.0.0.1 --port 8000

+++
add curve track
play around with brake/accel val to make em smoother