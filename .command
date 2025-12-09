git add .
git commit -m "update atc brake curve"
git push origin main

pip install fastapi uvicorn[standard]
cd tasc
uvicorn server:app --reload --host 127.0.0.1 --port 8000

+++
add curve track
add more sound effects 
add real rail texture
play around with brake/accel val to make em smoother
ktx/n700s atc update