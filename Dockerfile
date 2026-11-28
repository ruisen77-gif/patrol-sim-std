FROM python:3.11-slim

WORKDIR /app

COPY patrol-sim-std/backend /app/backend
COPY patrol-sim-std/frontend /app/frontend

WORKDIR /app/backend

RUN pip install -r requirements.txt

EXPOSE 5000

CMD ["python","app.py"]
