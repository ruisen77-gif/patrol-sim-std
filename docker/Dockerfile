FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PORT=5050

WORKDIR /app

ARG PIP_INDEX_URL=https://pypi.org/simple
COPY requirements.txt .
RUN pip install --no-cache-dir -i ${PIP_INDEX_URL} -r requirements.txt

COPY backend/ backend/
COPY frontend/ frontend/
COPY module_algorithm/ module_algorithm/
COPY module_ai_sensor/ module_ai_sensor/
COPY module_embed_sim/ module_embed_sim/
COPY docs/ docs/

EXPOSE 5050

CMD ["python", "backend/app.py"]
