FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PORT=5050

WORKDIR /app

ARG PIP_INDEX_URL=https://pypi.org/simple
COPY backend/requirements.txt backend/requirements.txt
RUN pip install --no-cache-dir -i ${PIP_INDEX_URL} -r backend/requirements.txt

COPY backend/ backend/
COPY frontend/ frontend/

EXPOSE 5050

WORKDIR /app/backend
CMD ["python", "app.py"]
