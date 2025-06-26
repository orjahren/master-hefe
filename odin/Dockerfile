FROM debian:latest

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
        python3 \
        python3-pip \
        python3-venv

RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create a virtual environment
RUN python3 -m venv /opt/venv   

# activate the virtual environment
ENV PATH="/opt/venv/bin:$PATH"

COPY requirements.txt /app/
# Set the working directory
WORKDIR /app

# Install Python packages
RUN pip install -r requirements.txt
# Copy the application code
COPY . /app
# Expose the port the app runs on
EXPOSE 4000

# Set the entrypoint to run the application
# It is a fastapi application
# CMD ["fastapi", "run", "--host", "0.0.0.0", "--port", "4000"]
CMD ["fastapi", "dev", "server.py", "--host", "0.0.0.0", "--port", "4000"]
