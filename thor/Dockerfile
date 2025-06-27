# Image for running Carla test cases 
FROM ubuntu:22.04
# Install dependencies
RUN apt-get update && \
    apt-get install -y \
        python3 \
        python3-pip \
        python3-venv \
        wait-for-it
    
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*


# Create a virtual environment
RUN python3 -m venv /opt/venv
# activate the virtual environment
ENV PATH="/opt/venv/bin:$PATH"
# Set the working directory
WORKDIR /app    
# Copy the requirements file
COPY requirements.txt /app/
# Install Python packages
RUN pip install --no-cache-dir -r requirements.txt
# Copy the application code
COPY . /app
# Expose the port the app runs on
EXPOSE 5000
ENV PYTHONPATH=/app
RUN pwd
RUN ls
RUN ls /app
# NOTE: wait-for-it is used to wait for RabbitMQ to be ready so we don't have to wait for it in the code
CMD ["wait-for-it", "localhost:5672", "--", "uvicorn", "server:app", "--host", "0.0.0.0", "--port", "5000", "--workers", "1"]
