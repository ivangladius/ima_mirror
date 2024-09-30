# Use Ubuntu as the base image
FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install basic utilities and Flutter dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl unzip xz-utils zip libglu1-mesa wget ca-certificates gnupg \
    && rm -rf /var/lib/apt/lists/*

# Install Flutter SDK
RUN git clone https://github.com/flutter/flutter.git /flutter
ENV PATH="/flutter/bin:/flutter/bin/cache/dart-sdk/bin:${PATH}"

# Enable Flutter web
RUN flutter channel stable && \
    flutter upgrade && \
    flutter config --enable-web

# Set the working directory
WORKDIR /app

# Copy the entire project into the container
COPY . .

# Get app dependencies and build for web
RUN flutter pub get && \
    flutter build web --release

# Make sure start.sh is executable
RUN chmod +x start.sh

# Expose port 8080
EXPOSE 8080

# Set the entry point to serve the built web app
ENTRYPOINT ["flutter", "run", "--release", "-d", "web-server", "--web-port", "8080", "--web-hostname", "0.0.0.0"]