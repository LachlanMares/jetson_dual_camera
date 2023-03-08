all: up exec down

build:
	cd docker && docker-compose build

up:
	cd docker && docker compose up --detach

exec:
	-cd docker && docker-compose exec camera_rig bash

down:
	cd docker && docker-compose down

# Ensures that build is not linked to a file, allowing it to always be run.
.PHONY: build