default:
	podman-compose down
	#podman image rm thor:latest odin:latest
	podman-compose up --build
