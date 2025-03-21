docker build \
	--build-arg HOST_HOSTNAME=$(hostname) \
	--build-arg HOST_IP=$(hostname -I | cut -d ' ' -f 1) \
	-t robo2025-workspace workspace
