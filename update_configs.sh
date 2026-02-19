#!/bin/bash
# Automatically discover IPs and update Dockerfile/docker-compose.yml

echo "Searching for Sawyer robot..."
# Run find_robot.sh and capture the output
eval "$(./find_robot.sh)"

if [ -z "$ROBOT_IP" ] || [ -z "$HOST_IP" ]; then
    echo "Error: Could not detect Robot or Host IP."
    exit 1
fi

echo "Detected Robot IP: $ROBOT_IP"
echo "Detected Host IP: $HOST_IP"

# 1. Update Dockerfile
echo "Updating Dockerfile..."
sed -i "s/your_ip="[0-9.]*"/your_ip="$HOST_IP"/g" Dockerfile
sed -i "s/robot_hostname="[0-9.]*"/robot_hostname="$ROBOT_IP"/g" Dockerfile

# 2. Update docker-compose.yml
echo "Updating docker-compose.yml..."
sed -i "s|ROS_MASTER_URI=http://[0-9.]*:11311|ROS_MASTER_URI=http://$ROBOT_IP:11311|g" docker-compose.yml
sed -i "s/ROS_IP=[0-9.]*/ROS_IP=$HOST_IP/g" docker-compose.yml
sed -i "s/ROS_HOSTNAME=[0-9.]*/ROS_HOSTNAME=$HOST_IP/g" docker-compose.yml
sed -i "s/HOST_IP=\${HOST_IP:-[0-9.]*}/HOST_IP=\${HOST_IP:-$HOST_IP}/g" docker-compose.yml
sed -i "s/ROBOT_IP=\${ROBOT_IP:-[0-9.]*}/ROBOT_IP=\${ROBOT_IP:-$ROBOT_IP}/g" docker-compose.yml
sed -i "s/extra_hosts:.*$/extra_hosts:/g" docker-compose.yml # Prep for next line
sed -i "/extra_hosts:/!b;n;c\      - "021607CP00070.local:$ROBOT_IP"" docker-compose.yml

echo "Configuration updated! You can now run:"
echo "   docker-compose up -d --build robot_dev"
