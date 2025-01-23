FROM ros:jazzy

ENV TEAM_NUMBER=6
ENV ROS_DOMAIN_ID=$TEAM_NUMBER

echo "source /ros_entrypoint.sh" >> ~/.bashrc

