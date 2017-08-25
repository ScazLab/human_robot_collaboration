FROM scazlab/hrc-docker:baxter

RUN cd ~/ros_ws/src \
    && git clone https://github.com/scazlab/human_robot_collaboration.git
RUN cd ~/ros_ws/src \
    && wstool merge -y human_robot_collaboration/dependencies.rosinstall
RUN cd ~/ros_ws/src \
    && wstool up

USER root
RUN  cd ~/ros_ws \
     && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
USER ros
RUN  cd ~/ros_ws && catkin build

CMD ["/bin/bash"]
