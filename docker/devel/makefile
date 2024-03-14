build:
	docker build -f Dockerfile.devel . -t ros2devel

start:
	docker start ros2devel

stop:
	docker stop ros2devel

run:
	docker run -itd --ipc host --net host --pid host --privileged -v $(PWD)/../../:/multi_robot_simulation --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix --name ros2devel ros2devel

kill:
	docker kill ros2devel

rm:
	docker rm ros2devel

attach:
	docker exec -it ros2devel bash
