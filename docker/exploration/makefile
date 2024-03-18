build:
	docker build -f Dockerfile.exploration . -t multi_robot_exploration

start:
	docker start multi_robot_exploration

stop:
	docker stop multi_robot_exploration

run:
	docker run -itd --ipc host --net host --pid host --privileged -v $(PWD)/../../:/multi_robot_simulation --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix --name multi_robot_exploration multi_robot_exploration

kill:
	docker kill multi_robot_exploration

rm:
	docker rm multi_robot_exploration

attach:
	docker exec -it multi_robot_exploration bash
