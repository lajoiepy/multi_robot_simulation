build:
	docker build -f Dockerfile.exploration.devel . -t multi_robot_exploration_devel

start:
	docker start multi_robot_exploration_devel

stop:
	docker stop multi_robot_exploration_devel

run:
	docker run -itd --ipc host --net host --pid host --privileged -v $(PWD)/../../../:/multi_robot_simulation --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix --name multi_robot_exploration_devel multi_robot_exploration_devel

kill:
	docker kill multi_robot_exploration_devel

rm:
	docker rm multi_robot_exploration_devel

attach:
	docker exec -it multi_robot_exploration_devel bash
