build:
	docker build -f Dockerfile.swarmslam . -t swarmslam

start:
	docker start swarmslam

stop:
	docker stop swarmslam

run:
	docker run -itd --ipc host --net host --pid host --privileged -v $(PWD)/../../:/multi_robot_simulation --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix --name swarmslam swarmslam

kill:
	docker kill swarmslam

rm:
	docker rm swarmslam

attach:
	docker exec -it swarmslam bash
