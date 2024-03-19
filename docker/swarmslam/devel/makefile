build:
	docker build -f Dockerfile.swarmslam.devel . -t swarmslam_devel

start:
	docker start swarmslam_devel

stop:
	docker stop swarmslam_devel

run:
	docker run -itd -v $(PWD)/../../../:/multi_robot_simulation --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix --name swarmslam_devel swarmslam_devel
#	docker run -itd --network=none -v $(PWD)/../../../:/multi_robot_simulation --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix --name swarmslam_devel swarmslam_devel

up:
	docker-compose -f docker-compose.swarmslam.devel.yml up -d 

down:
	docker-compose -f docker-compose.swarmslam.devel.yml down

kill:
	docker kill swarmslam_devel

rm:
	docker rm swarmslam_devel

attach:
	docker exec -it swarmslam_devel bash
