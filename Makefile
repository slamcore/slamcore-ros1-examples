# configuration ----------------------------------------------------------------
define announce
	@echo "===================================================================="
	@echo $(1)
	@echo "===================================================================="
	@echo
endef

default: run

SHELL:=/bin/bash
THIS_FILE:=$(lastword $(MAKEFILE_LIST))
THIS_DIR:=$(dir $(abspath ${THIS_FILE}))
STAMPS:=${THIS_DIR}/.stamps
$(shell mkdir -p ${STAMPS})
DOCKER_NAME:=slamcore-navstack-demo
DOCKERFILE_DIR=${THIS_DIR}
DOCKER_USER:=$(shell whoami)

# targets ----------------------------------------------------------------------

setup_host:
	$(call announce,"Installing packages required on the host...")
	dpkg -l | grep -q pciutils || sudo apt install -y pciutils
	@touch ${STAMPS}/setup_host.stamp

.PHONY: build
build:
	$(call announce,"Building container...")

	docker build ${DOCKERFILE_DIR} \
			--build-arg USERNAME=${DOCKER_USER} \
			--build-arg UID=$(shell id -u) \
			--build-arg GID=$(shell id -g) \
			--build-arg BUILDBOT_ADDR=${BUILDBOT_ADDR} \
			-t ${DOCKER_NAME}

.PHONY: run
run: build setup_host
	$(call announce,"Starting the container...")

	docker_cmd="docker run"; \
	lspci | grep VGA | grep -qi NVIDIA; \
	ret1=$$?; \
	docker run --rm --gpus all 2>/dev/null; \
	ret2=$$?; \
	if [[ $$ret1 == 0 && $$ret2 != 125 ]]; then \
		docker_cmd="docker run --rm --gpus all"; \
	fi; \
	$$docker_cmd --volume "${THIS_DIR}":"/home/${DOCKER_USER}/ros_ws/" \
		--volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume /dev/:/dev/ \
		--volume /var/run/dbus/:/var/run/dbus/ \
		--volume ${HOME}/.bash_history:/home/${DOCKER_USER}/.bash_history \
		--network host  \
		--ipc=host \
		--privileged \
		--env DISPLAY=${DISPLAY} \
		--env=QT_X11_NO_MITSHM=1 \
		-it --name ${DOCKER_NAME} ${DOCKER_NAME}


connect:
	@# assuming that only one container of the said image is running
	$(eval container=$(shell docker ps -q --filter "name=${DOCKER_NAME}"))
	$(call announce,"Connecting to running container -> ${container}")
	docker exec -it ${container} "bash"

.PHONY: clean
clean:
	rm -rf ${STAMPS}
	-docker stop ${DOCKER_NAME}
	-docker container rm ${DOCKER_NAME}

.PHONY: purge
purge: clean
	-docker rmi -f ${DOCKER_NAME}

