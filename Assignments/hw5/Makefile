RRT_DIR = ./RRT_planning/rrt
CONTROLLER_DIR = ./RRT_planning/controllers/LineFollowing

.PHONY: rrt
rrt:
	@echo "\033[0;36mRunning RRT ...\033[0m"
	cd $(RRT_DIR) && python3 rrt.py
	@echo "\033[0;32mRRT finished\033[0m"

.PHONY: ctrl
ctrl:
	@echo "\033[0;36mBuilding LineFollowing controller ...\033[0m"
	export WEBOTS_HOME=/usr/local/webots
	make clean -C $(CONTROLLER_DIR) --no-print-directory && make -C $(CONTROLLER_DIR) --no-print-directory
	@echo "\033[0;32mController building finished\033[0m"

.PHONY: clean
clean:
	-rm -f ./RRT_planning/rrt/*.png ./RRT_planning/textures/maze_*.png