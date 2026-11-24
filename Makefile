# One-line entry points; each target just calls the script that owns the logic.
.PHONY: help check build fly evidence clean

help:
	@echo "make check     no-flight self-check (tests/run_all.sh)"
	@echo "make build     build the two colcon workspaces (setup/build.sh)"
	@echo "make fly TAG=  fly 2 laps and grade (setup/run_hex.sh TAG 2 2)"
	@echo "make evidence  re-render the four figures (scripts/render_evidence.sh TAG)"

check:
	bash tests/run_all.sh

build:
	bash setup/build.sh

fly:
	bash setup/run_hex.sh "$(or $(TAG),kx7)" 2 2

evidence:
	bash scripts/render_evidence.sh "$(or $(TAG),kx6-03-01-01)"

clean:
	rm -rf .build .runs
