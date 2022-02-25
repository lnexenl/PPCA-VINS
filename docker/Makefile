all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build all images"
	# @echo "   1. make pull             - pull all images"
	@echo "   1. make clean              - remove all images"
	@echo ""

build:
	@docker build --tag ros:vins-fusion -f ./Dockerfile .. --network host --build-arg http_proxy=http://127.0.0.1:10809 --build-arg https_proxy=http://127.0.0.1:10809

clean:
	@docker rmi -f ros:vins-fusion
