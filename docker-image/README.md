sudo docker compose -f build-compose.yml build

sudo docker compose -f build-compose.yml push

to check supported platforms:
sudo docker run -it --rm --privileged tonistiigi/binfmt --install all