# Makefile for mlx90640 sensor application programs

CC = arm-linux-gnueabihf-gcc
CFLAGS = -Wall -O2

# Board transfer settings (for embedded board deployment)
BOARD_IP = 192.168.184.250  # Your board's IP address
BOARD_USER = root  # SSH username on the board
BOARD_PASS = 0000  # SSH password
BOARD_PATH = /home/feifei/linux/camera/  # Target directory on the board

# Windows transfer settings (modify these for your setup)
WINDOWS_IP = 192.168.1.100  # Replace with your Windows PC IP
WINDOWS_USER = your_username  # Replace with your Windows username
WINDOWS_PATH = C:/Users/your_username/Desktop/mlx90640  # Windows path example

# Target executables
TARGETS = mlx90640_lcd mlx90640_app mlx90640_rawdump camera_app dual_app

# Source files
SRCS = mlx90640_lcd.c mlx90640_app.c mlx90640_app_v5.c

# Default target: build all
all: $(TARGETS)

mlx90640_lcd: mlx90640_lcd.c
	$(CC) $(CFLAGS) -o $@ $^

mlx90640_app: mlx90640_app.c
	$(CC) $(CFLAGS) -std=c99 -lm -o $@ $^

camera_app: mlx90640_app_v5.c
	$(CC) $(CFLAGS) -std=c99 -lm -o $@ $^

mlx90640_test: mlx90640_test.c
	$(CC) $(CFLAGS) -o $@ $^

mlx90640_dump: mlx90640_dump.c
	$(CC) $(CFLAGS) -o $@ $^

mlx90640_rawdump: mlx90640_rawdump.c
	$(CC) $(CFLAGS) -std=c99 -o $@ $^

dual_app: mlx90640_app_dual.c
	$(CC) $(CFLAGS) -std=c99 -lm -pthread -o $@ $^

# Transfer compiled binaries to Windows PC via SCP
transfer: all
	scp $(TARGETS) $(WINDOWS_USER)@$(WINDOWS_IP):$(WINDOWS_PATH)

# Deploy compiled binaries to embedded board via SCP
deploy: all
	sshpass -p 0000 ssh root@192.168.184.250 "mkdir -p /home/feifei/linux/camera/"
	sshpass -p 0000 scp $(TARGETS) root@192.168.184.250:/home/feifei/linux/camera/

# Clean target: remove all executables
clean:
	rm -f $(TARGETS)

# Phony targets
.PHONY: all clean transfer deploy
