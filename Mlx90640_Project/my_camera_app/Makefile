# MLX90640 热成像应用 Makefile
# ARM交叉编译环境

CC = arm-linux-gnueabihf-gcc
CFLAGS = -Wall -O2 -std=c99
LDFLAGS = -lm -lrt

TARGET = camera_app
SRC = src/mlx90640_app_v4.c
OBJ = src/mlx90640_app_v4.o

.PHONY: all clean upload

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJ) $(LDFLAGS)
	@echo "✓ 编译完成: $(TARGET)"
	@ls -lh $(TARGET)

$(OBJ): $(SRC)
	$(CC) $(CFLAGS) -c $(SRC) -o $(OBJ)

clean:
	rm -f $(TARGET) $(OBJ)
	@echo "✓ 清理完成"

upload: $(TARGET)
	@echo "上传到 192.168.242.250..."
	sshpass -p "0000" scp -o StrictHostKeyChecking=no $(TARGET) root@192.168.242.250:/home/feifei/linux/camera/
	@echo "✓ 上传完成"