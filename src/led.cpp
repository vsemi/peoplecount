#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include "led.h"

static char gpio_path[75];

int gpio_init(const char *name)
{
    int fd;
    //index config

    sprintf(gpio_path, "/sys/class/gpio/gpio%s", name);

    if (access("gpio_path", F_OK)){
        fd = open("/sys/class/gpio/export", O_WRONLY);
        if(fd < 0)
            return 1 ;
    
        write(fd, name, strlen(name));
        close(fd);
    
        //direction config
        sprintf(gpio_path, "/sys/class/gpio/gpio%s/direction", name);
        fd = open(gpio_path, O_WRONLY);
        if(fd < 0)
            return 2;
    
        write(fd, "out", strlen("out"));
        close(fd);
    }

    return 0;
}

int gpio_deinit(const char *name)
{
    int fd;
    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if(fd < 0)
        return 1;

    write(fd, name, strlen(name));
    close(fd);

    return 0;
}


int gpio_high(const char *name)
{
    int fd;
    sprintf(gpio_path, "/sys/class/gpio/gpio%s/value", name);
    fd = open(gpio_path, O_WRONLY);
    if(fd < 0){
        printf("open %s wrong\n",gpio_path);
        return -1;
    }
        
    if(2 != write(fd, "1", sizeof("1")))
        printf("wrong set \n");
    close(fd);
    return 0;
}


int gpio_low(const char *name)
{
    int fd;
    sprintf(gpio_path, "/sys/class/gpio/gpio%s/value", name);
    fd = open(gpio_path, O_WRONLY);
    if(fd < 0){
        printf("open %s wrong\n",gpio_path);
        return -1;
    }
        
    if(2 != write(fd, "0", sizeof("0")))
        printf("wrong set \n");
    close(fd);
    return 0;
}
