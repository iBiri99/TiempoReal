gcc -o iface main.c serial.c aux.c
cat /dev/ttyACM0 > /dev/null
./iface
