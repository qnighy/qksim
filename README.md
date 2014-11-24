# qksim

## dependencies

- GCC with C++11/C99
- libboost-program-options-dev

## usage

```
$ make
...
$ ./qksim < examples/fib-loop.bin | hexdump -C
LW: End of File reached. Halt.
00000000  00 00 00 00 00 00 00 01  00 00 00 01 00 00 00 02  |................|
00000010  00 00 00 03 00 00 00 05  00 00 00 08              |............|
0000001c
$ ./qksim -s jit < examples/fib-loop.bin | hexdump -C
...
00000000  00 00 00 00 00 00 00 01  00 00 00 01 00 00 00 02  |................|
00000010  00 00 00 03 00 00 00 05  00 00 00 08              |............|
0000001c
$ ./qksim -h
simulator control:
  -s [ --sim ] arg (=ils) which implementation to use (ils,jit)
  -h [ --help ]           show help
```


