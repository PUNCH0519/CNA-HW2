#!/bin/bash
gcc -Wall -std=c99 -pedantic -o gbn emulator.c gbn.c
gcc -Wall -std=c99 -pedantic -o sr  emulator.c sr.c
echo "Build finished."
