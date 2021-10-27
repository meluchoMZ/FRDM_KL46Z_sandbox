# FRDM_KL46Z_sandbox
Compile, debug and execute on the KL46Z board with only one command \
> make 

Compiles, links and creates a .elf file.
> make binary

Compiles links and creates a .bin file.

> make flash

Compiles links (if needed) loads the .elf file into the board

> make install

Same as make flash but loads the .bin into the board
> make clean

removes .o, .elf, .map and .bin files

